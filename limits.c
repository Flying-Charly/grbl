/*
  limits.c - code pertaining to limit-switches and performing the homing cycle
  Part of Grbl

  Copyright (c) 2009-2011 Simen Svale Skogsrud
  Copyright (c) 2012 Sungeun K. Jeon
  Copyright (c) 2012 Chuck Harrison for http://opensourceecology.org/wiki/CNC_Torch_Table

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/
  
#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "stepper.h"
#include "settings.h"
#include "nuts_bolts.h"
#include "config.h"
#include "spindle_control.h"
#include "motion_control.h"
#include "planner.h"
#include "protocol.h"
#include "limits.h"
#include "report.h"
#ifdef USE_I2C_LIMITS
#include "MCP23017.h"
#include "i2c_tcb.h"
#endif

#include "print.h"
#include <avr/pgmspace.h>

#define MICROSECONDS_PER_ACCELERATION_TICK  (1000000/ACCELERATION_TICKS_PER_SECOND)

#ifndef USE_I2C_LIMITS
void limits_init() 
{
  LIMIT_DDR &= ~(LIMIT_MASK); // Set as input pins
  LIMIT_PORT |= (LIMIT_MASK); // Enable internal pull-up resistors. Normal high operation.

  if (bit_istrue(settings.flags,BITFLAG_HARD_LIMIT_ENABLE)) {
    LIMIT_PCMSK |= LIMIT_MASK; // Enable specific pins of the Pin Change Interrupt
    PCICR |= (1 << LIMIT_INT); // Enable Pin Change Interrupt
  } else {
    LIMIT_PCMSK &= ~LIMIT_MASK; // Disable
    PCICR &= ~(1 << LIMIT_INT); 
  }
}

// This is the Limit Pin Change Interrupt, which handles the hard limit feature. A bouncing 
// limit switch can cause a lot of problems, like false readings and multiple interrupt calls.
// If a switch is triggered at all, something bad has happened and treat it as such, regardless
// if a limit switch is being disengaged. It's impossible to reliably tell the state of a 
// bouncing pin without a debouncing method.
// NOTE: Do not attach an e-stop to the limit pins, because this interrupt is disabled during
// homing cycles and will not respond correctly. Upon user request or need, there may be a
// special pinout for an e-stop, but it is generally recommended to just directly connect
// your e-stop switch to the Arduino reset pin, since it is the most correct way to do this.
ISR(LIMIT_INT_vect) 
{
  // TODO: This interrupt may be used to manage the homing cycle directly with the main stepper
  // interrupt without adding too much to it. All it would need is some way to stop one axis 
  // when its limit is triggered and continue the others. This may reduce some of the code, but
  // would make Grbl a little harder to read and understand down road. Holding off on this until
  // we move on to new hardware or flash space becomes an issue. If it ain't broke, don't fix it.

  // Ignore limit switches if already in an alarm state or in-process of executing an alarm.
  // When in the alarm state, Grbl should have been reset or will force a reset, so any pending 
  // moves in the planner and serial buffers are all cleared and newly sent blocks will be 
  // locked out until a homing cycle or a kill lock command. Allows the user to disable the hard
  // limit setting if their limits are constantly triggering after a reset and move their axes.
  if (sys.state != STATE_ALARM) { 
    if (bit_isfalse(sys.execute,EXEC_ALARM)) {
      mc_reset(); // Initiate system kill.
      sys.execute |= EXEC_CRIT_EVENT; // Indicate hard limit critical event
    }
  }
}
#else

void limits_init() { 

} // not necessary with ose I2C limits
#endif

// Moves each axis in a trapezoidal move with axis-specific accel, decel, and slew speed
// Homing is a special motion case, where there is only an 
// acceleration followed by decelerated stops by each axes reaching their limit 
// switch independently. 
//
// Homing uses an independent-axis algorithm: upon hitting its home switch, one axis can
// do controlled deceleration while other axes continue to seek. Unlike the main stepper
// algorithm, there is no coordinated movement between axes.
// The update rate is constant and an independent 2nd-order phase accumulator handles each axis.
// We share the position counters from the stepper state variable st defined in stepper.c


// NOTE: Only the abort runtime command can interrupt this process.


// these data structures allow independent parameters for each axis
static struct {
  float rate[2]; // fast (seek) and slow (feed) rates mm/min
  float decel; 
  uint8_t accel_ratio;
} home_params[3];

void home_init() {
home_params[X_AXIS].rate[0] = settings.homing_seek_rate; // mm/min
home_params[X_AXIS].rate[1] = settings.homing_feed_rate; // mm/min
home_params[X_AXIS].decel = home_params[X_AXIS].rate[0]*60./0.1; // mm/min^2; 0.5 sec to stop
home_params[X_AXIS].accel_ratio = 20;
home_params[Y_AXIS].rate[0] = settings.homing_seek_rate; // mm/min
home_params[Y_AXIS].rate[1] = settings.homing_feed_rate; // mm/min
home_params[Y_AXIS].accel_ratio = 50;
home_params[Y_AXIS].decel = home_params[Y_AXIS].rate[0]*60./0.5; // mm/min^2; 0.5 sec to stop
home_params[Z_AXIS].rate[0] = settings.homing_seek_rate; // mm/min
home_params[Z_AXIS].rate[1] = settings.homing_feed_rate; // mm/min
home_params[Z_AXIS].accel_ratio = 50;
home_params[Z_AXIS].decel = home_params[Z_AXIS].rate[0]*60./0.1; // mm/min^2; 0.1 sec to stop
}

#ifdef USE_I2C_LIMITS
inline uint8_t home_limit_state() {
  return QUICKREAD_DATA(0);
  //return quickread_data(0); // updated in background by MCP23017_interrupt
}
#else
inline uint8_t home_limit_state() {
  return HOME_PIN;
}
#endif
// This function is used inside the Stepper Driver Interrupt when in independent-axis mode
//   it includes the complete state machine for a trapezoidal move with separate accel and
//   decel values. The indep_t structure is defined in stepper.h 
// This supports move-to-target-position as well as stop-at-home-switch functions.
bool indep_increment(indep_t_ptr ht)
{
  if(ht->state==idle || ht->state==done || ht->state==fault)
    return false;
  if(!(ht->flags & INDEP_NO_TARGET) && sys.position[ht->axis] == ht->target_pos) {
    ht->state = done;
    return false;
  }
  if(ht->state==decel) {
    ht->dpdt -= ht->d2pdt2d;
    if(ht->dpdt<=0) {
      if((ht->flags & INDEP_HOMING) || (sys.state == STATE_HOLD)) {
        ht->state = done;
        return false;
      } else {
        ht->dpdt = ht->dpdt_min;
      }
    }
  }
  if(ht->state==accel) {
    ht->dpdt += ht->d2pdt2a;
    if(ht->dpdt>=ht->dpdt_max) {
      ht->dpdt = ht->dpdt_max;
      ht->state = slew;
    }
  }
  if(ht->state!= decel) {
    if((ht->flags & INDEP_HOMING) 
       && (ht->home_mask & (home_limit_state()^ ht->home_nullstate)) ) {
      ht->state = decel;
      sys.position[ht->axis] = 0;
      ht->flags |= INDEP_HIT_HOME;
    } else if ( (!(ht->flags & INDEP_NO_TARGET)
                && (sys.position[ht->axis]==ht->decel_pos))
               || (sys.state == STATE_HOLD) ) {
      ht->state = decel;
    }
  }
  return true;
}

// Start the stepper driver in independent mode, and wait for it to complete
static void run_independent_move(indep_t_ptr frame) { 
  #ifdef USE_I2C_LIMITS
  // capture current home/limit state (needed at cold start)
  QUEUE_QUICKREAD(0);
  delay_ms(2); // a generous wait for I2C operation to complete
  #endif
  for(;;) {
    if(!indep_mode) {
      st_indep_start(frame);
    }
    // Check if we are done or for system abort
    protocol_execute_runtime();
    bool complete = true;   
    indep_t_ptr it = frame;
    while(it) {
      if(it->state==accel || it->state==slew || it->state==decel) {
        complete = false; }
      it = it->next_axis;
    }

    if(complete || sys.abort) {
      st_go_idle();
      indep_mode = false;
      return;
    }
  }
}

// some mnemonics for homing_cycle function arguments
const uint8_t ax_stop=0, ax_fast=1, ax_slow=2; // axis movement speed
const uint8_t slave_stop=0, slave_run=1, slave_run_watch_both=2; // x2 axis
const uint8_t dir_pos=1, dir_neg=0;
void homing_cycle(uint8_t x_axis, uint8_t x2_axis, uint8_t y_axis, uint8_t z_axis, uint8_t pos_dir) 
{
  uint8_t n = (x_axis!=0) + (y_axis!=0) + (z_axis!=0);
  if(n==0) { return; }

  uint32_t dt = 1000000L/HOME_EVENTS_PER_SECOND; // usec/step

  // Set default out_bits for stepper drive directions.
  out_bits0 = (out_bits0 & ~DIRECTION_MASK)
              | (settings.homing_dir_mask & DIRECTION_MASK); // Apply homing direction settings
  if (!pos_dir) { out_bits0 ^= DIRECTION_MASK; }   // Invert bits, if negative dir.
  
  // enable stepper drives (optionally enable slave axis X2)
  // TBD: handle settings.flags BITFLAG_INVERT_ST_ENABLE 
  uint8_t disable_bits_to_set;
  if(x2_axis) { disable_bits_to_set = STEPPERS_DISABLE_INVERT_MASK & STEPPERS_DISABLE_MASK; }
  else { disable_bits_to_set = (STEPPERS_DISABLE_INVERT_MASK^(1<<X2_DISABLE_BIT)) & STEPPERS_DISABLE_MASK; }
  STEPPERS_DISABLE_PORT = (STEPPERS_DISABLE_PORT & ~STEPPERS_DISABLE_MASK) | disable_bits_to_set;

  // Create the n frames defining n independent movements (n = number of axes moving)
  struct indep_t hm[n];
  
  uint8_t ax, ax_val=0;
  indep_t_ptr prev = NULL;
  indep_t_ptr this_axis = &hm[0];
  for(ax=0; ax<3; ax++) {
    switch(ax) {
    case X_AXIS:
      ax_val = x_axis;
      if(ax_val==0) continue;
      if(x2_axis) {
        this_axis->home_mask = 1<<X2_HOME_BIT;
        if(x2_axis==slave_run_watch_both) {
          this_axis->home_mask |= 1<<X_HOME_BIT;
        }
      } else {
        this_axis->home_mask = 1<<X_HOME_BIT;
      }
      break;
    case Y_AXIS:
      ax_val = y_axis;
      if(ax_val==0) continue;
      this_axis->home_mask = 1<<Y_HOME_BIT;
      break;
    case Z_AXIS:
      ax_val = z_axis;
      if(ax_val==0) continue;
      this_axis->home_mask = 1<<Z_HOME_BIT;
      break;
    }
    this_axis->axis = ax;
    this_axis->target_pos = 0;
    this_axis->decel_pos = 0;
    this_axis->dpdt = 0; // rate, phase LSB's/tick
    this_axis->d2pdt2d = (uint32_t)(INDEP_EVENT_COUNT/(1000000.*60.*1000000.*60.)
                                    *home_params[ax].decel*settings.steps_per_mm[ax]*dt*dt);
    this_axis->d2pdt2a = (uint32_t)( (this_axis->d2pdt2d*home_params[ax].accel_ratio)/100 );
    this_axis->dpdt_max = (uint32_t)(INDEP_EVENT_COUNT/(1000000.*60.)
                                     *home_params[ax].rate[ax_val-1]*settings.steps_per_mm[ax]*dt);
    this_axis->dpdt_min = (uint32_t)(INDEP_EVENT_COUNT/(1000000.*60.)*MINIMUM_STEPS_PER_MINUTE*dt);
    this_axis->home_nullstate = pos_dir ? 0 : 0xFF;
    this_axis->state = accel;
    this_axis->flags = INDEP_HOMING | INDEP_NO_TARGET;
    this_axis->next_axis = NULL;
    this_axis->prev_axis = prev;
    if(prev) {
      prev->next_axis = this_axis;
    }
    prev = this_axis;
    this_axis++;
  }
  
  run_independent_move(hm);
}

void limits_go_home() 
{
  plan_synchronize();  // Empty all motions in buffer.
  // Z-axis homing
  homing_cycle(ax_stop, slave_stop, ax_stop, ax_fast, dir_neg); // z axis approach home
  homing_cycle(ax_stop, slave_stop, ax_stop, ax_slow, dir_pos); // z back off

  // X- and Y-axis seek home simultaneously
  homing_cycle(ax_fast, slave_run_watch_both, ax_fast, ax_stop, dir_pos); // x and y axis away from home
  homing_cycle(ax_fast, slave_run_watch_both, ax_fast, ax_stop, dir_neg); // x and y axis approach home
  homing_cycle(ax_stop, slave_stop, ax_slow, ax_stop, dir_pos); // y back off 
  // X-axis master/slave homing
  // jog back and forth until slave enters home first
  while(!(home_limit_state() & (1<<X_HOME_BIT))) { // master in home, slave not
    homing_cycle(ax_slow, slave_stop, ax_stop, ax_stop, dir_pos); // back out master only 
    homing_cycle(ax_slow, slave_run_watch_both, ax_stop, ax_stop, dir_neg); // try again
  } 
  // now square up x axis
  int8_t n_cycle = N_HOMING_CYCLE;
  while (n_cycle--) {
    // using both motors, find slave home switch departure
    homing_cycle(ax_slow, slave_run, ax_stop, ax_stop, dir_neg);
    homing_cycle(ax_slow, slave_run, ax_stop, ax_stop, dir_pos);
    // using master motor only, find master home switch departure
    homing_cycle(ax_slow, slave_stop, ax_stop, ax_stop, dir_neg);
    homing_cycle(ax_slow, slave_stop, ax_stop, ax_stop, dir_pos);
  }

#if 0
  // sample code to test independent point-to-point positioning
  uint8_t x_axis=2, y_axis=2, z_axis=1;
  uint8_t n = (x_axis!=0) + (y_axis!=0) + (z_axis!=0);
  uint32_t dt = 1000000L/HOME_EVENTS_PER_SECOND; // usec/step
  out_bits0 = (out_bits0 & ~DIRECTION_MASK);
  // TBD: handle settings.flags BITFLAG_INVERT_ST_ENABLE 
  STEPPERS_DISABLE_PORT = (STEPPERS_DISABLE_PORT & ~STEPPERS_DISABLE_MASK)
                          | (STEPPERS_DISABLE_INVERT_MASK & STEPPERS_DISABLE_MASK);
                          
  // Create the n frames defining n independent movements (n = number of axes moving)
  struct indep_t hm[n];
  uint8_t ax, ax_val=0;
  indep_t_ptr prev = NULL;
  indep_t_ptr this_axis = &hm[0];
  
  for(ax=0; ax<3; ax++) {
    int32_t start_pos = sys.position[ax];
    int32_t target_pos = 5.*(ax+1)*settings.steps_per_mm[ax];
    bool negative_move = (target_pos < start_pos);
    
    switch(ax) {
    case X_AXIS:
      ax_val = x_axis;
      if(ax_val==0) continue;
      out_bits0 |= (1<<X_DIRECTION_BIT)
                   & (negative_move ? ~settings.invert_mask
                                    : settings.invert_mask);
      break;
    case Y_AXIS:
      ax_val = y_axis;
      if(ax_val==0) continue;
      out_bits0 |= (1<<Y_DIRECTION_BIT)
                   & (negative_move ? ~settings.invert_mask
                                    : settings.invert_mask);
      break;
    case Z_AXIS:
      ax_val = z_axis;
      if(ax_val==0) continue;
      out_bits0 |= (1<<Z_DIRECTION_BIT)
                   & (negative_move ? ~settings.invert_mask
                                    : settings.invert_mask);
      break;
    }
    this_axis->axis = ax;
    this_axis->target_pos = target_pos;
    
    // mm position from end at which we would need to start decelerating if no plateau
    float decel_dist_no_slew =  ( (this_axis->target_pos - start_pos)/settings.steps_per_mm[ax] )
                              *( 1. - 1./(1. + home_params[ax].accel_ratio/100.) );
    if(negative_move) { decel_dist_no_slew *= -1.; }
    // mm position from end at which we would need to start decelerating from plateau
    float decel_dist_from_slew = home_params[ax].rate[ax_val-1]*home_params[ax].rate[ax_val-1]
                                 /(2.* home_params[ax].decel);
    float decel_dist = ((decel_dist_no_slew) < (decel_dist_from_slew)) ? (decel_dist_no_slew) : (decel_dist_from_slew);
    this_axis->decel_pos = this_axis->target_pos
                           - (int32_t)(decel_dist*settings.steps_per_mm[ax]
                                       *(negative_move ? -1.0 : 1.0));
                           
    this_axis->dpdt = 0; // rate, LSB's/tick
    this_axis->d2pdt2d = (uint32_t)(INDEP_EVENT_COUNT/(1000000.*60.*1000000.*60.)
                                    *home_params[ax].decel*settings.steps_per_mm[ax]*dt*dt);
    this_axis->d2pdt2a = (uint32_t)( (this_axis->d2pdt2d*home_params[ax].accel_ratio)/100 );
    this_axis->dpdt_max = (uint32_t)(INDEP_EVENT_COUNT/(1000000.*60.)
                                     *home_params[ax].rate[ax_val-1]*settings.steps_per_mm[ax]*dt);
    this_axis->dpdt_min = (uint32_t)(INDEP_EVENT_COUNT/(1000000.*60.)*MINIMUM_STEPS_PER_MINUTE*dt);
    this_axis->home_nullstate = 0;
    this_axis->state = accel;
    this_axis->flags = 0;
    this_axis->next_axis = NULL;
    this_axis->prev_axis = prev;
    if(prev) {
      prev->next_axis = this_axis;
    }
    prev = this_axis;
    this_axis++;
  }
  run_independent_move(hm);
#endif  

  st_go_idle(); // Call main stepper shutdown routine.  
}
