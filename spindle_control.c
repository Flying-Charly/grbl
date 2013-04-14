/*
  spindle_control.c - spindle control methods
  Part of Grbl

  Copyright (c) 2009-2011 Simen Svale Skogsrud
  Copyright (c) 2012 Sungeun K. Jeon

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

#include "settings.h"
#include "spindle_control.h"
#include "planner.h"

#ifdef SPINDLE_ON_I2C
#include "i2c_tcb.h"
#include "MCP23017.h"

uint8_t spindle_tcb[5];


#endif

// this hack is supposed to help avr-gcc do 8-bit instead of 16-bit ops
static inline uint8_t only8 (uint8_t x) { return x; }

static uint8_t current_direction;

void spindle_init()
{
  current_direction = 0;
#ifdef SPINDLE_PRESENT
#ifdef SPINDLE_ON_I2C
  // fill spindle tcb to set output port bit directions
  spindle_tcb[0] = TCB_WRT_MASKED | (1<<TCB_REG_SHIFT) | TCB_INPLACE; //flags
  spindle_tcb[1] = MCP23017_UNIT0; // device address
  spindle_tcb[2] = MCP23017_IODIRB; // register
  spindle_tcb[3] = 0x00; // data bit values (clear)
  spindle_tcb[4] = (1 << SPINDLE_ENABLE_BIT) | (1 << SPINDLE_DIRECTION_BIT); // data mask
  queue_TWI((struct tcb*)spindle_tcb);
  while(only8(spindle_tcb[0]&TCB_COMPL)) { }; // block until transaction complete
  // all future transactions will be to the output latch register
  spindle_tcb[2] = MCP23017_OLATB; // register
#else  
  SPINDLE_ENABLE_DDR |= (1<<SPINDLE_ENABLE_BIT);
  SPINDLE_DIRECTION_DDR |= (1<<SPINDLE_DIRECTION_BIT); 
#endif
#endif
  spindle_stop();
}

void spindle_stop()
{
#ifdef SPINDLE_PRESENT
#ifdef SPINDLE_ON_I2C
  while(only8(spindle_tcb[0]&TCB_COMPL)) { }; // block if a spindle I2C transaction is pending
  spindle_tcb[3] = 0x00; // data bit values (clear)
  spindle_tcb[4] = (1 << SPINDLE_ENABLE_BIT); // data mask
  queue_TWI((struct tcb*)spindle_tcb);
#else
  SPINDLE_ENABLE_PORT &= ~(1<<SPINDLE_ENABLE_BIT);
#endif
#endif
}

// direction is 1, -1, 0 for M3, M4, M5
void spindle_run(int8_t direction) //, uint16_t rpm) 
{
#ifdef SPINDLE_PRESENT
  if (direction != current_direction) {
    plan_synchronize();
    
#ifdef SPINDLE_ON_I2C
    while(only8(spindle_tcb[0]&TCB_COMPL)) { }; // block if a spindle I2C transaction is pending
    if(direction) {
      if (direction < 0) {
        spindle_tcb[3] = (1 << SPINDLE_ENABLE_BIT) | (1 << SPINDLE_DIRECTION_BIT) ;
      } else {
        spindle_tcb[3] = (1 << SPINDLE_ENABLE_BIT);
      }
      spindle_tcb[4] = (1 << SPINDLE_ENABLE_BIT) | (1 << SPINDLE_DIRECTION_BIT) ;
      queue_TWI((struct tcb*)spindle_tcb);
#else
    if(direction) {
      if(direction > 0) {
        SPINDLE_DIRECTION_PORT &= ~(1<<SPINDLE_DIRECTION_BIT);
      } else {
        SPINDLE_DIRECTION_PORT |= 1<<SPINDLE_DIRECTION_BIT;
      }
      SPINDLE_ENABLE_PORT |= 1<<SPINDLE_ENABLE_BIT;
#endif

    } else {
      spindle_stop();
    }
  current_direction = direction;
  }
#endif
}
