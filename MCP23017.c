/*************************************************** 
  This is a limited-function library for the MCP23017 i2c port expander

  (c) 2012-13 Chuck Harrison for http:/opensourceecology.org
  BSD license
  (however check license of twi.c against which it links)
  
  Inspired by Adafruit_MCP23017.c whose copyright notice appears below
  > Adafruit invests time and resources providing this open source code, 
  > please support Adafruit and open-source hardware by purchasing 
  > products from Adafruit!
  > 
  > Written by Limor Fried/Ladyada for Adafruit Industries.  
  > BSD license, all text above must be included in any redistribution
 ****************************************************/

#include "i2c_tcb.h"
#include <avr/pgmspace.h>
#include "MCP23017.h"
#include "config.h"
#include <avr/interrupt.h>

#include "print.h"
#include <avr/pgmspace.h>

////////////////////////////////////////////////////////////////////////////////
void init_MCP23017_interrupt(); // forward declaration



void MCP23017_begin(uint8_t i2caddr) {
  volatile uint8_t init_tcb[4];   // TCB defn for startup operations (write 1 byte)
  twi_init();
  twi_releaseBus();
  // TBD: replace this with tcb operations
  // set defaults
  // All  pins input at startup (some B pins will become outputs later, e.g for spindle control)
  init_tcb[0] = TCB_WRT | (1<<TCB_REG_SHIFT) | TCB_INPLACE, //flags
  init_tcb[1] = i2caddr;
  init_tcb[2] = MCP23017_IODIRA;
  init_tcb[3] = 0xFF;
  queue_TWI((struct tcb*)init_tcb);
  while(init_tcb[0]&TCB_COMPL) { }; // block until transaction complete
  init_tcb[2] = MCP23017_IODIRB;
  init_tcb[3] = 0xFF;
  queue_TWI((struct tcb*)init_tcb);
  while(init_tcb[0]&TCB_COMPL) { }; // block until transaction complete
  
  #ifdef USE_I2C_LIMITS
  // set up IOCON.SEQOP=0, BANK=0 
  //  also INT output is active low, active driver.
  init_tcb[2] = MCP23017_IOCONA;
  init_tcb[3] = 0x00; // obsolete, IOCON.SEQOP=1: 0x20; 
  queue_TWI((struct tcb*)init_tcb);
  while(init_tcb[0]&TCB_COMPL) { }; // block until transaction complete
  // set up INTCONA for interrupt on change (same as power-on default)
  init_tcb[2] = MCP23017_INTCONA;
  init_tcb[3] = 0x00;
  queue_TWI((struct tcb*)init_tcb);
  while(init_tcb[0]&TCB_COMPL) { }; // block until transaction complete
  // set up GPINTENA to enable interrupt on all pins
  init_tcb[2] = MCP23017_GPINTENA;
  init_tcb[3] = 0xFF;
  queue_TWI((struct tcb*)init_tcb);
  while(init_tcb[0]&TCB_COMPL) { }; // block until transaction complete
  init_MCP23017_interrupt();
  QUEUE_QUICKREAD(0);
  #endif
}



#ifdef MCP23017_INT_PIN // if defined, it is 0 or 1
// Use MCP23017's interrupt output to trigger a GPIO read operation
// Using AVR's dedicated interrupts INT0/INT1 (not pin-change interrupt)
void init_MCP23017_interrupt() {
  // set INTx falling edge sensitive
  // TBD: may need to be level sensitive in case we get out of sync with MCP23017
  EICRA = (EICRA & ~( 3 << (2*MCP23017_INT_PIN) )) | ( 2 << (2*MCP23017_INT_PIN) );
  EIMSK |= 1 << (MCP23017_INT_PIN);
}  
ISR(MCP23017_INT_vect) 
{
  // schedule the predefined read operation at priority 0
  QUEUE_QUICKREAD(0);
}
#else
void init_MCP23017_interrupt() { }
#endif

