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


////////////////////////////////////////////////////////////////////////////////
void init_MCP23017_interrupt(); // forward declaration


// TCB definition for startup operations (write 1 byte)
uint8_t init_tcb[4];

void MCP23017_begin(uint8_t i2caddr) {
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
  #endif
}

/***

void MCP23017_pinMode(uint8_t p, uint8_t d) {
  uint8_t localbuf[2];

  // only 16 bits!
  if (p > 15)
    return;

  if (p < 8)
    localbuf[0] = MCP23017_IODIRA;
  else {
    localbuf[0] = MCP23017_IODIRB;
    p -= 8;
  }

  // read the current IODIR
  if(twi_writeTo(i2caddr, localbuf, 1, DO_WAIT) != 0) return;
  if(twi_readFrom(i2caddr, &localbuf[1], 1) != 1) return;
  // set the pin and direction
  if (d == INPUT) {
    localbuf[1] |= 1 << p; 
  } else {
    localbuf[1] &= ~(1 << p);
  }

  // write the new IODIR
  twi_writeTo(i2caddr, localbuf, 2, DO_WAIT);
}

uint16_t MCP23017_readGPIOAB() {
  uint16_t ba = 0;
  
  uint8_t localbuf[3] = {MCP23017_GPIOA, 0, 0};

  // read the current GPIO output latches
  if(twi_writeTo(i2caddr, localbuf, 1, DO_WAIT) != 0) return 0;
  if(twi_readFrom(i2caddr, &localbuf[1], 2) != 2) return 0;

  ba = localbuf[2];
  ba <<= 8;
  ba |= localbuf[1];

  return ba;
}

void MCP23017_digitalWrite(uint8_t p, uint8_t d) {
  uint8_t localbuf[2];
  uint8_t olataddr;

  // only 16 bits!
  if (p > 15)
    return;

  if (p < 8) {
    olataddr = MCP23017_OLATA;
    localbuf[0] = MCP23017_GPIOA;
  } else {
    olataddr = MCP23017_OLATB;
    localbuf[0] = MCP23017_GPIOB;
    p -= 8;
  }

  // read the current GPIO output latches
  uint8_t status = twi_writeTo(i2caddr, &olataddr, 1, DO_WAIT);
  if(twi_readFrom(i2caddr, &localbuf[1], 1) != 1) return;
  // set the pin 
  if (d != 0) {
    localbuf[1] |= 1 << p; 
  } else {
    localbuf[1] &= ~(1 << p);
  }
  // write the new GPIO
  status = twi_writeTo(i2caddr, localbuf, 2, DO_WAIT);
}
***/


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
  // schedule a read operation at priority 0
  queue_quickread(0);
}
#else
void init_MCP23017_interrupt() { }
#endif

