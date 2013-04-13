/*************************************************** 
  This is a limited-function library for the MCP23017 i2c port expander

  (c) 2012-13 Chuck Harrison for http:/opensourceecology.org
  BSD license
  
  Inspired by Adafruit_MCP23017.h whose copyright notice appears below
  > Adafruit invests time and resources providing this open source code, 
  > please support Adafruit and open-source hardware by purchasing 
  > products from Adafruit!
  > 
  > Written by Limor Fried/Ladyada for Adafruit Industries.  
  > BSD license, all text above must be included in any redistribution
 ****************************************************/

#ifndef _MCP23017_H_
#define _MCP23017_H_

  void MCP23017_begin(uint8_t addr);
  void MCP23017_pinMode(uint8_t p, uint8_t d);
  void MCP23017_digitalWrite(uint8_t p, uint8_t d);

  uint16_t MCP23017_readGPIOAB();

  uint8_t i2caddr;
  extern uint8_t GPIO_read_buf[2];

#define MCP23017_UNIT0 0x20

#define INPUT 1
#define DO_WAIT 1
#define DONT_WAIT 0

// registers
#define MCP23017_IODIRA 0x00
#define MCP23017_IPOLA 0x02
#define MCP23017_GPINTENA 0x04
#define MCP23017_DEFVALA 0x06
#define MCP23017_INTCONA 0x08
#define MCP23017_IOCONA 0x0A
#define MCP23017_GPPUA 0x0C
#define MCP23017_INTFA 0x0E
#define MCP23017_INTCAPA 0x10
#define MCP23017_GPIOA 0x12
#define MCP23017_OLATA 0x14


#define MCP23017_IODIRB 0x01
#define MCP23017_IPOLB 0x03
#define MCP23017_GPINTENB 0x05
#define MCP23017_DEFVALB 0x07
#define MCP23017_INTCONB 0x09
#define MCP23017_IOCONB 0x0B
#define MCP23017_GPPUB 0x0D
#define MCP23017_INTFB 0x0F
#define MCP23017_INTCAPB 0x11
#define MCP23017_GPIOB 0x13
#define MCP23017_OLATB 0x15

#endif
