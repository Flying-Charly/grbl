/*
  twi.c - TWI/I2C library for Wiring & Arduino
  Copyright (c) 2006 Nicholas Zambetti.  All right reserved.

  transaction based extensions
  (c) 2013 Chuck Harrison for http://opensourceecology.org, same license
  (also stripped out slave code to save space in this application)
  ****WARNING****  work in progress  ****WARNING****
  The transaction-based read operations have had very limited testing 
  and the transaction-based write operations have had none.
  
  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/


#include <math.h>
#include <stdlib.h>
#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <compat/twi.h>

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif

#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

#define TWI_BUFFER_LENGTH 8
#include "twi.h"

static volatile uint8_t twi_state;
static uint8_t twi_slarw;
static uint8_t twi_reg;
static uint8_t twi_rmw_data;
static uint8_t twi_rmw_mask;

static uint8_t twi_masterBuffer[TWI_BUFFER_LENGTH];
static volatile uint8_t twi_masterBufferIndex;
static uint8_t twi_masterBufferLength;
static uint8_t* twi_masterBufferPtr;

static volatile uint8_t twi_error;


/* 
 * Function twi_init
 * Desc     readys twi pins and sets twi bitrate
 * Input    none
 * Output   none
 */
void twi_init(void)
{
  // initialize state
  twi_state = TWI_READY;
  
  #if defined(__AVR_ATmega168__) || defined(__AVR_ATmega8__) || defined(__AVR_ATmega328P__)
    // activate internal pull-ups for twi
    // as per note from atmega8 manual pg167
    sbi(PORTC, 4);
    sbi(PORTC, 5);
  #else
    // activate internal pull-ups for twi
    // as per note from atmega128 manual pg204
    sbi(PORTD, 0);
    sbi(PORTD, 1);
  #endif

  // initialize twi prescaler and bit rate
  cbi(TWSR, TWPS0);
  cbi(TWSR, TWPS1);
  TWBR = ((F_CPU / TWI_FREQ) - 16) / 2;

  /* twi bit rate formula from atmega128 manual pg 204
  SCL Frequency = CPU Clock Frequency / (16 + (2 * TWBR))
  note: TWBR should be 10 or higher for master mode
  It is 72 for a 16mhz Wiring board with 100kHz TWI */

  // initialize transaction queue
  twi_queue_init();
  
  // enable twi module, acks, and twi interrupt
  TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWEA);
}


/* 
 * Function twi_readFrom
 * Desc     attempts to become twi bus master and read a
 *          series of bytes from a device on the bus
 * Input    address: 7bit i2c device address
 *          data: pointer to byte array
 *          length: number of bytes to read into array
 * Output   number of bytes read
 */
uint8_t twi_readFrom(uint8_t address, uint8_t* data, uint8_t length)
{
  uint8_t i;

  // ensure data will fit into buffer
  if(TWI_BUFFER_LENGTH < length){
    return 0;
  }
  // wait until twi is ready, then initiate read
  while(twi_nonBlockingReadFrom(address, twi_masterBuffer, length)) {
    continue;
  }
  // wait for read operation to complete
  while(TWI_MRX == twi_state){
    continue;
  }

  if (twi_masterBufferIndex < length)
    length = twi_masterBufferIndex;

  // copy twi buffer to data
  for(i = 0; i < length; ++i){
    data[i] = twi_masterBuffer[i];
  }
	
  return length;
}
// for non-blocking read, the interrupt service fills user buffer in background
// returns immediately with status -1 = busy, or 0 = read operation initiated. 
int8_t twi_nonBlockingReadFrom(uint8_t address, uint8_t* data, uint8_t length)
{
  // if twi is ready, become master receiver
  // use interrupt guard around twi_state test-and-set
  uint8_t sreg_save = SREG; 
  cli(); 
  if(TWI_READY != twi_state){
    SREG = sreg_save; 
    return -1; // busy
  }
  twi_state = TWI_MRX;
  SREG = sreg_save; 
  
  // reset error state (0xFF.. no error occured)
  twi_error = 0xFF;

  // initialize buffer iteration vars
  twi_masterBufferPtr=data;
  twi_masterBufferIndex = 0;
  twi_masterBufferLength = length-1;  // This is not intuitive, read on...
  // On receive, the previously configured ACK/NACK setting is transmitted in
  // response to the received byte before the interrupt is signalled. 
  // Therefor we must actually set NACK when the _next_ to last byte is
  // received, causing that NACK to be sent in response to receiving the last
  // expected byte of data.

  // build sla+w, slave device address + w bit
  twi_slarw = TW_READ;
  twi_slarw |= address << 1;

  // send start condition
  TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWEA) | _BV(TWINT) | _BV(TWSTA);

  return 0;
}

// for non-blocking register read, we pass a one-byte register address. The
// interrupt service sends the register address, then fills user buffer in background
// returns immediately with status -1 = busy, or 0 = read operation initiated.
// For a device like MCP23017, this makes a unitary transaction which can be
// interspersed with other transactions to the same device (e.g. from other threads)
int8_t twi_nonBlockingReadRegisterFrom(uint8_t address, uint8_t reg, uint8_t* data, uint8_t length)
{
  // if twi is ready, become master transmitter
  // use interrupt guard around twi_state test-and-set
  uint8_t sreg_save = SREG; 
  cli(); 
  if(TWI_READY != twi_state){
    SREG = sreg_save; 
    return -1; // busy
  }
  twi_state = TWI_MTRX;
  SREG = sreg_save; 
  
  // reset error state (0xFF.. no error occured)
  twi_error = 0xFF;
  // initialize register pointer
  twi_reg = reg;
  // initialize buffer iteration vars
  twi_masterBufferPtr=data;
  twi_masterBufferIndex = 0;
  twi_masterBufferLength = length-1;  // This is not intuitive, read on...
  // On receive, the previously configured ACK/NACK setting is transmitted in
  // response to the received byte before the interrupt is signalled. 
  // Therefor we must actually set NACK when the _next_ to last byte is
  // received, causing that NACK to be sent in response to receiving the last
  // expected byte of data.

  // build sla+w, slave device address + w bit
  twi_slarw = TW_WRITE;
  twi_slarw |= address << 1;
  
  // send start condition
  TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWEA) | _BV(TWINT) | _BV(TWSTA);

 
  if (twi_error == 0xFF)
    return 0;	// success
  else
    return -2;	// other twi error
}


/* 
 * Function twi_writeTo
 * Desc     attempts to become twi bus master and write a
 *          series of bytes to a device on the bus
 * Input    address: 7bit i2c device address
 *          data: pointer to byte array
 *          length: number of bytes in array
 *          wait: boolean indicating to wait for write or not
 * Output   0 .. success
 *          1 .. length to long for buffer
 *          2 .. address send, NACK received
 *          3 .. data send, NACK received
 *          4 .. other twi error (lost bus arbitration, bus error, ..)
 */
uint8_t twi_writeTo(uint8_t address, uint8_t* data, uint8_t length, uint8_t wait)
{
  uint8_t i;

  // ensure data will fit into buffer
  if(TWI_BUFFER_LENGTH < length){
    return 1;
  }

  // wait until twi is ready, become master transmitter
  // use interrupt guard around twi_state test-and-set
  while(1) {
    if(TWI_READY != twi_state) {
      continue;
    }
    uint8_t sreg_save = SREG; 
    cli(); 
    if(TWI_READY == twi_state) {
      twi_state = TWI_MTX;
      SREG = sreg_save; 
      break;
    }
    SREG = sreg_save; 
  }
  
  // reset error state (0xFF.. no error occured)
  twi_error = 0xFF;

  // initialize buffer iteration vars
  twi_masterBufferPtr = twi_masterBuffer;
  twi_masterBufferIndex = 0;
  twi_masterBufferLength = length;
  
  // copy data to twi buffer
  for(i = 0; i < length; ++i){
    twi_masterBuffer[i] = data[i];
  }
  
  // build sla+w, slave device address + w bit
  twi_slarw = TW_WRITE;
  twi_slarw |= address << 1;
  
  // send start condition
  TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWEA) | _BV(TWINT) | _BV(TWSTA);

  // wait for write operation to complete
  while(wait && (TWI_MTX == twi_state)){
    continue;
  }
  
  if (twi_error == 0xFF)
    return 0;	// success
  else if (twi_error == TW_MT_SLA_NACK)
    return 2;	// error: address send, nack received
  else if (twi_error == TW_MT_DATA_NACK)
    return 3;	// error: data send, nack received
  else
    return 4;	// other twi error
}

// for register read-modify-write, we pass a one-byte register address. 
// returns immediately with status -1 = busy, or 0 = read operation initiated.
// For a device like MCP23017, this makes a unitary transaction which can be
// interspersed with other transactions to the same device (e.g. from other threads)
int8_t twi_writeRegisterMaskedOneByte(uint8_t address, uint8_t reg, uint8_t data, uint8_t mask)
{
  // if twi is ready, become master transmitter
  // use interrupt guard around twi_state test-and-set
  uint8_t sreg_save = SREG; 
  cli(); 
  if(TWI_READY != twi_state){
    SREG = sreg_save; 
    return -1; // busy
  }
  twi_state = TWI_M_RMW;
  SREG = sreg_save; 

  twi_rmw_data = data;
  twi_rmw_mask = mask;  
  // reset error state (0xFF.. no error occured)
  twi_error = 0xFF;
  // initialize register pointer
  twi_reg = reg;
  // initialize buffer iteration vars
  twi_masterBufferPtr=twi_masterBuffer;
  twi_masterBufferIndex = 0;
  twi_masterBufferLength = 0; 

  // build sla+w, slave device address + w bit
  twi_slarw = TW_WRITE;
  twi_slarw |= address << 1;
  
  // send start condition
  TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWEA) | _BV(TWINT) | _BV(TWSTA);

 
  if (twi_error == 0xFF)
    return 0;	// success
  else
    return -2;	// other twi error
}


/************** transaction queue management ***************/
/*
*  max is just one read & one write transaction queued at each priority level
*/

twi_transaction_read* twi_read_queue[TWI_RD_TRANS_QUEUE_SIZE];
twi_transaction_write_one_masked* twi_wr1_queue[TWI_WR1_TRANS_QUEUE_SIZE];

void twi_queue_init() {
  uint8_t i;
  for(i=0; i<TWI_RD_TRANS_QUEUE_SIZE; i++) {
    twi_read_queue[i] = NULL;
  }
  for(i=0; i<TWI_WR1_TRANS_QUEUE_SIZE; i++) {
    twi_wr1_queue[i] = NULL;
  }
}

// check for pending transactions and initiate highest priority one
//  TBD: remember last transaction executed in order to support completion status
void twi_check_queues () {
  uint8_t p;
  for(p=0; p<TWI_RD_TRANS_QUEUE_SIZE || p<TWI_WR1_TRANS_QUEUE_SIZE; p++) {
    twi_transaction_read** tr = twi_read_queue+p;
    if (p<TWI_RD_TRANS_QUEUE_SIZE && *tr!= NULL) {
      if(twi_nonBlockingReadRegisterFrom((*tr)->address, (*tr)->reg, (*tr)->data, (*tr)->length) == -1) {
        return; // TWI is busy
      } else {
        *tr = NULL;
        // TBD: use completion field to alert originator that his transaction block has been unlinked 
        return;
      }
    }
    twi_transaction_write_one_masked** tw1 = twi_wr1_queue+p;
    if (p<TWI_WR1_TRANS_QUEUE_SIZE && *tw1!=NULL) {
      if(twi_writeRegisterMaskedOneByte((*tw1)->address, (*tw1)->reg, (*tw1)->data, (*tw1)->mask) == -1) {
        return; // TWI is busy
      } else {
        *tw1 = NULL;
        // TBD: use completion field to alert originator that his transaction block has been unlinked
      return;
      }
    }
  }
  return; // nothing in queue
}
// put a read transaction in the queue (priority=0 is highest)
int8_t twi_queue_read_transaction(twi_transaction_read* trans, uint8_t priority) {
  if (priority>=TWI_RD_TRANS_QUEUE_SIZE) {
    return -2; // invalid priority level
  }
  uint8_t rv = 0;
  uint8_t sreg_save = SREG; 
  cli(); 
  if(twi_read_queue[priority] != NULL) {
    rv = -1; // busy
  } else {
    twi_read_queue[priority] = trans;
    if(twi_state==TWI_READY) {
      twi_check_queues();
    }
  }
  SREG = sreg_save;
  return rv;
}
// put a write transaction in the queue (priority=0 is highest)
int8_t twi_queue_write_one_masked_transaction(twi_transaction_write_one_masked* trans, uint8_t priority) {
  if (priority>=TWI_WR1_TRANS_QUEUE_SIZE) {
    return -2; // invalid priority level
  }
  uint8_t rv = 0;
  uint8_t sreg_save = SREG; 
  cli(); 
  if(twi_wr1_queue[priority] != NULL) {
    rv = -1; // busy
  } else {
    twi_wr1_queue[priority] = trans;
    if(twi_state==TWI_READY) {
      twi_check_queues();
    }
  }
  SREG = sreg_save;
  return rv;
}


  
/************** low level interrupt service routines   ****************/
/* 
 * Function twi_reply
 * Desc     sends byte or readys receive line
 * Input    ack: byte indicating to ack or to nack
 * Output   none
 */
void twi_reply(uint8_t ack)
{
  // transmit master read ready signal, with or without ack
  if(ack){
    TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWINT) | _BV(TWEA);
  }else{
	  TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWINT);
  }
}

/* 
 * Function twi_stop
 * Desc     relinquishes bus master status
 * Input    none
 * Output   none
 */
void twi_stop(void)
{
  // send stop condition
  TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWEA) | _BV(TWINT) | _BV(TWSTO);

  // wait for stop condition to be exectued on bus
  // TWINT is not set after a stop condition!
  while(TWCR & _BV(TWSTO)){
    continue;
  }

  // update twi state
  twi_state = TWI_READY;
  // proceed with any queued transactions
  twi_check_queues();
}

/* 
 * Function twi_releaseBus
 * Desc     releases bus control
 * Input    none
 * Output   none
 */
void twi_releaseBus(void)
{
  // release bus
  TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWEA) | _BV(TWINT);

  // update twi state
  twi_state = TWI_READY;
}

SIGNAL(TWI_vect)
{
  switch(TW_STATUS){
    // All Master
    case TW_START:     // sent start condition
    case TW_REP_START: // sent repeated start condition
      // copy device address and r/w bit to output register and ack
      TWDR = twi_slarw;
      twi_reply(1); // ack
      break;

    // Master Transmitter
    case TW_MT_SLA_ACK:  // slave receiver acked address
      if(twi_state==TWI_MTRX || twi_state==TWI_M_RMW) {
        TWDR = twi_reg; // multibyte: TWDR = *twi_reg_ptr++;
        twi_reply(1); // ack
        break;
      }
      // else fall thru...
    case TW_MT_DATA_ACK: // slave receiver acked data
      if(twi_state==TWI_MTRX || twi_state==TWI_M_RMW) {
        // multibyte: if(--reg_addr_bytes_remain) {
        //   TWDR = *twi_reg_ptr++;
        //   twi_reply(1);
        //   break;
        // }        
        // sent register addr; switch to RX mode and issue repeated start
        // timing problem with repeated start? send stop/start
        // send stop condition
        TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWEA) | _BV(TWINT) | _BV(TWSTO);
        while(TWCR & _BV(TWSTO)){
          continue;
        }
        twi_slarw |= TW_READ;
        // now start
        TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWEA) | _BV(TWINT) | _BV(TWSTA);
        break;
      }
      // if there is data to send, send it, otherwise stop 
      if(twi_masterBufferIndex < twi_masterBufferLength){
        // copy data to output register and ack
        TWDR = *(twi_masterBufferPtr+twi_masterBufferIndex++);
        twi_reply(1); // ack
      }else{
        twi_stop();
      }
      break;
    case TW_MT_SLA_NACK:  // address sent, nack received
      twi_error = TW_MT_SLA_NACK;
      twi_stop();
      break;
    case TW_MT_DATA_NACK: // data sent, nack received
      twi_error = TW_MT_DATA_NACK;
      twi_stop();
      break;
    case TW_MT_ARB_LOST: // lost bus arbitration
      twi_error = TW_MT_ARB_LOST;
      twi_releaseBus();
      break;

    // Master Receiver
    case TW_MR_DATA_ACK: // data received, ack sent
      // put byte into buffer
      *(twi_masterBufferPtr+twi_masterBufferIndex++) = TWDR;
    case TW_MR_SLA_ACK:  // address sent, ack received
      // ack if more bytes are expected, otherwise nack
      if(twi_masterBufferIndex < twi_masterBufferLength){
        twi_reply(1); // ack
      }else{
        twi_reply(0); // nack; next byte recvd will be last
      }
      break;
    case TW_MR_DATA_NACK: // data received, nack sent
      // put final byte into buffer
      *(twi_masterBufferPtr+twi_masterBufferIndex++) = TWDR;
      if(twi_state==TWI_M_RMW) {
        // finished read phase of read-modify-write operation;
        // move modified data into 2nd buffer byte & put register value in 1st
        *(twi_masterBufferPtr+1) = (*twi_masterBufferPtr & ~twi_rmw_mask)
                                   | (twi_rmw_data & twi_rmw_mask);
        *twi_masterBufferPtr = twi_reg;
        twi_masterBufferIndex = 0;
        twi_masterBufferLength = 2;
        twi_state = TWI_MTX;
        twi_slarw &= ~TW_READ; // reset to WRITE mode
        // repeated start should work properly here because NAK left SDA high ?
        TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWEA) | _BV(TWINT) | _BV(TWSTA);
        break;
      }
    case TW_MR_SLA_NACK: // address sent, nack received
      twi_stop();
      break;
    // TW_MR_ARB_LOST handled by TW_MT_ARB_LOST case

    // Slave Receiver
    case TW_SR_SLA_ACK:   // addressed, returned ack
    case TW_SR_GCALL_ACK: // addressed generally, returned ack
    case TW_SR_ARB_LOST_SLA_ACK:   // lost arbitration, returned ack
    case TW_SR_ARB_LOST_GCALL_ACK: // lost arbitration, returned ack
    case TW_SR_DATA_ACK:       // data received, returned ack
    case TW_SR_GCALL_DATA_ACK: // data received generally, returned ack
    case TW_SR_STOP: // stop or repeated start condition received
    case TW_SR_DATA_NACK:       // data received, returned nack
    case TW_SR_GCALL_DATA_NACK: // data received generally, returned nack
    case TW_ST_SLA_ACK:          // addressed, returned ack
    case TW_ST_ARB_LOST_SLA_ACK: // arbitration lost, returned ack
    case TW_ST_DATA_ACK: // byte sent, ack returned
    case TW_ST_DATA_NACK: // received nack, we are done 
    case TW_ST_LAST_DATA: // received ack, but we are done already!

    // All
    case TW_NO_INFO:   // no state information
      break;
    case TW_BUS_ERROR: // bus error, illegal stop/start
      twi_error = TW_BUS_ERROR;
      twi_stop();
      break;
  }
}

