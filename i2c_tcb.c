// interface for I2C devices using service queue and transaction control blocks (TCBs)

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


#include "i2c_tcb.h"

#include "config.h"
#ifdef MCP23017_PRESENT
#include "MCP23017.h"
#endif

volatile uint8_t twi_state;
uint8_t twi_slarw;

static volatile uint8_t twi_masterBufferIndex;
static uint8_t twi_masterBufferLength;
static uint8_t* twi_masterBufferPtr;

uint8_t twi_reg_bytes;
uint8_t twi_reg_cnt;
uint8_t* twi_regs;
uint8_t* twi_rmw_mask;

static volatile uint8_t twi_error;

// forward declarations
void start_transaction(struct tcb* tran);
int8_t twi_readGeneric(uint8_t dev_addr, uint8_t* reg_addr_spec, uint8_t* data, uint8_t length);
int8_t twi_writeGeneric(uint8_t dev_addr, uint8_t* reg_addr_spec, uint8_t* data, uint8_t length);

// this fields is fixed , so we give it a label:
//struct tcb {
//  uint8_t flags
//};
// remainder of block is an ordered sequence of data items; however items may be
//  absent from the sequence if their values are implied by the flag byte
// uint8_t device_addr // 7-bit convention
// uint8_t register_MSB
// uint8_t register_LSB
// uint8_t data_length_bytes
// uint8_t literal data(in place in tcb) - one byte read or write only
// uint8_t mask_byte - one byte write only
// uint8_t data_pointer_MSB
// uint8_t data_pointer_LSB


//#define WELL_KNOWN_TRANSACTIONS
// (supported if compile option set) well known transaction is defined by a byte array
//  uint8_t flags; // the predefined flag bits 7..3 applying to this transaction
//  uint8_t sequence_bits; // selects local vs tcb source of data for transaction execution
//     e.g. 0b10000000 would take device_addr from local table, rest from tcb
//  followed by local table
//  uint8_t device_addr
//  uint8_t register_MSB
//  [...]


// A quickread transaction is a one-byte I2C read intended to update a mirror register
// To queue a quickread, increment its 'pending' field
struct quickread {
  uint8_t pending;
  uint8_t data; // mirror register
  const uint8_t device; // I2C device address, 7-bit convention
  uint8_t reg_spec[2];
};
struct quickread quickreads[] = {
 {0, 0, MCP23017_UNIT0, {1, MCP23017_GPIOA}}
};
#define QR_END (struct quickread*)((uint8_t*)quickreads + sizeof(quickreads))

inline void queue_quickread(uint8_t i) {
  quickreads[i].pending++;
}
inline uint8_t quickread_data(uint8_t i) {
  return quickreads[i].data;
}

// All non-quickread transactions share a single round-robin fifo
uint8_t twi_fifo_write_pointer;
uint8_t twi_fifo_read_pointer;
#define TWI_FIFO_SIZE 8
struct tcb* twi_fifo[TWI_FIFO_SIZE];

#ifdef WELL_KNOWN_TRANSACTIONS
/******** well known transactions ***********/
// masked write to GPIOB, first MCP23017
uint8_t W_MCP0_PORTB[] = {
  TCB_WRT_MASKED | 1<<TCB_REG_SHIFT | TCB_INPLACE, // flags 
  0b11000000,  // seq: use preset for device addr, register addr
  MCP23017_UNIT0,
  MCP23017_OLATB,
};
// 

  
uint8_t* wellknown_table[8] = {
  W_MCP0_PORTB
};
#endif

struct tcb* tcb_in_progress;

// assumption: this is the only way I2C activity gets initiated (no mutexes needed)
// only call this if you know twi_state is TWI_READY.
void twi_process_queue() {
  // mark last transaction as complete
  if (tcb_in_progress != NULL) { 
    tcb_in_progress->flags &= ~TCB_COMPL;
    tcb_in_progress = NULL;
  }
  // quickread entries are highest priority before FIFO queues
  struct quickread * qr;
  for(qr = quickreads; qr!=QR_END; qr++) {
    if(qr->pending) {
      twi_readGeneric(qr->device, qr->reg_spec, &qr->data, 1);
      if(--qr->pending) { qr->pending=1; }
      return;
    }
  }
  // now process fifo
  struct tcb* transaction = NULL;
  if (twi_fifo_write_pointer == twi_fifo_read_pointer) {
    return; // fifo is empty
  }
  transaction = twi_fifo[twi_fifo_read_pointer];
  if((twi_fifo_read_pointer++)==TWI_FIFO_SIZE) {
    twi_fifo_read_pointer=0; // wrap pointer
  }
  start_transaction(transaction);
}

#ifdef WELL_KNOWN_TRANSACTIONS
struct {
  uint8_t sequence_map;
  uint8_t sequence_pointer;
  uint8_t* tcb_byte;
  uint8_t* presets_byte;
} tcb_feeder;
uint8_t* next_tcb_byte () {
  if(tcb_feeder.sequence_pointer&tcb_feeder.sequence_map) {
    return tcb_feeder.presets_byte++;
  }
  else {
    return tcb_feeder.tcb_byte++;
  }
  tcb_feeder.sequence_pointer >>= 1;
}
#else
uint8_t* tcb_byte;
inline uint8_t* next_tcb_byte () {
  return tcb_byte++;
}
#endif

uint8_t reg_addr_spec[3]; // addr width(0..2), MSB, LSB
void start_transaction(struct tcb* tran) {
  if (tran==NULL) { return; }
  uint8_t flags = tran->flags;
  if ((flags&TCB_COMPL)==TCB_IDLE) { return; } // cancelled
  #ifdef WELL_KNOWN_TRANSACTIONS
  tcb_feeder.sequence_map = 0;
  tcb_feeder.sequence_pointer = 0x80;
  tcb_feeder.tcb_byte = (uint8_t*)(tran + 1); // start of tcb variant section
  if ((flags & TCB_RWW)==TCB_WK) {  //read/write/wellknown flags
    // well known transaction
    tcb_feeder.presets_byte = wellknown_table[(flags & TCB_WKID)>>TCB_WK_SHIFT];
    flags = *tcb_feeder.presets_byte++;
    tcb_feeder.sequence_map = *tcb_feeder.presets_byte++;
  }
  #else
  tcb_byte = (uint8_t*)(tran + 1); // start of tcb variant section
  #endif
  // get device and register addresses
  uint8_t dev_addr = *next_tcb_byte();
  uint8_t ct; // counter for register address bytes
  reg_addr_spec[0] = ct = (flags&TCB_REG)>>TCB_REG_SHIFT; // number of register address bytes
  while(ct--) {
    reg_addr_spec[ct] = *next_tcb_byte();
  }
  // get length & data pointer
  uint8_t length;
  uint8_t* data;
  if(flags&TCB_INPLACE) { // in-place
    length = 1;
    data = next_tcb_byte();
    // if (masked_write) {next_tcb_byte();} would need to skip mask, but this is last field
  } else {
    length = *next_tcb_byte();
    uint16_t data_t = (*next_tcb_byte() << 8);
    data_t |= *next_tcb_byte();
    data = (uint8_t*)data_t;
  }
  // start the transaction
  if((flags&TCB_RWW)==TCB_READ) { // read
    twi_readGeneric(dev_addr, reg_addr_spec, data, length);
  } else { // write
    if ((flags&TCB_RWW)==TCB_WRT_MASKED) { // write masked
      length = -length;
    }
    twi_writeGeneric(dev_addr, reg_addr_spec, data, length);
  }
  // set completion status
  uint8_t newflags = tran->flags & ~TCB_COMPL;
  newflags |= TCB_SERVICING;
  tran->flags = newflags;
  tcb_in_progress = tran; // save tcb pointer for final completion report
}

int8_t queue_TWI(struct tcb* control_block) {
  uint8_t wrt = twi_fifo_write_pointer + 1;
  if(wrt==TWI_FIFO_SIZE) {
    wrt=0;
  }
  if(wrt == twi_fifo_read_pointer) {
    return -1; // fail, queue full
  }
  twi_fifo_write_pointer = wrt;
  twi_fifo[wrt]=control_block;
  uint8_t newflags = control_block->flags & ~TCB_COMPL;
  newflags |= TCB_QUEUED;
  control_block->flags = newflags;
  return 0; // success
}

/*** TBD:
int8_t setbit_TWI(struct tcb* control_block, uint8_t bitmask, uint8_t sync_mode) {
  // sync_modes  queue: wait on availability or return regardless
  //             completion: wait on service start, wait on completion,
  //                         or return regardless
  //             timeout value if wait
  return 0; //dummy
}
int8_t clearbit_TWI(struct tcb* control_block, uint8_t bitmask, uint8_t sync_mode) {
//TBD
  return 0; //dummy
} 
***/

int8_t twi_readGeneric(uint8_t dev_addr, uint8_t* reg_addr_spec, uint8_t* data, uint8_t length)
{
  if(TWI_READY != twi_state){
    return -1; // busy
  }
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
  twi_reg_bytes = reg_addr_spec[0];
  twi_reg_cnt = 0;
  if(twi_reg_bytes) {
    // initialize register spec
    twi_regs = reg_addr_spec+1;
    twi_state = TWI_MTRX;
    twi_slarw = TW_WRITE;
  } else { // no register, simple read
    twi_state = TWI_MRX;
    twi_slarw = TW_READ;
  }
  twi_slarw |= dev_addr << 1;
  // send start condition
  TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWEA) | _BV(TWINT) | _BV(TWSTA);
  return 0;	// success
}
  
int8_t twi_writeGeneric(uint8_t dev_addr, uint8_t* reg_addr_spec, uint8_t* data, uint8_t length) {
  if(TWI_READY != twi_state){
    return -1; // busy
  }
  // negative length means do masked write; mask data immediately follows source data
  if (length<0) {
    twi_state = TWI_M_RMW;
    length = -length; 
    twi_rmw_mask = data + length;
  } else {
    twi_state = TWI_MTX;
  }    
  twi_slarw = TW_WRITE; // we change this if doing masked write with no register addr
  // reset error state (0xFF.. no error occured)
  twi_error = 0xFF;
  twi_reg_bytes = reg_addr_spec[0];
  twi_regs = reg_addr_spec+1;
  twi_reg_cnt = 0;
  twi_masterBufferPtr=data; // modified data is built in caller's data buffer during read
  twi_masterBufferIndex = 0;
  twi_masterBufferLength = length;
  if (twi_state==TWI_M_RMW) {
    twi_masterBufferLength -= 1; // adjust for read operation's NAK timing
    if (twi_reg_bytes==0) { 
      twi_slarw = TW_READ; 
    }
  } 
  // build sla+w, slave device address + w bit
  twi_slarw |= dev_addr << 1;
  
  // send start condition
  TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWEA) | _BV(TWINT) | _BV(TWSTA);
  return 0;	// success
}


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
  twi_fifo_init();
  
  // enable twi module, acks, and twi interrupt
  TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWEA);
}



void twi_fifo_init() {
  // clear all pending quickreads
  struct quickread* qr;
  for(qr = quickreads; qr!=QR_END; qr++) {
    qr->pending=0;
  }
  // set fifo empty
  twi_fifo_write_pointer=twi_fifo_read_pointer=0;
  // set last transaction pointer NULL
  tcb_in_progress = NULL;
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
  twi_process_queue();
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
    case TW_MT_DATA_ACK: // slave receiver acked data
      if(twi_reg_cnt < twi_reg_bytes) {
        TWDR = twi_regs[twi_reg_cnt++];
        twi_reply(1);
        break;
      }        
      if(twi_state==TWI_MTRX || twi_state==TWI_M_RMW) {
        // finished writing register address, now do a repeated start and read
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
      }else {
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
      if(twi_state != TWI_M_RMW) {
        // put byte into buffer
        *(twi_masterBufferPtr+twi_masterBufferIndex++) = TWDR;
      } else { // using tmi_rmw_data as buffer
        twi_masterBufferPtr[twi_masterBufferIndex] =
          (twi_masterBufferPtr[twi_masterBufferIndex] & twi_rmw_mask[twi_masterBufferIndex])
          | (TWDR & ~twi_rmw_mask[twi_masterBufferIndex]);
        twi_masterBufferIndex++;
      }
    case TW_MR_SLA_ACK:  // address sent, ack received
      // ack if more bytes are expected, otherwise nack
      if(twi_masterBufferIndex < twi_masterBufferLength){
        twi_reply(1); // ack
      }else{
        twi_reply(0); // nack; next byte recvd will be last
      }
      break;
    case TW_MR_DATA_NACK: // data received, nack sent
      if(twi_state != TWI_M_RMW) {
        // put final byte into buffer
        *(twi_masterBufferPtr+twi_masterBufferIndex++) = TWDR;
      } else {
        twi_masterBufferPtr[twi_masterBufferIndex] =
          (twi_masterBufferPtr[twi_masterBufferIndex] & twi_rmw_mask[twi_masterBufferIndex])
          | (TWDR & ~twi_rmw_mask[twi_masterBufferIndex]);
        // finished read/modify phase of read-modify-write operation;
        // reset pointers
        twi_masterBufferIndex = 0;
        twi_masterBufferLength++; // read mode used decremented length
        twi_reg_cnt = 0;
        twi_state = TWI_MTX;
        twi_slarw &= ~TW_READ; // reset to WRITE mode & send repeated start 
        // timing problem with repeated start? send stop/start
        // send stop condition
        TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWEA) | _BV(TWINT) | _BV(TWSTO);
        while(TWCR & _BV(TWSTO)){
          continue;
        }
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

