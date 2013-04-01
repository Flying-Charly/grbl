// interface for I2C devices



// transaction control block
// polymorphic, primarily to conserve memory
// flag byte controls interpretation
#define TCB_RW_SHIFT 6
#define TCB_RWW (3 << TCB_RW_SHIFT)
#define TCB_WK (0 << TCB_RW_SHIFT)
#define TCB_READ (1 << TCB_RW_SHIFT)
#define TCB_WRT (2 << TCB_RW_SHIFT)
#define TCB_WRT_MASKED (3 << TCB_RW_SHIFT)
//bit 7..6 - read/write/wellknown
//           00 - well known transaction
//           01 - read
//           10 - write
//           11 - write masked
#define TCB_WK_SHIFT 3
#define TCB_WKID (7<<TCB_WK_SHIFT)           
//bits 5..3 - transaction identifier (if well known)
#define TCB_REG_SHIFT 4
#define TCB_REG (3<<TCB_REG_SHIFT)
//bits 5..4 - register addressing (if not well known)
//            00 - no registers
//            01 - 1 byte address
//            10 - 2 byte address
//            11 - reserved
#define TCB_INPLACE (0x80)
//bit 3 - single-byte, in-place data (if not well known)
//bit 2 -
#define TCB_COMPL (0x03)
#define TCB_IDLE 0
#define TCB_SERVICING 1
#define TCB_QUEUED 2
//bits 1..0 - completion status
//            00 - idle = transaction completed
//            01 - being serviced, tcb may be reused
//            10 - on queue
//            11 - reserved

// this fields is fixed , so we give it a label:
struct tcb {
  uint8_t flags
}
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


// well known transaction is defined by a byte array
//  uint8_t flags; // the predefined flag bits 7..3 applying to this transaction
//  uint8_t sequence_bits; // selects local vs tcb source of data for transaction execution
//     e.g. 0b10000000 would take device_addr from local table, rest from tcb
//  followed by local table
//  uint8_t device_addr
//  uint8_t register_MSB
//  [...]


// A quickread transaction is a one-byte I2C read intended to update a mirror register
// To queue a quickread, increment its 'pending' field.
struct quickread {
  uint8_t pending;
  uint8_t data; // mirror register
  const uint8_t device; // I2C device address, 7-bit convention
  const uint8_t[2] reg_spec;
};
struct quickread quickreads[] = {
 {0, 0, MCP23017_UNIT0, {1, MCP23017_GPIOA}};
}
#define QR_END (quickread*)((uint8_t*)quickreads + sizeof(quickreads))

// All non-quickread transactions share a single round-robin fifo
uint8_t twi_fifo_write_pointer;
uint8_t twi_fifo_read_pointer;
#define TWI_FIFO_SIZE 8
tcb* twi_fifo[TWI_FIFO_SIZE];


/******** well known transactions ***********/
// masked write to GPIOB, first MCP23017
uint8_t W_MCP0_PORTB[] = {
  TCB_WRT_MASKED | 1<<TCB_REG_SHIFT | TCB_INPLACE, // flags 
  0b11000000,  // seq: use preset for device addr, register addr
  MCP23017_UNIT0,
  MCP23017_OLATB,
}
// 

  
uint8_t* wellknown_table[8] = {
  W_MCP0_PORTB
}

// assumption: this is the only way I2C activity gets initiated (no mutexes needed)
// only call this if you know twi_state is TWI_READY.
void twi_process_queue() {
  // mark last transaction as complete
  if (tcb_in_progress != NULL) { 
    tcb_in_progress->flags &= ~TCB_COMPL_MASK;
    tcb_in_progress = NULL;
  }
  // quickread entries are highest priority before FIFO queues
  for(quickread* qr = quickreads; qr!=QR_END; qr++) {
    if(qr.pending) {
      twi_readGeneric(qr.device, qr.reg_spec, &qr.data, 1);
      if(--qr.pending) { qr.pending=1; }
      return;
    }
  }
  // now process fifo
  tcb* transaction = NULL;
  if (twi_fifo_write_pointer == twi_fifo_read_pointer) { continue; }
    transaction = twi_fifo[twi_fifo_read_pointer];
    if((twi_fifo_read_pointer++)==TWI_FIFO_SIZE) {twi_fifo_read_pointer=0; }
  }
  start_transaction(transaction);
}

struct {
  uint8_t sequence_map;
  uint8_t sequence_pointer;
  uint8_t* tcb_byte;
  uint8_t* presets_byte;
} tcb_feeder;
uint8_t* next_tcb_byte () {
  if(tcb_feeder.sequence_ptr&tcb_feeder.sequence_map) {
    return presets_byte++;
  }
  else {
    return tcb_byte++
  sequence_ptr >>= 1;
}
uint8_t reg_addr_spec[3]; // addr width(0..2), MSB, LSB
tcb* tcb_in_progress;
void start_transaction(tcb* tran) {
  if (tran==NULL) { return; }
  uint8_t flags = tran->flags;
  if ((flags&TCB_COMPL)==TCB_IDLE) { return; } // cancelled
  tcb_feeder.sequence_map = 0;
  tcb_feeder.sequence_ptr = 0x80;
  tcb_feeder.tcb_byte = (uint8_t*)(tran + 1); // start of tcb variant section
  uint8_t* presets;
  if ((flags & TCB_RWW)==TCB_WK) {  //read/write/wellknown flags
    // well known transaction
    tcb_feeder.presets_byte = wellknown_table + (flags & TCB_WKID)>>TCB_WK_SHIFT;
    flags = *tcb_feeder.presets_byte++;
    sequence_map = *tcb_feeder.presets_byte++;
  }
  // get device and register addresses
  dev_addr = *next_tcb_byte();
  uint8_t ct; // counter for register address bytes
  reg_addr_spec[0] = ct = flags&TCB_REG)>>TCB_REG_SHIFT; // number of register address bytes
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
    data = *next_tcb_byte() << 8;
    data |= *next_tcb_byte();
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

int8_t queue_TWI( tcb* control_block) {
  uint8_t* prio = control_block+1;
  uint8_t wrt = twi_fifo_write_pointer + 1;
  if(wrt==TWI_FIFO_SIZE) (wrt==0);
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

int8_t setbit_TWI(tcb* control_block, uint8_t bitmask, uint8_t sync_mode) {
  // sync_modes  queue: wait on availability or return regardless
  //             completion: wait on service start, wait on completion,
  //                         or return regardless
  //             timeout value if wait
}
int8_t clearbit_TWI(tcb* control_block, uint8_t bitmask, uint8_t sync_mode) {
} 

int8_t twi_readGeneric(uint8_t dev_addr, uint8_t* reg_addr_spec, uint8_t* data, uint8_t length) {
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
  if(twi_reg_bytes) {
    // initialize register spec
    twi_reg = reg_addr_spec[1];
    twi_reg2 = reg_addr_spec[2];
    twi_state = TWI_MTRX;
    twi_slarw = TW_WRITE;
  } else { // no register, simple read
    twi_state = TWI_MRX;
    twi_slarw = TW_READ;
  }
  twi_slarw |= address << 1;
  // send start condition
  TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWEA) | _BV(TWINT) | _BV(TWSTA);
  return 0;	// success
}
  
int8_t twi_writeGeneric(uint8_t dev_addr, uint8_t* reg_addr_spec, uint8_t* data, unit8_t length) {
  if(TWI_READY != twi_state){
    return -1; // busy
  }
  // negative length means do masked write; mask data immediately follows source data
  if (length<0) {
    twi_state = TWI_M_RMW;
    length = -length; 
  else {
    twi_state = TWI_MTX;
  }    
  // reset error state (0xFF.. no error occured)
  twi_error = 0xFF;
  twi_slarw = TW_WRITE; // we change this only if doing a masked write with no register address
  twi_reg_bytes = reg_addr_spec[0];
  twi_reg_ptr = reg_addr_spec+1;
  if(twi_reg_bytes && twi_state==TWI_MTX) { // non_masked write; with register address
    twi_masterBufferPtr=twi_reg_ptr;
    twi_masterBufferIndex = 0;
    twi_masterBufferLength = twi_reg_bytes;
    source_data = data;
    source_length = length;
  } else { 
    twi_masterBufferPtr=data; // modified data is built in caller's data buffer during read
    twi_masterBufferIndex = 0;
    twi_masterBufferLength = length;
    if (twi_state==TWI_M_RMW) { // first phase of masked write without register addr is READ
      twi_slarw = TW_READ; 
      twi_masterBufferLength -= 1; // adjust for read operation's NAK timing
    }
  }
  // build sla+w, slave device address + w bit
  twi_slarw |= address << 1;
  
  // send start condition
  TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWEA) | _BV(TWINT) | _BV(TWSTA);
  return 0;	// success
}

/*
int8_t twi_writeMaskedGeneric(uint8_t dev_addr, uint8_t* reg_addr_spec, uint8_t* data, unit8_t length) {
  if(TWI_READY != twi_state){
    return -1; // busy
  }
  // reset error state (0xFF.. no error occured)
  twi_error = 0xFF;
  source_data = data;
  source_length = length;  
  // initialize buffer iteration vars for reading phase
  twi_masterBufferPtr=data; // modified data is built in caller's data buffer during read
  twi_masterBufferIndex = 0;
  twi_masterBufferLength = length-1;
  twi_state = TWI_M_RMW;
  twi_reg_bytes = reg_addr_spec[0];
  if(twi_reg_bytes) {
    // initialize register spec
    twi_reg = reg_addr_spec[1];
    twi_reg2 = reg_addr_spec[2];
    twi_slarw = TW_WRITE;
  } else { // no register, simple read
    twi_slarw = TW_READ;
  }

  // build sla+w, slave device address + w bit
  twi_slarw |= address << 1;
  
  // send start condition
  TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWEA) | _BV(TWINT) | _BV(TWSTA);
  return 0;	// success
}

int8_t twi_writeGeneric(uint8_t dev_addr, uint8_t* reg_addr_spec, uint8_t* data, uint8_t length) {
  if(TWI_READY != twi_state){
    return -1; // busy
  }
  // reset error state (0xFF.. no error occured)
  twi_error = 0xFF;
  // initialize buffer iteration vars
  twi_reg_bytes = reg_addr_spec[0];
  if(twi_reg_bytes) { 
    twi_masterBufferPtr=reg_addr_spec+1;
    twi_masterBufferIndex = 0;
    twi_masterBufferLength = twi_reg_bytes;
    source_data = data;
    source_length = length;
  } else {
    twi_masterBufferPtr=data;
    twi_masterBufferIndex = 0;
    twi_masterBufferLength = length;
  }
  twi_state = TWI_MTX;
  twi_slarw = TW_WRITE;
  // build sla+w, slave device address + w bit
  twi_slarw |= address << 1;
  
  // send start condition
  TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWEA) | _BV(TWINT) | _BV(TWSTA);
  return 0;	// success
}
*/