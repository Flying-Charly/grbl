/*
  i2c_tcb.h Transmission Control Block-based I2C/TWI library for AVR
  (c) 2012-2013 Chuck Harrison for http://opensourceecology.org
  inspired by twi.h - TWI/I2C library for Wiring & Arduino
  Copyright (c) 2006 Nicholas Zambetti.  All right reserved.

  transaction based extensions
  (c) 2013 Chuck Harrison for http://opensourceecology.org, same license
  
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

#ifndef i2c_tcb_h
#define i2c_tcb_h

  #include <inttypes.h>

  //#define ATMEGA8

  #ifndef TWI_FREQ
  #define TWI_FREQ 400000L
  #endif

  #define TWI_READY 0
  #define TWI_MRX   1
  #define TWI_MTX   2
  #define TWI_SRX   3
  #define TWI_STX   4
  #define TWI_MTRX  5
  #define TWI_M_RMW 6
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

union pointer {
  void* addr;
  struct {
    uint8_t msb;
    uint8_t lsb;
  };
};

// A quickread transaction is a one-byte I2C read intended to update a mirror register
// To queue a quickread, increment its 'pending' field
struct quickread {
  uint8_t pending;
  uint8_t data; // mirror register
  const uint8_t device; // I2C device address, 7-bit convention
  uint8_t reg_spec[2];
};
// would have preferred to make these inline functions, but avr-gcc optimizes these better
extern volatile struct quickread quickreads[];
#define QUICKREAD_DATA(x) quickreads[(x)].data
#define QUEUE_QUICKREAD(x) queue_quickread(x)

inline void queue_quickread(uint8_t i);

struct tcb {
  uint8_t flags;
};


void twi_process_queue();
int8_t queue_TWI(struct tcb* control_block);
/*** TBD
int8_t setbit_TWI(struct tcb* control_block, uint8_t bitmask, uint8_t sync_mode);
  // sync_modes  queue: wait on availability or return regardless
  //             completion: wait on service start, wait on completion,
  //                         or return regardless
  //             timeout value if wait

int8_t clearbit_TWI(struct tcb* control_block, uint8_t bitmask, uint8_t sync_mode);
***/

  void twi_init(void);
  void twi_reply(uint8_t);
  void twi_stop(void);
  void twi_releaseBus(void);

  void twi_fifo_init();
#endif

