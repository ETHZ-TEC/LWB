/*
 * Copyright (c) 2015, Swiss Federal Institute of Technology (ETH Zurich).
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Author:  Federico Ferrari
 */

#ifndef __RF1A_CORE_H__
#define __RF1A_CORE_H__

/* status byte (see Table 25-8) */
#define GET_RADIO_CORE_READY(status)  ((status & 0x80) >> 7)
#define GET_RF_STATE(status)          ((status & 0x70) >> 4)
#define GET_FIFO_BYTES_AVAIL(status)  ((status & 0x0f))

/* radio core main state machine (see Table 25-8) */
typedef enum {
  RF_STATE_IDLE = 0x0,
  RF_STATE_RX = 0x1,
  RF_STATE_TX = 0x2,
  RF_STATE_FSTXON = 0x3,
  RF_STATE_CALIBRATE = 0x4,
  RF_STATE_SETTLING = 0x5,
  RF_STATE_RX_OVERFLOW = 0x6,
  RF_STATE_TX_UNDERFLOW = 0x7
} rf1a_rf_states_t;

#define LQI_MASK              0x7f
#define CRC_MASK              0x80
/* (page 85 of the CC430F5137 datasheet) */
#define RSSI_OFFSET           74

/* possible states of the radio layer */
typedef enum {
  NO_RX_TX,
  RX,
  TX,
} rf1a_rx_tx_states_t;

/* possible off modes where the radio switches at the end of RX or TX */
typedef enum {
  RF1A_OFF_MODE_IDLE = 0x0,
  RF1A_OFF_MODE_FSTXON = 0x1,
  RF1A_OFF_MODE_TX = 0x2,
  RF1A_OFF_MODE_RX = 0x3
} rf1a_off_modes_t;

/* standard TX power values */
typedef enum {
  RF1A_TX_POWER_MINUS_30_dBm = 0x0,
  RF1A_TX_POWER_MINUS_12_dBm = 0x1,
  RF1A_TX_POWER_MINUS_6_dBm = 0x2,
  RF1A_TX_POWER_0_dBm = 0x3,
  RF1A_TX_POWER_PLUS_10_dBm = 0x4,
  RF1A_TX_POWER_MAX = 0x5,
  N_TX_POWER_LEVELS
} rf1a_tx_powers_t;

extern const char* rf1a_tx_powers_to_string[N_TX_POWER_LEVELS];

/* possible calibration modes */
typedef enum {
  RF1A_CALIBRATION_MODE_MANUAL = 0x0,
  RF1A_CALIBRATION_MODE_AUTOMATIC_FROM_IDLE = 0x1,
  RF1A_CALIBRATION_MODE_AUTOMATIC_TO_IDLE = 0x2,
  RF1A_CALIBRATION_MODE_AUTOMATIC_EVERY_FOURTH_TO_IDLE = 0x3
} rf1a_calibration_modes_t;

/* wait until the radio core is ready to accept the next instruction */
#define WAIT_UNTIL_READY_FOR_INSTR() while(!(RF1AIFCTL1 & RFINSTRIFG))

/* wait until the radio core is ready to accept additional data */
#define WAIT_UNTIL_READY_FOR_DATA() while(!(RF1AIFCTL1 & RFDINIFG))

/* wait until data has been provided by the radio core */
#define WAIT_UNTIL_DATA_IS_READY() while(!(RF1AIFCTL1 & RFDOUTIFG))

/* wait until the radio core has updated the status */
#define WAIT_UNTIL_STATUS_IS_READY() while(!(RF1AIFCTL1 & RFSTATIFG))

/* mark the radio core status as read */
#define MARK_STATUS_AS_READ() (RF1AIFCTL1 &= ~RFSTATIFG)

/* read a data byte from the radio register at address addr */
static inline uint8_t
read_byte_from_register(uint8_t addr)
{
  /* wait until the radio is ready to take an instruction */
  WAIT_UNTIL_READY_FOR_INSTR();

  /* send the instruction (one-byte auto-read) */
  if((addr <= 0x2E) || (addr == 0x3E)) {
    /* radio core configuration register */
    RF1AINSTR1B = RF_SNGLREGRD | addr;
  } else {
    /* radio core status register */
    RF1AINSTR1B = RF_STATREGRD | addr;
  }
  /* wait until the radio has provided the data */
  WAIT_UNTIL_DATA_IS_READY();

  /* read the data (clears also flag RFDOUTIFG) */
  uint8_t data = RF1ADOUTB;
  return data;
}
/* write a data byte to the radio register at address addr */
static inline void
write_byte_to_register(uint8_t addr, uint8_t data)
{
  /* wait until the radio is ready to take an instruction */
  WAIT_UNTIL_READY_FOR_INSTR();

  /* send the instruction */
  RF1AINSTRB = RF_SNGLREGWR | addr;
  
  /* send the data (clears also RFDINIFG) */
  RF1ADINB = data;
}
/* set the value of a (n_bits)-bit field with the specified offset */
/* into the radio register at address addr */
static inline void
set_register_field(uint8_t addr, uint8_t value, uint8_t n_bits, uint8_t offset)
{
  /* create the bit-mask */
  uint8_t mask = (1 << n_bits) - 1;
  /* accept only values that do not exceed the mask */
  if(value <= mask) {
    /* set the given value into the desired field */
    write_byte_to_register(addr,
                           (read_byte_from_register(
                              addr) & (~(mask << offset))) |
                           (value << offset));
  }
}
/* read n_bytes of buffer starting from the radio register at address addr */
static inline void
read_data_from_register(uint8_t addr, uint8_t *buffer, uint16_t n_bytes)
{
  /* execute only if there is at least one byte to read */
  if(n_bytes == 0) {
    return;
  }

  uint16_t i;
  /* wait until the radio is ready to take an instruction */
  WAIT_UNTIL_READY_FOR_INSTR();

  /* send the instruction (one-byte auto-read) */
  RF1AINSTR1B = RF_REGRD | addr;
  /* read byte-by-byte all bytes but the last one */
  for(i = 0; i < n_bytes - 1; i++) {
    /* wait until the radio has provided the data */
    WAIT_UNTIL_DATA_IS_READY();

    /* read the data (clears also flag RFDOUTIFG and initiates auto-read for
       the next data byte) */
    buffer[i] = RF1ADOUT1B;
  }
  /* wait until the radio has provided the data */
  WAIT_UNTIL_DATA_IS_READY();

  /* read the last data byte (clears also flag RFDOUTIFG, no auto-read) */
  buffer[n_bytes - 1] = RF1ADOUTB;
}
/* write n_bytes of buffer starting from the radio register at address addr */
static inline void
write_data_to_register(uint8_t addr, uint8_t *buffer, uint16_t n_bytes)
{
  /* execute only if there is at least one byte to write */
  if(n_bytes == 0) {
    return;
  }

  uint16_t i;
  /* wait until the radio is ready to take an instruction */
  WAIT_UNTIL_READY_FOR_INSTR();

  /* send the instruction and the first byte of data */
  RF1AINSTRW = ((RF_REGWR | addr) << 8) + buffer[0];

  /* write byte-by-byte all remaining bytes */
  for(i = 1; i < n_bytes; i++) {
    /* send one more byte */
    RF1ADINB = buffer[i];
    /* wait until the radio is ready to take more data */
    WAIT_UNTIL_READY_FOR_DATA();
  }

  /* read one byte (clears also flag RFDOUTIFG, no auto-read) */
  i = RF1ADOUTB;
}
/* load configuration exported from TI's SmartRF Studio */
static inline void
load_SmartRF_configuration(void)
{
  /* packet control */
#ifdef SMARTRF_PKTCTRL0  
  write_byte_to_register(PKTCTRL0, SMARTRF_PKTCTRL0);
#endif /* SMARTRF_PKTCTRL0 */
  
  /* frequency synthesizer */
#ifdef SMARTRF_FSCTRL1
  write_byte_to_register(FSCTRL1, SMARTRF_FSCTRL1);
#endif /* SMARTRF_FSCTRL1 */
#ifdef SMARTRF_FSCTRL0
  write_byte_to_register(FSCTRL1, SMARTRF_FSCTRL0);
#endif /* SMARTRF_FSCTRL0 */

  /* frequency control word */
#ifdef SMARTRF_FREQ2
  write_byte_to_register(FREQ2, SMARTRF_FREQ2);
#endif /* SMARTRF_FREQ2 */
#ifdef SMARTRF_FREQ1
  write_byte_to_register(FREQ1, SMARTRF_FREQ1);
#endif /* SMARTRF_FREQ1 */
#ifdef SMARTRF_FREQ0
  write_byte_to_register(FREQ0, SMARTRF_FREQ0);
#endif /* SMARTRF_FREQ0 */

  /* data rate, RX filter BW */
#ifdef SMARTRF_MDMCFG4
  write_byte_to_register(MDMCFG4, SMARTRF_MDMCFG4);
#endif /* SMARTRF_MDMCFG4 */
#ifdef SMARTRF_MDMCFG3
  write_byte_to_register(MDMCFG3, SMARTRF_MDMCFG3);
#endif /* SMARTRF_MDMCFG3 */

  /* modulation, sensitivity vs. current, Manchester, sync word bits */
#ifdef SMARTRF_MDMCFG2
  write_byte_to_register(MDMCFG2, SMARTRF_MDMCFG2);
#endif /* SMARTRF_MDMCFG2 */

  /* preamble bytes, exponent of channel spacing */
#ifdef SMARTRF_MDMCFG1
  write_byte_to_register(MDMCFG1, SMARTRF_MDMCFG1);
#endif /* SMARTRF_MDMCFG1 */

  /* mantissa of channel spacing */
#ifdef SMARTRF_MDMCFG0
  write_byte_to_register(MDMCFG0, SMARTRF_MDMCFG0);
#endif /* SMARTRF_MDMCFG0 */

  /* modem deviation */
#ifdef SMARTRF_DEVIATN
  write_byte_to_register(DEVIATN, SMARTRF_DEVIATN);
#endif /* SMARTRF_DEVIATN */

  /* frequency offset compensation */
#ifdef SMARTRF_FOCCFG
  write_byte_to_register(FOCCFG, SMARTRF_FOCCFG);
#endif /* SMARTRF_FOCCFG */

  /* bit synchronization configuration */
#ifdef SMARTRF_BSCFG
  write_byte_to_register(BSCFG, SMARTRF_BSCFG);
#endif /* SMARTRF_BSCFG */

  /* automatic gain control configuration */
#ifdef SMARTRF_AGCCTRL2
  write_byte_to_register(AGCCTRL2, SMARTRF_AGCCTRL2);
#endif /* SMARTRF_AGCCTRL2 */
#ifdef SMARTRF_AGCCTRL1
  write_byte_to_register(AGCCTRL1, SMARTRF_AGCCTRL1);
#endif /* SMARTRF_AGCCTRL1 */
#ifdef SMARTRF_AGCCTRL0
  write_byte_to_register(AGCCTRL0, SMARTRF_AGCCTRL0);
#endif /* SMARTRF_AGCCTRL0 */

  /* wake on radio control */
#ifdef SMARTRF_WORCTRL
  write_byte_to_register(WORCTRL, SMARTRF_WORCTRL);
#endif /* SMARTRF_WORCTRL */

  /* front end RX configuration */
#ifdef SMARTRF_FREND1
  write_byte_to_register(FREND1, SMARTRF_FREND1);
#endif /* SMARTRF_FREND1 */

  /* front end TX configuration */
#ifdef SMARTRF_FREND0
  write_byte_to_register(FREND0, SMARTRF_FREND0);
#endif /* SMARTRF_FREND0 */

  /* frequency synthesizer calibration */
#ifdef SMARTRF_FSCAL3
  write_byte_to_register(FSCAL3, SMARTRF_FSCAL3);
#endif /* SMARTRF_FSCAL3 */
#ifdef SMARTRF_FSCAL2
  write_byte_to_register(FSCAL2, SMARTRF_FSCAL2);
#endif /* SMARTRF_FSCAL2 */
#ifdef SMARTRF_FSCAL1
  write_byte_to_register(FSCAL1, SMARTRF_FSCAL1);
#endif /* SMARTRF_FSCAL1 */
#ifdef SMARTRF_FSCAL0
  write_byte_to_register(FSCAL0, SMARTRF_FSCAL0);
#endif /* SMARTRF_FSCAL0 */

  /* various test settings */
#ifdef SMARTRF_TEST2
  write_byte_to_register(TEST2, SMARTRF_TEST2);
#endif /* SMARTRF_TEST2 */
#ifdef SMARTRF_TEST1
  write_byte_to_register(TEST1, SMARTRF_TEST1);
#endif /* SMARTRF_TEST1 */
#ifdef SMARTRF_TEST0
  write_byte_to_register(TEST0, SMARTRF_TEST0);
#endif /* SMARTRF_TEST0 */
}
/* issue a command strobe and return the corresponding status byte */
/* rx = 1: returns the number of bytes currently in the RXFIFO queue */
/* rx = 0: returns the number of bytes currently in the TXFIFO queue */
/* NOTE: it is assumed here that a valid command strobe is passed to the
   function */
static inline uint8_t
strobe(uint8_t strobe, uint8_t rx)
{
  /* mark the radio status as read (to be sure we will read the new one after
     the strobe) */
  MARK_STATUS_AS_READ();
  /* wait until the radio is ready to take an instruction */
  WAIT_UNTIL_READY_FOR_INSTR();

  if(IS_XT2_ENABLED() || (strobe == RF_SXOFF) || (strobe == RF_SPWD) ||
     (strobe == RF_SWOR) ||
     (strobe == RF_SRES) || (strobe == RF_SNOP)) {
    /* if XT2 is enabled or the strobe does not put the radio into an active
       state */
    /* just issue the command strobe */
    RF1AINSTRB = strobe | (RF_RXSTAT * (rx & 0x1));
  } else {
    /* a transition from the SLEEP state to an active state requires the MCU
       to wait up to 810 us until XT2 becomes stable (see Section 25.3.3.7.1)*/

    /* store the current IOCFG2 value */
    uint8_t iocfg2 = read_byte_from_register(IOCFG2);
    /* set IOCFG2 to RF_RDYn (i.e., 0x29 based on Table 25-21) */
    write_byte_to_register(IOCFG2, 0x29);
    /* issue the command strobe */
    RF1AINSTRB = strobe | (RF_RXSTAT * (rx & 0x1));
    /* check whether the oscillator is not ready (i.e., RF_RDYn = 1) */
    if(RF1AIN & BIT2) {
      /* wait until RF_RDYn goes to 0 */
      while(RF1AIN & BIT2);
      /* wait for 810 us (= 16,200 clock ticks at 20 MHz) */
      __delay_cycles(16200);
    }
    /* restore the previous IOCFG2 value */
    write_byte_to_register(IOCFG2, iocfg2);
  }

  uint8_t status = 0;
  if(strobe != RF_SRES) {
    /* wait until the status byte is ready to be read */
    WAIT_UNTIL_STATUS_IS_READY();
    status = RF1ASTATB;
  }
  return status;
}
/* enable the radio core interrupts corresponding to the given flags */
#define ENABLE_INTERRUPTS(flags, positive_edge) \
  do { \
    if(positive_edge) { \
      RF1AIES &= ~flags; \
    } else { \
      RF1AIES |= flags; \
    } \
    /* clear pending interrupt flags */ \
    RF1AIFG &= ~flags; \
    /* enable the desired interrupt */ \
    RF1AIE |= flags; \
  } while(0)

/* disable the radio core interrupts corresponding to the given flags */
#define DISABLE_INTERRUPTS(flags) \
  do { \
    /* disable the desired interrupt */ \
    RF1AIE &= ~flags; \
    /* clear pending interrupt flags */ \
    RF1AIFG &= ~flags; \
  } while(0)

/* invert edges of the radio core interrupts corresponding to the given flags*/
#define INVERT_INTERRUPT_EDGES(flags) \
  do { \
    /* invert the edge of the desired interrupt */ \
    RF1AIES ^= flags; \
    /* clear pending interrupt flags */ \
    RF1AIFG &= ~flags; \
  } while(0)

#endif /* __RF1A_CORE_H__ */

