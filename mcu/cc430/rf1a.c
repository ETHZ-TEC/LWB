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
 *          Reto Da Forno
 */


#include "contiki.h" 
#include "platform.h"

#if RF_CONF_ON

/*---------------------------------------------------------------------------*/
/* default values */
#ifndef RF_CONF_TX_POWER
#define RF_CONF_TX_POWER        RF1A_TX_POWER_0_dBm
#endif /* RF_CONF_TX_POWER */

#ifndef RF_CONF_TX_CH
#define RF_CONF_TX_CH           0
#endif /* RF_CONF_TX_CH */

#ifndef RF_CONF_MAX_PKT_LEN
#define RF_CONF_MAX_PKT_LEN     255     /* max. is 255 */
#endif /* RF_CONF_MAX_PKT_LEN */
/*---------------------------------------------------------------------------*/
const int8_t rf1a_tx_power_val[N_TX_POWER_LEVELS] = { 
    -30, -12, -6, 0, 10, 12, 
};
/* number of bytes to read from the RX FIFO (or to write to the TX FIFO) */
/* after crossing a FIFO threshold */
/* must be between 3 and 63, in steps of 4 (e.g., 3, 7, 11, ...) */
#define FIFO_CHUNK_SIZE       15
/*---------------------------------------------------------------------------*/
/* state of the radio core */
static rf1a_rx_tx_states_t rf1a_state;
/* buffer used to manage packets longer than the RX/TX FIFO size (64 bytes) */
/* force its address to be even in order to avoid misalignment issues */
/* when executing the callback functions */
/* add +2 to account for rssi and lqi */
static uint8_t rf1a_buffer[RF_CONF_MAX_PKT_LEN +2] __attribute__((aligned(2)));
/* variables to indicate where is the starting point of the buffer (used for
   TX) and its length */
static uint8_t rf1a_buffer_start, rf1a_buffer_len;
/* length of the packet being received or transmitted */
static uint8_t packet_len;
static uint8_t packet_len_max;
static uint8_t header_len_rx, header_len_notified;
/* timestamp of radio events */
static rtimer_clock_t timestamp;
/* automatic mode switches at the end of RX and TX */
static rf1a_off_modes_t rxoff_mode, txoff_mode;
/* TX power level */
static rf1a_tx_powers_t rf1a_tx_pwr = RF_CONF_TX_POWER;
/*---------------------------------------------------------------------------*/
static inline void
read_bytes_from_rx_fifo(uint8_t n_bytes)
{
  /* note that rssi and lqi can be added to RX buffer, therefore +2 
   * (length byte is already accounted for) */
  if(rf1a_buffer_len + n_bytes <= packet_len_max + 2) {
    if(rf1a_buffer_len == 0) {
      /* no bytes read from the RX FIFO yet: store the first one as the packet
         length */
      packet_len = read_byte_from_register(RXFIFO);
      n_bytes--;
    }
    /* read the bytes from the RX FIFO and append them to the buffer */
    read_data_from_register(RXFIFO, &rf1a_buffer[rf1a_buffer_len], n_bytes);
    /* update the buffer indexes */
    rf1a_buffer_len += n_bytes;

    if((header_len_rx != 0) && (header_len_notified == 0) &&
       (rf1a_buffer_len >= header_len_rx)) {
      rf1a_cb_header_received(&timestamp, rf1a_buffer, packet_len);
      header_len_notified = 1;
    }
  } else {
    /* too many bytes: something went wrong */
    rf1a_cb_rx_tx_error(&timestamp);
  }
}
/*---------------------------------------------------------------------------*/
static inline void
energest_off_mode(rf1a_off_modes_t off_mode)
{
  SET_ENERGEST_TIME();
  switch(off_mode) {
  case RF1A_OFF_MODE_IDLE:
    ENERGEST_OFF_AT_TIME(ENERGEST_TYPE_LISTEN);
    ENERGEST_OFF_AT_TIME(ENERGEST_TYPE_TRANSMIT);
    ENERGEST_ON_AT_TIME(ENERGEST_TYPE_IDLE);
    break;
  case RF1A_OFF_MODE_RX:
    ENERGEST_OFF_AT_TIME(ENERGEST_TYPE_IDLE);
    ENERGEST_OFF_AT_TIME(ENERGEST_TYPE_TRANSMIT);
    ENERGEST_ON_AT_TIME(ENERGEST_TYPE_LISTEN);
    break;
  case RF1A_OFF_MODE_TX:
  case RF1A_OFF_MODE_FSTXON:
    ENERGEST_OFF_AT_TIME(ENERGEST_TYPE_LISTEN);
    ENERGEST_OFF_AT_TIME(ENERGEST_TYPE_IDLE);
    ENERGEST_ON_AT_TIME(ENERGEST_TYPE_TRANSMIT);
    break;
  }
}
/*---------------------------------------------------------------------------*/
void
rf1a_init(void)
{
  /* reset the radio core */
  rf1a_reset();

  rxoff_mode = RF1A_OFF_MODE_IDLE;
  txoff_mode = RF1A_OFF_MODE_IDLE;

  packet_len_max = RF_CONF_MAX_PKT_LEN;
  
  load_SmartRF_configuration();
  
  /* set transmit power, channel and packet length */
  rf1a_set_tx_power(RF_CONF_TX_POWER);
  rf1a_set_channel(RF_CONF_TX_CH);
  rf1a_set_maximum_packet_length(RF_CONF_MAX_PKT_LEN);

  /* set the FIFO threshold such that FIFO_CHUNK_SIZE bytes can be written/read
     after an interrupt */
  write_byte_to_register(FIFOTHR, (FIFO_CHUNK_SIZE - 3) / 4);

  /* output RF_RDY to GDO0 (RFIN0) -> if changed, don't forget to adjust
   * glossy_start() */
  rf1a_configure_gdo_signal(0, 0x29, 0);
  
  /* RSSI valid indicator */
  rf1a_configure_gdo_signal(1, 0x1e, 0); /* adjust glossy_start if changed! */
  /* output CRC_OK to GDO1 (RFIN1) */
  //rf1a_configure_gdo_signal(1, 7, 0);
  /* output the serial clock on GDO1 */
  /*rf1a_configure_gdo_signal(1, 0x0B, 0);*/
  
  /* map the sync word signal to GDO2 (corresponding to GDO2_CFG = 0x06, see
     Table 25-21) */
  rf1a_configure_gdo_signal(2, 6, 0);
  
  /* timer 4: capture input CCI4B, which corresponds to GDO2, i.e., to the sync
     word signal */
  /* synchronous capture on both edges */
  /* NOTE: as a result timer 4 is not available for being used as a rtimer */
  TA0CCTL4 = CM_3 | CCIS_1 | CAP | SCS;
  
  /* interrupt when the number of bytes in the RX FIFO is greater than the
     threshold */
  ENABLE_INTERRUPTS(BIT3, 1);
  /* interrupt when the number of bytes in the TX FIFO is smaller than the
   threshold */
  /* ENABLE_INTERRUPTS(BIT5, 0); */
  /* interrupt when RX FIFO overflows (RFIFG7) */
  ENABLE_INTERRUPTS(BIT7, 1);
  /* interrupt when TX FIFO underflows (RFIFG8) */
  ENABLE_INTERRUPTS(BIT8, 1);
  /* interrupt when sync word received or transmitted, or end of packet
     (RFIFG9) */
  ENABLE_INTERRUPTS(BIT9, 1);
    
  /* must set the PMMHPMRE bit to enable radio operation in LPM3/4 */
  PMMCTL0_H  = 0xa5;  /* unlock */
  PMMCTL0_L |= PMMHPMRE;
  PMMCTL0_H  = 0;        /* lock */
  
  printf("RF module configured (%ddBm, %u.%uMHz, %ub)\r\n",
         rf1a_tx_power_val[RF_CONF_TX_POWER], 
         RF_CONF_TX_CH / 5 + 868, (RF_CONF_TX_CH * 2) % 10,
         RF_CONF_MAX_PKT_LEN);
}
/*---------------------------------------------------------------------------*/
void
rf1a_reset(void)
{
  /* NOTE: the radio goes to the SLEEP state (see 25.3.1) */
  SET_ENERGEST_TIME();
  ENERGEST_OFF_AT_TIME(ENERGEST_TYPE_LISTEN);
  ENERGEST_OFF_AT_TIME(ENERGEST_TYPE_TRANSMIT);
  ENERGEST_OFF_AT_TIME(ENERGEST_TYPE_IDLE);
  /* issue the SRES command strobe */
  strobe(RF_SRES, 1);
  /* issue the SNOP command strobe */
  strobe(RF_SNOP, 1);
  /* clear all radio interface error interrupt flags */
  RF1AIFERR = 0;

  header_len_rx = 0;

  rf1a_state = NO_RX_TX;
}
/*---------------------------------------------------------------------------*/
uint8_t
rf1a_is_busy(void)
{
  return (rf1a_state == NO_RX_TX) ? 0 : 1;
}
/*---------------------------------------------------------------------------*/
void
rf1a_set_tx_power(rf1a_tx_powers_t tx_power_level)
{
  if(tx_power_level < N_TX_POWER_LEVELS) {
    /* register settings corresponding to TX power levels {-30, -12, -6, 0, +10,
        max} dBm */
    /* NOTE: these values work only for 2-FSK, 2-GFSK, and MSK modulation 
    * schemes */
    static const uint8_t pa_values[N_TX_POWER_LEVELS] =
    { 0x03, 0x25, 0x2d, 0x8d, 0xc3, 0xc0 };
    write_data_to_register(PATABLE, (uint8_t *)pa_values, N_TX_POWER_LEVELS);
    set_register_field(FREND0, tx_power_level, 3, 0);
    rf1a_tx_pwr = tx_power_level;
  }
}
/*---------------------------------------------------------------------------*/
void
rf1a_configure_gdo_signal(uint8_t gdo, uint8_t signal, uint8_t invert)
{
  if(gdo <= 2) {
    write_byte_to_register(IOCFG0 - gdo, (signal & 0x3f) | (invert << 6));
  }
}
/*---------------------------------------------------------------------------*/
uint8_t
rf1a_get_status_byte(void)
{
  /* issue the SNOP command strobe in order to get the radio core status */
  /* byte without causing any further actions */
  return strobe(RF_SNOP, 1);
}
/*---------------------------------------------------------------------------*/
void
rf1a_go_to_sleep(void)
{
  /* issue the SXOFF command strobe */
  strobe(RF_SXOFF, 1);
  rf1a_state = NO_RX_TX;

  SET_ENERGEST_TIME();
  ENERGEST_OFF_AT_TIME(ENERGEST_TYPE_LISTEN);
  ENERGEST_OFF_AT_TIME(ENERGEST_TYPE_TRANSMIT);
  ENERGEST_OFF_AT_TIME(ENERGEST_TYPE_IDLE);
}
/*---------------------------------------------------------------------------*/
void
rf1a_go_to_idle(void)
{
  /* issue the SIDLE command strobe */
  uint8_t status = strobe(RF_SIDLE, 1);
  /* wait until the radio core goes to the IDLE state */
  while(GET_RF_STATE(status) != RF_STATE_IDLE) {
    status = strobe(RF_SNOP, 1);
  }
  rf1a_state = NO_RX_TX;

  SET_ENERGEST_TIME();
  ENERGEST_OFF_AT_TIME(ENERGEST_TYPE_LISTEN);
  ENERGEST_OFF_AT_TIME(ENERGEST_TYPE_TRANSMIT);
  ENERGEST_ON_AT_TIME(ENERGEST_TYPE_IDLE);
}
/*---------------------------------------------------------------------------*/
void
rf1a_manual_calibration(void)
{
  /* issue the SIDLE command strobe */
  uint8_t status = strobe(RF_SIDLE, 1);
  /* wait until the radio core goes to the IDLE state */
  while(GET_RF_STATE(status) != RF_STATE_IDLE) {
    status = strobe(RF_SNOP, 1);
  }
  rf1a_state = NO_RX_TX;

  SET_ENERGEST_TIME();
  ENERGEST_OFF_AT_TIME(ENERGEST_TYPE_LISTEN);
  ENERGEST_OFF_AT_TIME(ENERGEST_TYPE_TRANSMIT);
  ENERGEST_ON_AT_TIME(ENERGEST_TYPE_IDLE);

  /* then issue the SCAL command strobe */
  strobe(RF_SCAL, 1);
  /* wait until the calibration is finished */
  while(read_byte_from_register(MARCSTATE) != 1) ;
}
/*---------------------------------------------------------------------------*/
void
rf1a_reconfig_after_sleep(void)
{
  /* re-configure the lost registers after SLEEP state */
  /* patable and power level */
  rf1a_set_tx_power(rf1a_tx_pwr);
  /* re-configure the TESTx registers (lost in sleep) */
  write_byte_to_register(TEST0, SMARTRF_TEST0);
#ifdef SMARTRF_TEST1
  write_byte_to_register(TEST1, SMARTRF_TEST1);
#endif /* SMARTRF_TEST1 */
#ifdef SMARTRF_TEST2
  write_byte_to_register(TEST2, SMARTRF_TEST2);
#endif /* SMARTRF_TEST2 */
}
/*---------------------------------------------------------------------------*/
void
rf1a_flush_rx_fifo(void)
{
  /* issue the SIDLE command strobe */
  uint8_t status = strobe(RF_SIDLE, 1);
  /* wait until the radio core goes to the IDLE state */
  while(GET_RF_STATE(status) != RF_STATE_IDLE) {
    status = strobe(RF_SNOP, 1);
  }
  rf1a_state = NO_RX_TX;

  SET_ENERGEST_TIME();
  ENERGEST_OFF_AT_TIME(ENERGEST_TYPE_LISTEN);
  ENERGEST_OFF_AT_TIME(ENERGEST_TYPE_TRANSMIT);
  ENERGEST_ON_AT_TIME(ENERGEST_TYPE_IDLE);

  /* then issue the SFRX command strobe */
  strobe(RF_SFRX, 1);
}
/*---------------------------------------------------------------------------*/
void
rf1a_flush_tx_fifo(void)
{
  /* issue the SIDLE command strobe */
  uint8_t status = strobe(RF_SIDLE, 1);
  /* wait until the radio core goes to the IDLE state */
  while(GET_RF_STATE(status) != RF_STATE_IDLE) {
    status = strobe(RF_SNOP, 1);
  }
  rf1a_state = NO_RX_TX;

  SET_ENERGEST_TIME();
  ENERGEST_OFF_AT_TIME(ENERGEST_TYPE_LISTEN);
  ENERGEST_OFF_AT_TIME(ENERGEST_TYPE_TRANSMIT);
  ENERGEST_ON_AT_TIME(ENERGEST_TYPE_IDLE);

  /* then issue the SFTX command strobe */
  strobe(RF_SFTX, 0);
}
/*---------------------------------------------------------------------------*/
void
rf1a_start_rx(void)
{
  SET_ENERGEST_TIME();
  ENERGEST_OFF_AT_TIME(ENERGEST_TYPE_IDLE);
  ENERGEST_OFF_AT_TIME(ENERGEST_TYPE_TRANSMIT);
  ENERGEST_ON_AT_TIME(ENERGEST_TYPE_LISTEN);

  /* issue the SRX command strobe */
  strobe(RF_SRX, 1);
}
/*---------------------------------------------------------------------------*/
void
rf1a_start_tx(void)
{
  SET_ENERGEST_TIME();
  ENERGEST_OFF_AT_TIME(ENERGEST_TYPE_IDLE);
  ENERGEST_OFF_AT_TIME(ENERGEST_TYPE_LISTEN);
  ENERGEST_ON_AT_TIME(ENERGEST_TYPE_TRANSMIT);

  /* issue the STX command strobe */
  strobe(RF_STX, 0);
  rf1a_state = TX;
}
/*---------------------------------------------------------------------------*/
void
rf1a_write_to_tx_fifo(uint8_t *header,
                      uint8_t header_len,
                      uint8_t *payload,
                      uint8_t payload_len)
{
  /* ensure that the TX FIFO threshold interrupt is disabled */
  DISABLE_INTERRUPTS(BIT5);

  /* check that the total packet length does not exceed the maximum allowed */
  packet_len = header_len + payload_len;
  if(packet_len > packet_len_max) {
    return;
  }
  
  /* write the packet length into the TX FIFO */
  write_byte_to_register(TXFIFO, packet_len);

  /* write the header into the TX FIFO */
  /* NOTE: it is assumed here that header_len is at most 63 bytes! */
  write_data_to_register(TXFIFO, header, header_len);

  /* compute the number of bytes still available in the TX FIFO */
  uint8_t free_tx_fifo_bytes = 63 - header_len;
  /* compute the number of payload bytes to write immediately into the TX
   * FIFO */
  uint8_t payload_bytes_to_tx_fifo;
  if(payload_len > free_tx_fifo_bytes) {
    /* write the first free_tx_fifo_bytes bytes of the payload into the TX
     * FIFO */
    payload_bytes_to_tx_fifo = free_tx_fifo_bytes;
  } else {
    /* write the whole payload into the TX FIFO */
    payload_bytes_to_tx_fifo = payload_len;
  }
  write_data_to_register(TXFIFO, payload, payload_bytes_to_tx_fifo);

  /* store the whole packet (header + payload) into the buffer */
  memcpy(rf1a_buffer, header, header_len);
  memcpy(&rf1a_buffer[header_len], payload, payload_len);
  /* mark that the header and payload_bytes_to_tx_fifo payload bytes */
  /* have already been written into the TX FIFO */
  rf1a_buffer_start = header_len + payload_bytes_to_tx_fifo;
  rf1a_buffer_len = packet_len - rf1a_buffer_start;

  if(payload_len > free_tx_fifo_bytes) {
    /* enable the TX FIFO threshold interrupt */
    ENABLE_INTERRUPTS(BIT5, 0);
  }
}
/*---------------------------------------------------------------------------*/
void
rf1a_tx_packet(uint8_t *header,
               uint8_t header_len,
               uint8_t *payload,
               uint8_t payload_len)
{
  /* flush the TX FIFO */
  rf1a_flush_tx_fifo();
  /* start writing the packet to the TX FIFO */
  rf1a_write_to_tx_fifo(header, header_len, payload, payload_len);
  /* start the transmission */
  rf1a_start_tx();
}
/*---------------------------------------------------------------------------*/
void
rf1a_set_rxoff_mode(rf1a_off_modes_t mode)
{
  set_register_field(MCSM1, mode, 2, 2);
  rxoff_mode = mode;
}
/*---------------------------------------------------------------------------*/
void
rf1a_set_txoff_mode(rf1a_off_modes_t mode)
{
  set_register_field(MCSM1, mode, 2, 0);
  txoff_mode = mode;
}
/*---------------------------------------------------------------------------*/
int8_t
rf1a_get_rssi(void)
{
  /* transform the RSSI value to dBm (see Section 25.3.3.6.3) */
  int8_t rssi = (int8_t)read_byte_from_register(RSSI) / 2 - RSSI_OFFSET;
  return rssi;
}
/*---------------------------------------------------------------------------*/
uint8_t
rf1a_get_lqi(void)
{
  uint8_t lqi = read_byte_from_register(LQI) & LQI_MASK;
  return lqi;
}
/*---------------------------------------------------------------------------*/
int8_t
rf1a_get_last_packet_rssi(void)
{
  /* extract the RSSI value appended to the packet and transform it to dBm (see
     Section 25.3.3.6.3) */
  int8_t rssi = (int8_t)rf1a_buffer[packet_len] / 2 - RSSI_OFFSET;
  return rssi;
}
/*---------------------------------------------------------------------------*/
uint8_t
rf1a_get_last_packet_lqi(void)
{
  /* extract the LQI value appended to the packet */
  uint8_t lqi = rf1a_buffer[packet_len + 1] & LQI_MASK;
  return lqi;
}
/*---------------------------------------------------------------------------*/
void
rf1a_set_maximum_packet_length(uint8_t length)
{
  /* ensure that we are in variable packet length mode (default value) */
  set_register_field(PKTCTRL0, 0x1, 2, 0);
  /* set the maximum packet length */
  if(length > RF_CONF_MAX_PKT_LEN) {
    packet_len_max = RF_CONF_MAX_PKT_LEN;
  } else {
    packet_len_max = length;
  }
  write_byte_to_register(PKTLEN, packet_len_max);
}
/*---------------------------------------------------------------------------*/
void
rf1a_set_channel(uint8_t channel)
{
  write_byte_to_register(CHANNR, channel);
}
/*---------------------------------------------------------------------------*/
void
rf1a_set_header_len_rx(uint8_t header_len)
{
  header_len_rx = header_len;
}
/*---------------------------------------------------------------------------*/
void
rf1a_set_calibration_mode(rf1a_calibration_modes_t mode)
{
  set_register_field(MCSM0, mode, 2, 4);
}
/*---------------------------------------------------------------------------*/
void
rf1a_clear_pending_interrupts(void)
{
  /* disable TX threshold interrupt */
  DISABLE_INTERRUPTS(BIT5);
  /* enable sync word interrupt (positive edge) */
  ENABLE_INTERRUPTS(BIT9, 1);
  /* clear all interrupt flags */
  RF1AIFG = 0;
}
/*---------------------------------------------------------------------------*/
ISR(CC1101, radio_interrupt) 
{
  DEBUG_ISR_ENTRY;
  DCSTAT_CPU_ON;
  ENERGEST_ON(ENERGEST_TYPE_CPU);

  /* take a timestamp */
  timestamp = rtimer_now_hf();

  switch(RF1AIV) {

  case RF1AIV_RFIFG3:
    /* RX FIFO above threshold: read FIFO_CHUNK_SIZE bytes */
    read_bytes_from_rx_fifo(FIFO_CHUNK_SIZE);
    break;

  case RF1AIV_RFIFG5:
    /* TX FIFO below threshold */
    if(rf1a_buffer_len > 0) {
      /* there are still bytes to write into the TX FIFO */
      if(rf1a_buffer_len > FIFO_CHUNK_SIZE) {
        /* write FIFO_CHUNK_SIZE more bytes into the TX FIFO */
        write_data_to_register(TXFIFO,
                               &rf1a_buffer[rf1a_buffer_start],
                               FIFO_CHUNK_SIZE);
        /* update the buffer indexes */
        rf1a_buffer_start += FIFO_CHUNK_SIZE;
        rf1a_buffer_len -= FIFO_CHUNK_SIZE;
      } else {
        /* write the remaining rf1a_buffer_len bytes into the TX FIFO */
        write_data_to_register(TXFIFO,
                               &rf1a_buffer[rf1a_buffer_start],
                               rf1a_buffer_len);
        /* reset the buffer indexes */
        rf1a_buffer_start = 0;
        rf1a_buffer_len = 0;
        /* no more bytes left to write into the the TX FIFO: */
        /* disable the TX FIFO threshold interrupt */
        DISABLE_INTERRUPTS(BIT5);
      }
    }
    break;

  case RF1AIV_RFIFG7:
    /* RX FIFO overflowed */
    if(RF1AIN & BIT7) {       /* RF1A5 errata, only handle if signal is high */
      rf1a_cb_rx_tx_error(&timestamp);
      rf1a_state = NO_RX_TX;
    }
    break;

  case RF1AIV_RFIFG8:
    /* TX FIFO underflowed */
    if(RF1AIN & BIT8) {       /* RF1A5 errata, only handle if signal is high */
      rf1a_cb_rx_tx_error(&timestamp);
      rf1a_state = NO_RX_TX;
    }
    break;

  /* Asserts when sync word has been sent or received, and deasserts at the end
   * of the packet. In RX, the pin deassert when the optional address check
   * fails or the RX FIFO overflows. In TX the pin deasserts if the TX FIFO
   * underflows. */
  case RF1AIV_RFIFG9:
    /* sync word received or transmitted, or end of packet */
    /* correct the timestamp based on the time captured by timer 4 */
    timestamp = timestamp - ((uint16_t)(timestamp & 0xffff) - TA0CCR4);
    if(!(RF1AIES & BIT9)) {
      /* sync word received or transmitted */
      switch(GET_RF_STATE(rf1a_get_status_byte())) {
      case RF_STATE_RX:
        /* sync word received */
        rf1a_state = RX;
        /* reset the buffer indexes */
        rf1a_buffer_len = 0;
        rf1a_buffer_start = 0;
        header_len_notified = 0;
        /* invert the edge for the next interrupt */
        INVERT_INTERRUPT_EDGES(BIT9);
        rf1a_cb_rx_started(&timestamp);
        break;
      case RF_STATE_TX:
        /* sync word transmitted */
        rf1a_state = TX;
        /* invert the edge for the next interrupt */
        INVERT_INTERRUPT_EDGES(BIT9);
        /* invert the edge for the next interrupt */
        rf1a_cb_tx_started(&timestamp);
        break;
      default:
        /* RX or TX already ended, or some other error has occurred 
         * This could e.g. happen with bad timing, when TX or RX has just
         * started when glossy_stop() is called */
        rf1a_state = NO_RX_TX;
        //rf1a_cb_rx_tx_error(&timestamp);
        break;
      }
    } else {
      /* Errata RF1A5: only proceed if input signal is low (added by rdaforno)
       * -> removed, seems to cause problems! */
      //if(RF1AIN & BIT9) { break; }
      
      /* invert the edge for the next interrupt */
      INVERT_INTERRUPT_EDGES(BIT9);

      /* end of packet (high-to-low transition) */
      switch(rf1a_state) {
      case RX:
        /* RX ended */
        energest_off_mode(rxoff_mode);
        if(read_byte_from_register(PKTSTATUS) & BIT7) {
          /* CRC OK */
          /* read the remaining bytes from the RX FIFO */
          read_bytes_from_rx_fifo(read_byte_from_register(RXBYTES));
          /* execute the callback function */
          rf1a_cb_rx_ended(&timestamp, rf1a_buffer, packet_len);
        } else {
          /* CRC not OK */
          rf1a_cb_rx_failed(&timestamp);
        }
        break;
      case TX:
        /* TX ended */
        energest_off_mode(txoff_mode);
        rf1a_cb_tx_ended(&timestamp);
        break;
      default:
        /* there is no RX or TX to end, either some error occurred or the RX
         * was ended earlier already (e.g. because of corrupted header or if
         * glossy_stop was called when SFD was high) */
        /* some occurances could also be due to errata RF1A5 */
        //rf1a_cb_rx_tx_error(&timestamp);
        break;
      }
      rf1a_state = NO_RX_TX;
    }
    break;

  default:
    break;
  }

  ENERGEST_OFF(ENERGEST_TYPE_CPU);
  DCSTAT_CPU_OFF;
  DEBUG_ISR_EXIT;
}
/*---------------------------------------------------------------------------*/

#endif /* RF_CONF_ON */
