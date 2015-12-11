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

/*---------------------------------------------------------------------------*/
#ifndef GLOSSY_CONF_RTIMER_ID
#define GLOSSY_CONF_RTIMER_ID   RTIMER_HF_3
#endif /* GLOSSY_CONF_RTIMER_ID */
/*---------------------------------------------------------------------------*/

/* minimum and maximum number of slots after which the timeout expires, since
 * the last transmission
 * NOTE: values below 2 do not make sense, as there would be no chance to
 * receive a packet in between */
#define SLOT_TIMEOUT_MIN 2
#define SLOT_TIMEOUT_MAX 2
/* number of extra ticks required by the timeout callback before starting the
 * transmission (to keep synchronous transmissions and time synchronization as
 * accurate as possible) */
#define TIMEOUT_EXTRA_TICKS 70

/* maximum tolerance to accept measurements of T_slot when comparing to the
 * theoretical value (value in clock ticks) */
#define T_SLOT_TOLERANCE 10

#define GLOSSY_COMMON_HEADER 0x80

#define SET_PKT_TYPE(pkt_type, sync, n_tx_max) \
  (pkt_type = GLOSSY_COMMON_HEADER | ((sync) & 0x30) | ((n_tx_max) & 0x0f))
#define SET_SYNC(pkt_type, sync) \
  (pkt_type = ((pkt_type) & 0xcf) | ((sync) & 0x30))
#define SET_N_TX_MAX(pkt_type, n_tx_max) \
  (pkt_type = ((pkt_type) & 0xf0) | ((n_tx_max) & 0x0f))

#define GET_COMMON_HEADER(pkt_type)     ((pkt_type) & 0xc0)
#define GET_SYNC(pkt_type)              ((pkt_type) & 0x30)
#define GET_N_TX_MAX(pkt_type)          ((pkt_type) & 0x0f)

#define IS_INITIATOR() \
  (g.header.initiator_id == node_id)
#define WITH_SYNC() \
  (GET_SYNC(g.header.pkt_type) == GLOSSY_WITH_SYNC)
#define WITH_RELAY_CNT() \
  ((WITH_SYNC()) || (GET_SYNC(g.header.pkt_type) == GLOSSY_ONLY_RELAY_CNT))

#define GLOSSY_HEADER_LEN(pkt_type) \
  ((GET_SYNC(pkt_type) == GLOSSY_WITHOUT_SYNC) ? 3 : 4)

/* MCU specific code for disabling undesired interrupts during a glossy flood.
 * All but the following interrupts will be disabled: 
 * radio, watchdog and the timer CCR needed for glossy */
#ifndef GLOSSY_DISABLE_INTERRUPTS
#define GLOSSY_DISABLE_INTERRUPTS {\
    g.enabled_interrupts  = 0;\
    g.enabled_interrupts |= (CBINT & CBIIE) ? (1 << 0) : 0;\
    g.enabled_interrupts |= (CBINT & CBIE) ? (1 << 1) : 0;\
    CBINT &= ~(CBIIE | CBIE);\
    g.enabled_interrupts |= (UCA0IE & UCTXIE) ? (1 << 2) : 0;\
    g.enabled_interrupts |= (UCA0IE & UCRXIE) ? (1 << 3) : 0;\
    UCA0IE &= ~(UCTXIE | UCRXIE);\
    g.enabled_interrupts |= (UCB0IE & UCTXIE) ? (1 << 4) : 0;\
    g.enabled_interrupts |= (UCB0IE & UCRXIE) ? (1 << 5) : 0;\
    UCB0IE &= ~(UCTXIE | UCRXIE);\
    g.enabled_interrupts |= (TA0CTL & TAIE) ? (1 << 6) : 0;\
    g.enabled_interrupts |= (TA1CTL & TAIE) ? (1 << 7) : 0;\
    TA0CTL &= ~TAIE;\
    TA1CTL &= ~TAIE;\
    /*g.enabled_interrupts |= (TA0CCTL0 & CCIE) ? (1 << 8) : 0;*/\
    g.enabled_interrupts |= (TA0CCTL1 & CCIE) ? (1 << 9) : 0;\
    g.enabled_interrupts |= (TA0CCTL2 & CCIE) ? (1 << 10) : 0;\
    g.enabled_interrupts |= (TA0CCTL3 & CCIE) ? (1 << 11) : 0; */\
    /*g.enabled_interrupts |= (TA0CCTL4 & CCIE) ? (1 << 12) : 0; */\
    /*TA0CCTL0 &= ~CCIE;*/\
    TA0CCTL1 &= ~CCIE;\
    TA0CCTL2 &= ~CCIE;\
    TA0CCTL3 &= ~CCIE;\
    /*TA0CCTL4 &= ~CCIE; -> required for radio */\
    g.enabled_interrupts |= (TA1CCTL0 & CCIE) ? (1 << 13) : 0;\
    /*g.enabled_interrupts |= (TA1CCTL1 & CCIE) ? (1 << 14) : 0;*/\
    g.enabled_interrupts |= (TA1CCTL2 & CCIE) ? (1 << 15) : 0;\
    TA1CCTL0 &= ~CCIE;\
    /*TA1CCTL1 &= ~CCIE;*/\
    TA1CCTL2 &= ~CCIE;\
    g.enabled_interrupts |= (DMA0CTL & DMAIE) ? (1 << 16) : 0;\
    g.enabled_interrupts |= (DMA1CTL & DMAIE) ? (1 << 17) : 0;\
    g.enabled_interrupts |= (DMA2CTL & DMAIE) ? (1 << 18) : 0;\
    DMA0CTL &= ~DMAIE;\
    DMA1CTL &= ~DMAIE;\
    DMA2CTL &= ~DMAIE;\
    g.enabled_interrupts |= (RTCCTL0 & RTCTEVIE) ? (1 << 19) : 0;\
    g.enabled_interrupts |= (RTCCTL0 & RTCAIE) ? (1 << 20) : 0;\
    g.enabled_interrupts |= (RTCCTL0 & RTCRDYIE) ? (1 << 21) : 0;\
    RTCCTL0 &= ~(RTCTEVIE | RTCAIE | RTCRDYIE);\
    g.enabled_interrupts |= (RTCPS0CTL & RT0PSIE) ? (1 << 22) : 0;\
    g.enabled_interrupts |= (RTCPS1CTL & RT1PSIE) ? (1 << 23) : 0;\
    g.enabled_interrupts |= (AESACTL0 & AESRDYIE) ? (1 << 24) : 0;\
    RTCPS0CTL &= ~RT0PSIE;\
    RTCPS1CTL &= ~RT1PSIE;\
    AESACTL0 &= ~AESRDYIE;\
    enabled_port_interrupts = ((uint16_t)P1IE << 8) | (uint16_t)P2IE;\
    P1IE = 0;\
    P2IE = 0;\
    enabled_adc_interrupts = ADC12IE;\
    ADC12IE = 0;}
#endif /* GLOSSY_DISABLE_INTERRUPTS */

#ifndef GLOSSY_ENABLE_INTERRUPTS
#define GLOSSY_ENABLE_INTERRUPTS {\
    if(g.enabled_interrupts & (1 << 0)) { CBINT |= CBIIE; }\
    if(g.enabled_interrupts & (1 << 1)) { CBINT |= CBIE; }\
    if(g.enabled_interrupts & (1 << 2)) { UCA0IE |= UCTXIE; }\
    if(g.enabled_interrupts & (1 << 3)) { UCA0IE |= UCRXIE; }\
    if(g.enabled_interrupts & (1 << 4)) { UCB0IE |= UCTXIE; }\
    if(g.enabled_interrupts & (1 << 5)) { UCB0IE |= UCRXIE; }\
    if(g.enabled_interrupts & (1 << 6)) { TA0CTL |= TAIE; }\
    if(g.enabled_interrupts & (1 << 7)) { TA1CTL |= TAIE; }\
    if(g.enabled_interrupts & (1 << 8)) { TA0CCTL0 |= CCIE; }\
    if(g.enabled_interrupts & (1 << 9)) { TA0CCTL1 |= CCIE; }\
    if(g.enabled_interrupts & (1 << 10)) { TA0CCTL2 |= CCIE; }\
    if(g.enabled_interrupts & (1 << 11)) { TA0CCTL3 |= CCIE; }\
    if(g.enabled_interrupts & (1 << 12)) { TA0CCTL4 |= CCIE; }\
    if(g.enabled_interrupts & (1 << 13)) { TA1CCTL0 |= CCIE; }\
    if(g.enabled_interrupts & (1 << 14)) { TA1CCTL1 |= CCIE; }\
    if(g.enabled_interrupts & (1 << 15)) { TA1CCTL2 |= CCIE; }\
    if(g.enabled_interrupts & (1 << 16)) { DMA0CTL |= DMAIE; }\
    if(g.enabled_interrupts & (1 << 17)) { DMA1CTL |= DMAIE; }\
    if(g.enabled_interrupts & (1 << 18)) { DMA2CTL |= DMAIE; }\
    if(g.enabled_interrupts & (1 << 19)) { RTCCTL0 |= RTCTEVIE; }\
    if(g.enabled_interrupts & (1 << 20)) { RTCCTL0 |= RTCAIE; }\
    if(g.enabled_interrupts & (1 << 21)) { RTCCTL0 |= RTCRDYIE; }\
    if(g.enabled_interrupts & (1 << 22)) { RTCPS0CTL |= RT0PSIE; }\
    if(g.enabled_interrupts & (1 << 23)) { RTCPS1CTL |= RT1PSIE; }\
    if(g.enabled_interrupts & (1 << 24)) { AESACTL0 |= AESRDYIE; }\
    P2IE = enabled_port_interrupts & 0xff;\
    P1IE = (enabled_port_interrupts >> 8) & 0xff;\
    ADC12IE = enabled_adc_interrupts;}
#endif /* GLOSSY_ENABLE_INTERRUPTS */

/* mainly for debugging purposes */
#ifdef GLOSSY_START_PIN
#define GLOSSY_STARTED      PIN_SET(GLOSSY_START_PIN)
#define GLOSSY_STOPPED      PIN_CLR(GLOSSY_START_PIN)
#else
#define GLOSSY_STARTED
#define GLOSSY_STOPPED
#endif

#ifdef GLOSSY_RX_PIN 
#define GLOSSY_RX_STARTED   PIN_SET(GLOSSY_RX_PIN)
#define GLOSSY_RX_STOPPED   PIN_CLR(GLOSSY_RX_PIN)
#else
#define GLOSSY_RX_STARTED
#define GLOSSY_RX_STOPPED
#endif

#ifdef GLOSSY_TX_PIN
#define GLOSSY_TX_STARTED   PIN_SET(GLOSSY_TX_PIN)
#define GLOSSY_TX_STOPPED   PIN_CLR(GLOSSY_TX_PIN)
#else
#define GLOSSY_TX_STARTED
#define GLOSSY_TX_STOPPED
#endif

/*---------------------------------------------------------------------------*/
enum {
  SUCCESS = 0,
  FAIL = 1
};
/*---------------------------------------------------------------------------*/
typedef struct {
  uint16_t initiator_id;
  uint8_t pkt_type;
  uint8_t relay_cnt;
} glossy_header_t;
/*---------------------------------------------------------------------------*/
typedef struct {
  rtimer_clock_t t_ref, t_tx_stop, t_rx_start, t_rx_stop, t_tx_start;
  rtimer_clock_t T_slot_sum;
  rtimer_clock_t T_slot_estimated;
  rtimer_clock_t t_timeout;
  uint8_t n_T_slot;
  glossy_header_t header;
  uint8_t *payload;
  uint8_t payload_len;
  uint8_t active;
  uint8_t n_rx;
  uint8_t n_tx;
  uint8_t relay_cnt_last_rx, relay_cnt_last_tx, relay_cnt_first_rx,
          relay_cnt_t_ref;
  uint8_t relay_cnt_timeout;
  uint8_t t_ref_updated;
  uint8_t header_ok;
#ifdef GLOSSY_DISABLE_INTERRUPTS
  uint32_t enabled_interrupts;
  uint16_t enabled_adc_interrupts;
  uint16_t enabled_port_interrupts;
#endif /* GLOSSY_DISABLE_INTERRUPTS */
  int16_t  rssi_sum;
  int16_t  rssi_noise;
  uint32_t pkt_cnt;
  uint32_t corrupted_pkt_cnt;
} glossy_state_t;
/*---------------------------------------------------------------------------*/
static glossy_state_t g;

/*------------------------ Glossy helper functions --------------------------*/
static inline uint8_t
process_glossy_header(uint8_t *pkt, uint8_t pkt_len, uint8_t crc_ok)
{
  /* extract the Glossy header from the packet */
  glossy_header_t *rcvd_header = (glossy_header_t *)pkt;

  if(!g.header_ok) {
    /* we have not checked the header yet, so check it now */

    if(GET_COMMON_HEADER(rcvd_header->pkt_type) != GLOSSY_COMMON_HEADER) {
      /* keep processing only if the common header is correct */
      return FAIL;
    }

    if((GET_SYNC(g.header.pkt_type) != GLOSSY_UNKNOWN_SYNC) &&
       (GET_SYNC(g.header.pkt_type) != GET_SYNC(rcvd_header->pkt_type))) {
      /* keep processing only if the local sync value is either unknown or it
       * matches the received one */
      return FAIL;
    }

    if((GET_N_TX_MAX(g.header.pkt_type) != GLOSSY_UNKNOWN_N_TX_MAX) &&
       GET_SYNC(g.header.pkt_type) != GET_SYNC(rcvd_header->pkt_type)) {
      /* keep processing only if the local n_tx_max value is either unknown or
       * it matches the received one */
      return FAIL;
    }

    if((g.header.initiator_id != GLOSSY_UNKNOWN_INITIATOR) &&
       (g.header.initiator_id != rcvd_header->initiator_id)) {
      /* keep processing only if the local initiator_id value is either unknown
       * or it matches the received one */
      return FAIL;
    }

    if((g.payload_len != GLOSSY_UNKNOWN_PAYLOAD_LEN) &&
       (g.payload_len != (pkt_len - GLOSSY_HEADER_LEN(g.header.pkt_type)))) {
      /* keep processing only if the local payload_len value is either unknown
       * or it matches the received one */
      return FAIL;
    }

    /* the header is ok */
    g.header_ok = 1;
  }

  if(crc_ok) {
    /* we have received the entire packet (and the CRC was ok) */

    /* store the received header (all the unknown values are also learned) */
    g.header = *rcvd_header;
    /* store the payload_len */
    g.payload_len = pkt_len - GLOSSY_HEADER_LEN(g.header.pkt_type);
    /* store the header_len */
    rf1a_set_header_len_rx(GLOSSY_HEADER_LEN(g.header.pkt_type));
  }

  return SUCCESS;
}
/*---------------------------------------------------------------------------*/
static inline rtimer_clock_t
estimate_T_slot(uint8_t pkt_len)
{
  rtimer_clock_t T_tx_estim = T_TX_BYTE * (pkt_len + 3) + T_TX_OFFSET;
  return NS_TO_RTIMER_HF(T_tx_estim + T2R - TAU1);
}
/*---------------------------------------------------------------------------*/
static inline char
timeout_expired(rtimer_t *rt)
{
  if(!rf1a_is_busy()) {
    /* we are not receiving anything: retransmit the packet */
    rf1a_start_tx();
    g.header.relay_cnt = g.relay_cnt_timeout;
    rf1a_write_to_tx_fifo((uint8_t *)&g.header,
                          GLOSSY_HEADER_LEN(g.header.pkt_type),
                          (uint8_t *)g.payload, g.payload_len);
    g.t_timeout = rt->time;
  } else {
    /* we are receiving a packet: postpone the timeout by one slot */
    g.relay_cnt_timeout++;
    rtimer_schedule(GLOSSY_CONF_RTIMER_ID, 
                    rt->time + g.T_slot_estimated, 
                    0, 
                    timeout_expired);
  }
  return 0;
}
/*---------------------------------------------------------------------------*/
static inline void
schedule_timeout(void)
{
  /* number of slots after which the timeout will expire: */
  /* random number between SLOT_TIMEOUT_MIN and SLOT_TIMEOUT_MAX */
  uint8_t slot_timeout = SLOT_TIMEOUT_MIN +
    (random_rand() % (SLOT_TIMEOUT_MAX - SLOT_TIMEOUT_MIN + 1));
  if(WITH_RELAY_CNT()) {
    /* if the relay counter is sent, increment it by the chosen number of
     * slots */
    g.relay_cnt_timeout = g.header.relay_cnt + slot_timeout;
  }
  rtimer_schedule(GLOSSY_CONF_RTIMER_ID, 
                  g.t_timeout + slot_timeout * g.T_slot_estimated, 
                  0,
                  timeout_expired);
}
/*---------------------------------------------------------------------------*/
static inline void
update_t_ref(rtimer_clock_t t_ref, uint8_t relay_cnt)
{
  g.t_ref = t_ref;
  g.t_ref_updated = 1;
  g.relay_cnt_t_ref = relay_cnt;
}
/*---------------------------------------------------------------------------*/
static inline void
add_T_slot_measurement(rtimer_clock_t T_slot_measured)
{
  if((T_slot_measured > g.T_slot_estimated - T_SLOT_TOLERANCE) &&
     (T_slot_measured < g.T_slot_estimated + T_SLOT_TOLERANCE)) {
    g.T_slot_sum += T_slot_measured;
    g.n_T_slot++;
  }
}
/*---------------------------- Glossy interface -----------------------------*/
void
glossy_start(uint16_t initiator_id, uint8_t *payload, uint8_t payload_len,
             uint8_t n_tx_max, glossy_sync_t sync, glossy_rf_cal_t rf_cal)
{
  GLOSSY_STARTED;
  DEBUG_PRINT_VERBOSE("Glossy started: in=%u, pl=%u, n=%u, s=%u", initiator_id,
                      payload_len, n_tx_max, sync);

  /* disable undesired interrupts */
  GLOSSY_DISABLE_INTERRUPTS;

  g.active = 1;
  g.payload = payload;
  g.payload_len = payload_len;
  g.n_rx = 0;
  g.n_tx = 0;
  g.relay_cnt_last_rx = 0;
  g.relay_cnt_last_tx = 0;
  g.t_ref_updated = 0;
  g.T_slot_sum = 0;
  g.n_T_slot = 0;
  g.rssi_sum = 0;

  /* prepare the Glossy header, with the information known so far */
  g.header.initiator_id = initiator_id;
  SET_PKT_TYPE(g.header.pkt_type, sync, n_tx_max);
  g.header.relay_cnt = 0;

  /* automatically switch to TX at the end of RX */
  rf1a_set_rxoff_mode(RF1A_OFF_MODE_TX);
  /* automatically switch to RX at the end of TX */
  rf1a_set_txoff_mode(RF1A_OFF_MODE_RX);
  /* do not calibrate automatically */
  rf1a_set_calibration_mode(RF1A_CALIBRATION_MODE_MANUAL);
  
  /* re-configure patable (config is lost when radio was in sleep mode) */
  rf1a_set_tx_power(RF_CONF_TX_POWER);

  if(rf_cal == GLOSSY_WITH_RF_CAL) {
    /* if instructed so, perform a manual calibration */
    rf1a_manual_calibration();
  }

  rf1a_set_header_len_rx(GLOSSY_HEADER_LEN(g.header.pkt_type));

  rf1a_go_to_idle();
  
  if(IS_INITIATOR()) {
    /* Glossy initiator */
    if(GET_SYNC(g.header.pkt_type) == GLOSSY_UNKNOWN_SYNC) {
      /* the initiator must know whether there will be synchronization or
       * not! */
      glossy_stop();
    } else {
      /* start the first transmission */
      g.t_timeout = rtimer_now_hf() + TIMEOUT_EXTRA_TICKS;
      rf1a_start_tx();
      rf1a_write_to_tx_fifo((uint8_t *)&g.header,
                            GLOSSY_HEADER_LEN(g.header.pkt_type),
                            (uint8_t *)g.payload, g.payload_len);
      g.relay_cnt_timeout = 0;
    }
  } else {
    /* Glossy receiver */
    rf1a_start_rx();        
    /* wait after entering RX mode before reading RSSI (see swra114d.pdf) */
    __delay_cycles(MCLK_SPEED / 2000);    /* wait 0.5 ms */
    g.rssi_noise = rf1a_get_rssi();       /* get RSSI of the noise floor */
    LED_TOGGLE(FLOCKLAB_LED3);
  }
}
/*---------------------------------------------------------------------------*/
uint8_t
glossy_stop(void)
{
  if(g.active) {
    GLOSSY_STOPPED;
    /* stop the timeout */
    rtimer_stop(GLOSSY_CONF_RTIMER_ID);
    /* flush both RX FIFO and TX FIFO and go to sleep */
    rf1a_flush_rx_fifo();
    rf1a_flush_tx_fifo();
    /* important: if the radio is put into sleep mode, the patable must be 
     * re-configured! see CC1101 datasheet p.33 */
    rf1a_go_to_sleep();
    GLOSSY_RX_STOPPED;
    GLOSSY_TX_STOPPED;
    g.active = 0;

    if(g.t_ref_updated) {
      if(g.n_T_slot > 0) {
        g.t_ref -= (g.relay_cnt_t_ref * g.T_slot_sum) / g.n_T_slot;
      } else {
        g.t_ref -= g.relay_cnt_t_ref * g.T_slot_estimated;
      }
    }

    if(g.n_rx > 0) {
      DEBUG_PRINT_VERBOSE("Glossy stopped: in=%u, pl=%u, n=%u, s=%u, rc_rx=%u,"
                          " rc_tx=%u",
                          g.header.initiator_id, g.payload_len,
                          GET_N_TX_MAX(g.header.pkt_type),
                          GET_SYNC(g.header.pkt_type), g.relay_cnt_last_rx,
                          g.relay_cnt_last_tx);
      DEBUG_PRINT_VERBOSE("Glossy n_Ts=%u, rc_tref=%u, Ts=%llu, tref=%llu, "
                          "Ts_est=%llu", g.n_T_slot, g.relay_cnt_t_ref,
                          (g.n_T_slot > 0) ? (g.T_slot_sum / g.n_T_slot) : 0,
                          g.t_ref, g.T_slot_estimated);
    } else {
      DEBUG_PRINT_VERBOSE("Glossy stopped");
    }

    /* re-enable interrupts */
    GLOSSY_ENABLE_INTERRUPTS;
  }

  return g.n_rx;
}
/*---------------------------------------------------------------------------*/
uint8_t
glossy_is_active(void)
{
  return g.active;
}
/*---------------------------------------------------------------------------*/
uint8_t
glossy_get_n_rx(void)
{
  return g.n_rx;
}
/*---------------------------------------------------------------------------*/
int8_t
glossy_get_snr(void)
{
  int16_t rssi_avg = g.rssi_sum / (int16_t)g.n_rx;
  if(rssi_avg == 0) {
      return 0;
  }
  return (int8_t)( - g.rssi_noise);
}
/*---------------------------------------------------------------------------*/
uint8_t
glossy_get_n_tx(void)
{
  return g.n_tx;
}
/*---------------------------------------------------------------------------*/
uint8_t
glossy_get_payload_len(void)
{
  return g.payload_len;
}
/*---------------------------------------------------------------------------*/
uint8_t
glossy_is_t_ref_updated(void)
{
  return g.t_ref_updated;
}
/*---------------------------------------------------------------------------*/
rtimer_clock_t
glossy_get_t_ref(void)
{
  return g.t_ref;
}
/*---------------------------------------------------------------------------*/
uint8_t
glossy_get_relay_cnt_first_rx(void)
{
  return g.relay_cnt_first_rx;
}
/*---------------------------------------------------------------------------*/
uint8_t
glossy_get_per(void)
{
  if(g.pkt_cnt > 0) {
    return g.corrupted_pkt_cnt * 100 / g.pkt_cnt;
  }
  return 0;
}
/*---------------------- RF1A callback implementation -----------------------*/
void
rf1a_cb_rx_started(rtimer_clock_t *timestamp)
{
  GLOSSY_RX_STARTED;
  /* notify about the beginning of the reception */
  DEBUG_PRINT_VERBOSE("Glossy RX started");

  /* disable timer overflow / update interrupt (required before every RX!) */
  rtimer_update_enable(0);

  g.t_rx_start = *timestamp;
  g.header_ok = 0;
  g.pkt_cnt++;

  if(IS_INITIATOR()) {
    /* we are the initiator and we have started a packet reception: stop the
     * timeout */
    rtimer_stop(GLOSSY_CONF_RTIMER_ID);
  }
}
/*---------------------------------------------------------------------------*/
void
rf1a_cb_tx_started(rtimer_clock_t *timestamp)
{
  GLOSSY_TX_STARTED;
  /* notify about the beginning of the transmission */
  DEBUG_PRINT_VERBOSE("Glossy TX started");

  g.t_tx_start = *timestamp;

  if(g.n_tx == 0) {
    /* first transmission: estimate the slot length based on the packet
     * length */
    g.T_slot_estimated = estimate_T_slot(GLOSSY_HEADER_LEN(g.header.pkt_type) +
                                         g.payload_len);
  }
}
/*---------------------------------------------------------------------------*/
void
rf1a_cb_header_received(rtimer_clock_t *timestamp, uint8_t *header,
                        uint8_t packet_len)
{
  if(process_glossy_header(header, packet_len, 0) != SUCCESS) {
    /* the header is not ok: interrupt the reception and start a new attempt */
    rf1a_cb_rx_failed(timestamp);
  }
}
/*---------------------------------------------------------------------------*/
void
rf1a_cb_rx_ended(rtimer_clock_t *timestamp, uint8_t *pkt, uint8_t pkt_len)
{
  GLOSSY_RX_STOPPED;
  /* enable timer overflow / update interrupt */
  rtimer_update_enable(1);
  g.t_rx_stop = *timestamp;
  
  if((process_glossy_header(pkt, pkt_len, 1) == SUCCESS)) {
    /* we received a correct packet, and the header has been stored into
     * g.header */
    uint8_t *payload = pkt + GLOSSY_HEADER_LEN(g.header.pkt_type);

    if(WITH_RELAY_CNT()) {
      /* the relay counter is part of the header */
      if(g.n_rx == 0) {
        /* store the relay counter corresponding to the first reception */
        g.relay_cnt_first_rx = g.header.relay_cnt;
      }
      /* increment the relay counter */
      g.header.relay_cnt++;
    }

    if((GET_N_TX_MAX(g.header.pkt_type) == 0) ||
       (g.n_tx < GET_N_TX_MAX(g.header.pkt_type))) {
      /* if n_tx_max is either unknown or not yet reached, transmit the
       * packet */
      rf1a_write_to_tx_fifo((uint8_t *)&g.header,
                            GLOSSY_HEADER_LEN(g.header.pkt_type),
                            payload, g.payload_len);
    } else {
      /* otherwise, stop Glossy */
      glossy_stop();
    }

    /* get the RSSI value */
    g.rssi_sum += rf1a_get_last_packet_rssi();
    
    /* increment the reception counter */
    g.n_rx++;

    if((!IS_INITIATOR()) && (g.n_rx == 1)) {
      /* we are a receiver and this was our first packet reception: */
      /* store the payload for the application */
      memcpy((uint8_t *)g.payload, payload, g.payload_len);
    }

    if(WITH_SYNC()) {
      /* store the relay counter of this last reception */
      g.relay_cnt_last_rx = g.header.relay_cnt - 1;

      if(g.t_ref_updated == 0) {
        /* t_ref has not been updated yet: update it */
        update_t_ref(g.t_rx_start - NS_TO_RTIMER_HF(TAU1),
                     g.header.relay_cnt - 1);
      }

      if((g.relay_cnt_last_rx == g.relay_cnt_last_tx + 1) &&
         (g.n_tx > 0)) {
        /* this reception immediately followed a transmission: measure
         * T_slot */
        add_T_slot_measurement(g.t_rx_start - g.t_tx_start -
                               NS_TO_RTIMER_HF(TAU1));
      }
    }

    /* notify about the successful reception */
    DEBUG_PRINT_VERBOSE("Glossy RX completed. Received a %u-byte packet with "
                        "initiator %u.",
                        pkt_len, g.header.initiator_id);
  } else {
    /* some fields in the header were not correct: discard it */
    rf1a_cb_rx_failed(timestamp);
  }
}
/*---------------------------------------------------------------------------*/
void
rf1a_cb_tx_ended(rtimer_clock_t *timestamp)
{
  GLOSSY_TX_STOPPED;
  /* notify about the successful transmission */
  DEBUG_PRINT_VERBOSE("Glossy TX completed");

  g.t_tx_stop = *timestamp;

  if(WITH_SYNC()) {
    /* store the relay counter of this last transmission */
    g.relay_cnt_last_tx = g.header.relay_cnt;

    if(g.t_ref_updated == 0) {
      /* t_ref has not been updated yet: update it */
      update_t_ref(g.t_tx_start, g.header.relay_cnt);
    }

    if((g.relay_cnt_last_tx == g.relay_cnt_last_rx + 1) && (g.n_rx > 0)) {
      /* this transmission immediately followed a reception: measure T_slot */
      add_T_slot_measurement(g.t_tx_start - g.t_rx_start +
                             NS_TO_RTIMER_HF(TAU1));
    }
  }

  /* increment the transmission counter */
  g.n_tx++;

  if((g.n_tx == GET_N_TX_MAX(g.header.pkt_type)) &&
     (GET_N_TX_MAX(g.header.pkt_type) > 1 || (!IS_INITIATOR()))) {
    /* we have reached N_tx_max and either N_tx_max > 1 or we are a receiver:
     * stop Glossy */
    glossy_stop();
  } else {
    if((IS_INITIATOR()) && (g.n_rx == 0)) {
      /* we are the initiator and we still have not received any packet:
       * schedule the timeout */
      schedule_timeout();
    }
  }     
}
/*---------------------------------------------------------------------------*/
void
rf1a_cb_rx_failed(rtimer_clock_t *timestamp)
{
  GLOSSY_RX_STOPPED;
  /* notify about the failure, flush the RX FIFO and start a new reception
   * attempt */
  DEBUG_PRINT_VERBOSE("Glossy RX failed, corrupted packet received");
  g.corrupted_pkt_cnt++;
  rtimer_update_enable(1);
  rf1a_flush_rx_fifo();
  rf1a_start_rx();
}
/*---------------------------------------------------------------------------*/
void
rf1a_cb_rx_tx_error(rtimer_clock_t *timestamp)
{
  GLOSSY_RX_STOPPED;
  GLOSSY_TX_STOPPED;
  /* notify about the error */
  DEBUG_PRINT_VERBOSE("Glossy RX/TX error (interference?)");

  rtimer_update_enable(1);

  if(g.active) {
    /* if Glossy is still active, flush both RX FIFO and TX FIFO and start a
     * new reception attempt */
    rf1a_flush_rx_fifo();
    rf1a_flush_tx_fifo();
    rf1a_start_rx();
  }
}
/*---------------------------------------------------------------------------*/
