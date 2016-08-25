/*
 * Copyright (c) 2016, Swiss Federal Institute of Technology (ETH Zurich).
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
 * Author:  Reto Da Forno
 */

/* application code for the source node */

#include "main.h"

/*---------------------------------------------------------------------------*/
/* send a data packet to all sinks */
void
send_pkt(const uint8_t* data,
         uint8_t len,
         message_type_t type)
{
#define LWB_STREAM_ID_STATUS_MSG        1

  /* TODO use different stream IDs for different message types */

  static uint16_t seq_no = 0;

  message_t msg;
  msg.header.device_id   = node_id;
  msg.header.type        = type;
  msg.header.payload_len = len;
  msg.header.seqnr       = seq_no++;
  MSG_SET_CRC16(&msg, crc16(data, len, 0));

  if(data) {
    memcpy(msg.payload, data, len);
  }
  if(!lwb_send_pkt(LWB_RECIPIENT_SINKS, LWB_STREAM_ID_STATUS_MSG,
                   (uint8_t*)&msg, msg.header.payload_len + MSG_HDR_LEN)) {
    DEBUG_PRINT_WARNING("message dropped (queue full)");
  } else {
    DEBUG_PRINT_INFO("message added to TX queue");
  }
}
/*---------------------------------------------------------------------------*/
uint8_t
get_node_health(cc430_health_t* out_data)
{
  static int16_t          temp = 0;
  static rtimer_clock_t   last_energest_rst = 0;
  static uint16_t         last_rx_drop = 0,
                          last_tx_drop = 0;
  const lwb_statistics_t* lwb_stats = lwb_get_stats();

  while(REFCTL0 & REFGENBUSY);
  REFCTL0 |= REFON;
  while(REFCTL0 & REFGENBUSY);
  __delay_cycles(MCLK_SPEED / 25000);                /* let REF settle */

  temp = (temp + adc_get_temp()) / 2;      /* moving average (LP filter) */
  out_data->temp = temp;
  out_data->vcc  = adc_get_vcc();
  REFCTL0 &= ~REFON;             /* shut down REF module to save power */

  glossy_get_rssi(out_data->lwb_rssi);
  out_data->rf_snr        = glossy_get_snr();
  out_data->lwb_rx_cnt    = glossy_get_n_pkts_crcok();
  out_data->rf_per        = glossy_get_per();
  out_data->lwb_n_rx_hops = glossy_get_n_rx() |
                            (glossy_get_relay_cnt() << 4);
  out_data->lwb_fsr       = glossy_get_success_rate();
  rtimer_clock_t now      = rtimer_now_lf();
  out_data->cpu_dc        = (uint16_t)
                            (energest_type_time(ENERGEST_TYPE_CPU) *
                            1000 / (now - last_energest_rst));
  out_data->rf_dc         = (uint16_t)
                            ((energest_type_time(ENERGEST_TYPE_TRANSMIT) +
                            energest_type_time(ENERGEST_TYPE_LISTEN)) *
                            1000 / (now - last_energest_rst));
  out_data->lwb_tx_buf    = lwb_send_buffer_state();
  out_data->lwb_rx_buf    = lwb_rcv_buffer_state();
  out_data->lwb_tx_drop   = lwb_stats->txbuf_drop - last_tx_drop;
  out_data->lwb_rx_drop   = lwb_stats->rxbuf_drop - last_rx_drop;
  out_data->lfxt_ticks    = rtimer_now_lf();
  out_data->uptime        = rtimer_now_lf() / XT1CLK_SPEED;


  /* calculate the timestamp */
  rtimer_clock_t round_start;
  uint64_t       lwb_time_seconds = lwb_get_time(&round_start);
  /* convert to microseconds */
  round_start = (rtimer_now_hf() - round_start) * 1000000 / SMCLK_SPEED;
  out_data->generation_time = lwb_time_seconds * 1000000 +
                                        round_start;

  /* reset values */
  last_energest_rst  = now;
  energest_type_set(ENERGEST_TYPE_CPU, 0);
  energest_type_set(ENERGEST_TYPE_TRANSMIT, 0);
  energest_type_set(ENERGEST_TYPE_LISTEN, 0);
  last_rx_drop = lwb_stats->rxbuf_drop;
  last_tx_drop = lwb_stats->txbuf_drop;

  return (sizeof(cc430_health_t) + MSG_HDR_LEN);
}
/*---------------------------------------------------------------------------*/
void
source_init(void)
{
#if LWB_CONF_USE_LF_FOR_WAKEUP
  SVS_DISABLE;
#endif /* LWB_CONF_USE_LF_FOR_WAKEUP */
  /* all other necessary initialization is done in contiki-cc430-main.c */

  /* init the ADC */
  adc_init();
  REFCTL0 &= ~REFON;             /* shut down REF module to save power */

 #if DEBUG_PORT2_INT
  /* enable ISR for debugging! */
  PIN_IES_RISING(BOLT_CONF_IND_PIN);
  PIN_CLR_IFG(BOLT_CONF_IND_PIN);
  PIN_INT_EN(BOLT_CONF_IND_PIN);
 #endif /* DEBUG_PORT2_INT */
}
/*---------------------------------------------------------------------------*/
void
source_run(void)
{
#if SEND_HEALTH_DATA
  static uint8_t stream_state = 0;
  if(stream_state != LWB_STREAM_STATE_ACTIVE) {
	  stream_state = lwb_stream_get_state(1);
	  if(stream_state == LWB_STREAM_STATE_INACTIVE) {
      /* request a stream with ID LWB_STREAM_ID_STATUS_MSG and an IPI
       * (inter packet interval) of LWB_CONF_SCHED_PERIOD_IDLE seconds */
      lwb_stream_req_t my_stream = { node_id, 0, LWB_STREAM_ID_STATUS_MSG,
                                     LWB_CONF_SCHED_PERIOD_IDLE };
      if(!lwb_request_stream(&my_stream, 0)) {
        DEBUG_PRINT_ERROR("stream request failed");
      }
    }
  } else {
    /* generate a node health packet and schedule it for transmission */
    cc430_health_t node_health;
    if(get_node_health(&node_health)) {
       send_pkt((const uint8_t*)&node_health, sizeof(cc430_health_t),
                MSG_TYPE_CC430_HEALTH);
    }

    /* is there a packet to read? */
    message_t msg_buffer;
    uint8_t   pkt_len = 0;
    do {
      uint8_t pkt_len = lwb_rcv_pkt((uint8_t*)&msg_buffer, 0, 0);
      if(pkt_len) {
        DEBUG_PRINT_INFO("packet received (%ub)", pkt_len);
        if(msg_buffer.header.type == MSG_TYPE_LWB_CMD &&
           msg_buffer.payload[0] == LWB_CMD_SET_STATUS_PERIOD) {
          // change health/status report interval
          lwb_stream_req_t my_stream = { node_id, 0,
                         LWB_STREAM_ID_STATUS_MSG,
                         msg_buffer.payload[2] };
          lwb_request_stream(&my_stream, 0);
        }
      }
    } while (pkt_len);
  }
#endif /* SEND_HEALTH_DATA */
}
/*---------------------------------------------------------------------------*/
