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
 *          Tonio Gsell
 */

/* application code for the source node */

#include "main.h"

/*---------------------------------------------------------------------------*/

#define LWB_STREAM_ID_HEALTH_MSG    1

/*---------------------------------------------------------------------------*/
/* send a data packet to all sinks */
uint8_t
send_pkt(const uint8_t* data,
         uint8_t len,
         message_type_t type)
{
  /* TODO use different stream IDs for different message types */

  static uint16_t seq_no = 0;

  if(!data || !len) {
    return 0;
  }

  message_t msg;
  msg.header.device_id   = node_id;
  msg.header.type        = type;
  msg.header.payload_len = len;
  msg.header.target_id   = DEVICE_ID_SINK;
  msg.header.seqnr       = seq_no++;

  /* calculate the timestamp, convert to microseconds */
  rtimer_clock_t round_start;
  uint64_t       lwb_time_seconds = lwb_get_time(&round_start);
  round_start = (rtimer_now_hf() - round_start) * 1000000 / SMCLK_SPEED;
  msg.header.generation_time = lwb_time_seconds * 1000000 + round_start;

  /* copy the payload */
  memcpy(msg.payload, data, len);
  uint16_t crc = crc16((uint8_t*)&msg, len + MSG_HDR_LEN, 0);
  MSG_SET_CRC16(&msg, crc);

  if(!lwb_send_pkt(LWB_RECIPIENT_SINK, LWB_STREAM_ID_HEALTH_MSG,
                   (uint8_t*)&msg, MSG_LEN(msg))) {
    DEBUG_PRINT_WARNING("message dropped (queue full)");
    return 0;
  }
  DEBUG_PRINT_INFO("message added to TX queue");
  return 1;
}
/*---------------------------------------------------------------------------*/
uint8_t
get_node_health(comm_health_t* out_data)
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

  rtimer_clock_t now      = rtimer_now_lf();

  glossy_get_rssi(out_data->lwb_rssi);
  out_data->rf_snr        = glossy_get_snr();
  out_data->lwb_rx_cnt    = glossy_get_n_pkts_crcok();
  out_data->rf_per        = glossy_get_per();
  out_data->lwb_n_rx_hops = glossy_get_n_rx() |
                            (glossy_get_relay_cnt() << 4);
  out_data->lwb_fsr       = glossy_get_fsr();
  out_data->cpu_dc        = (uint16_t)
                            (energest_type_time(ENERGEST_TYPE_CPU) *
                            1000 / (now - last_energest_rst));
  out_data->rf_dc         = (uint16_t)
                            ((energest_type_time(ENERGEST_TYPE_TRANSMIT) +
                            energest_type_time(ENERGEST_TYPE_LISTEN)) *
                            1000 / (now - last_energest_rst));
  out_data->lwb_tx_buf    = lwb_tx_buffer_state();
  out_data->lwb_rx_buf    = lwb_rx_buffer_state();
  out_data->lwb_tx_drop   = lwb_stats->txbuf_drop - last_tx_drop;
  out_data->lwb_rx_drop   = lwb_stats->rxbuf_drop - last_rx_drop;
  out_data->lwb_sleep_cnt = lwb_stats->sleep_cnt;
  out_data->lwb_bootstrap_cnt = lwb_stats->bootstrap_cnt;
  //out_data->lfxt_ticks    = now;
  out_data->uptime        = now / XT1CLK_SPEED;
  out_data->lwb_n_rx_started = glossy_get_n_rx_started();
  out_data->lwb_t_flood   = (uint16_t)(glossy_get_flood_duration() * 100 /325);
  out_data->lwb_t_to_rx   = (uint16_t)(glossy_get_t_to_first_rx() * 100 / 325);

  /* reset values */
  last_energest_rst  = now;
  energest_type_set(ENERGEST_TYPE_CPU, 0);
  energest_type_set(ENERGEST_TYPE_TRANSMIT, 0);
  energest_type_set(ENERGEST_TYPE_LISTEN, 0);
  last_rx_drop = lwb_stats->rxbuf_drop;
  last_tx_drop = lwb_stats->txbuf_drop;

  return (sizeof(comm_health_t) + MSG_HDR_LEN);
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
}
/*---------------------------------------------------------------------------*/
void
source_run(void)
{
#if SEND_HEALTH_DATA
  static lwb_stream_req_t health_stream = { node_id, 0,
                                            LWB_STREAM_ID_HEALTH_MSG,
                                            LWB_CONF_SCHED_PERIOD_IDLE };
  static uint32_t t_last_health_pkt = 0;
  static uint32_t curr_time         = 0;
  static uint16_t health_period     = LWB_CONF_SCHED_PERIOD_IDLE;
  static uint8_t  ipi_changed       = 1;

  curr_time = lwb_get_time(0);

  /* adjust the IPI in case the fill level of the output queue reaches a
   * certain threshold */
  if(health_stream.ipi == health_period) {
    if(lwb_tx_buffer_state() > (LWB_CONF_OUT_BUFFER_SIZE / 2)) {
      health_stream.ipi = health_period / 2; /* reduce the IPI */
      ipi_changed = 1;
    }
  } else if(lwb_tx_buffer_state() < 2) {
    /* only 0 or 1 element left in the queue -> set IPI back to default */
    health_stream.ipi = health_period;
    ipi_changed = 1;
  }

  /* only send data if the stream is active */
  if(lwb_stream_get_state(LWB_STREAM_ID_HEALTH_MSG) ==
     LWB_STREAM_STATE_ACTIVE) {
    if((curr_time - t_last_health_pkt) >= health_period) {
      /* generate a node health packet and schedule it for transmission */
      comm_health_t node_health;
      if(get_node_health(&node_health)) {
        if(!send_pkt((const uint8_t*)&node_health, sizeof(comm_health_t),
                     MSG_TYPE_COMM_HEALTH)) {
          DEBUG_PRINT_INFO("failed to send msg, tx queue full");
        } else {
          DEBUG_PRINT_INFO("health message added to tx queue");
        }
      }
      t_last_health_pkt = curr_time;
    }
  }

  /* is there a packet to read? */
  message_t msg_buffer;
  uint8_t   count = 0;
  while(lwb_rcv_pkt((uint8_t*)&msg_buffer, 0, 0)) {
    count++;
    if(msg_buffer.header.target_id == node_id ||
       msg_buffer.header.target_id == DEVICE_ID_BROADCAST) {
      if(msg_buffer.header.type == MSG_TYPE_COMM_CMD) {
        switch(msg_buffer.comm_cmd.type) {
        case COMM_CMD_LWB_SET_HEALTH_PERIOD:
          /* change health/status report interval */
          health_period = msg_buffer.comm_cmd.value;
          health_stream.ipi = msg_buffer.comm_cmd.value;
          ipi_changed = 1;
          break;
        case COMM_CMD_LWB_SET_TX_PWR:
          glossy_set_tx_pwr(msg_buffer.comm_cmd.value);
          break;
        default:
          /* unknown command */
          break;
        }
      } /* else: unknown message type */
    } /* else: target ID does not match node ID */
  }
  if(count) {
    DEBUG_PRINT_INFO("%d packet(s) received", count);
  }

  if(ipi_changed) {
    if(!lwb_request_stream(&health_stream, 0)) {
      DEBUG_PRINT_ERROR("stream request failed");
    }
    ipi_changed = 0;
  }
#endif /* SEND_HEALTH_DATA */
}
/*---------------------------------------------------------------------------*/
