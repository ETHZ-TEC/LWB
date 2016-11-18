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

/* utility functions */

#include "main.h"

#ifndef MIN
#define MIN(x,y)    (((x) < (y)) ? (x) : (y))
#endif /* MIN */
/*---------------------------------------------------------------------------*/
/* convert an ASCII string of up to 8 hex characters to a decimal value */
uint32_t
hexstr_to_dec(const char* str, uint8_t num_chars)
{
  uint32_t res = 0;
  while(1) {
    if(*str >= 'A' && *str <= 'F') {
      res += *str - 'A' + 10;
    } else if(*str >= 'a' && *str <= 'f') {
      res += *str - 'a' + 10;
    } else if (*str >= '1' && *str <= '9') {
      res += *str - '0';
    }
    num_chars--;
    if(!num_chars) { break; }
    res <<= 4;  /* shift to the left by 4 bits */
    str++;
  }
  return res;
}
/*---------------------------------------------------------------------------*/
void
send_msg(uint16_t recipient,
         message_type_t type,
         const uint8_t* data,
         uint8_t len,
         uint8_t send_to_bolt)
{
  if(!data) {   /* parameter check */
    return;
  }
  /* compose the message header */
  message_t msg;
  msg.header.device_id   = node_id;
  msg.header.type        = type;
  if(!len) {
    switch(type) {
    case MSG_TYPE_COMM_HEALTH:
      msg.header.payload_len = sizeof(comm_health_t); break;
    case MSG_TYPE_COMM_CMD:
      msg.header.payload_len = sizeof(comm_cmd_t); break;
    case MSG_TYPE_NODE_INFO:
      msg.header.payload_len = sizeof(node_info_t); break;
    default:
      msg.header.payload_len = 0; break;
    }
  } else {
    msg.header.payload_len   = len;
  }
  msg.header.target_id       = recipient;
  if(send_to_bolt) {
    msg.header.seqnr         = seq_no_bolt++;
  } else {
    msg.header.seqnr         = seq_no_lwb++;
  }
  msg.header.generation_time = lwb_get_timestamp();
  /* copy the payload */
  memcpy(msg.payload, data, msg.header.payload_len);
  /* calculate and append the CRC */
  uint16_t crc = crc16((uint8_t*)&msg, msg.header.payload_len + MSG_HDR_LEN,0);
  MSG_SET_CRC16(&msg, crc);

  /* forward the message either to BOLT or the LWB */
  if(send_to_bolt) {
    BOLT_WRITE((uint8_t*)&msg, MSG_LEN(msg));
  } else {
    if(!lwb_send_pkt(recipient, STREAM_ID,
                     (uint8_t*)&msg, MSG_LEN(msg))) {
      DEBUG_PRINT_INFO("message dropped (queue full)");
    } else {
      DEBUG_PRINT_INFO("message for node %u added to TX queue", recipient);
    }
  }
}
/*---------------------------------------------------------------------------*/
void
get_node_health(comm_health_t* out_data)
{
  static int16_t          temp = 0;
  static rtimer_clock_t   last_energest_rst = 0;
  //static uint16_t         last_rx_drop = 0,
  //                        last_tx_drop = 0;
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
  out_data->lwb_tx_buf    = lwb_get_send_buffer_state();
  out_data->lwb_rx_buf    = lwb_get_rcv_buffer_state();
  out_data->lwb_tx_drop   = lwb_stats->txbuf_drop; // - last_tx_drop;
  out_data->lwb_rx_drop   = lwb_stats->rxbuf_drop; // - last_rx_drop;
  out_data->lwb_sleep_cnt = lwb_stats->sleep_cnt;
  out_data->lwb_bootstrap_cnt = lwb_stats->bootstrap_cnt;
  out_data->uptime        = now / XT1CLK_SPEED;
  out_data->lwb_n_rx_started = glossy_get_n_rx_started();
  out_data->lwb_t_flood   = (uint16_t)(glossy_get_flood_duration() * 100 /325);
  out_data->lwb_t_to_rx   = (uint16_t)(glossy_get_t_to_first_rx() * 100 / 325);
  /* packet delivery rate */
  out_data->reserved      = (lwb_stats->pkts_nack * 200) /lwb_stats->pkts_sent;

  /* reset values */
  last_energest_rst  = now;
  energest_type_set(ENERGEST_TYPE_CPU, 0);
  energest_type_set(ENERGEST_TYPE_TRANSMIT, 0);
  energest_type_set(ENERGEST_TYPE_LISTEN, 0);
  //last_rx_drop = lwb_stats->rxbuf_drop;
  //last_tx_drop = lwb_stats->txbuf_drop;
}
/*---------------------------------------------------------------------------*/
void
get_node_info(node_info_t* out_data)
{
  memset(out_data, 0, sizeof(node_info_t));     /* clear */
  out_data->component_id = COMPONENT_ID;
  out_data->compiler_ver = COMPILER_VERSION_ENC;  /* encoded 16-bit value */
  out_data->compile_date = COMPILE_TIME;          /* defined in makefile */
  out_data->fw_ver       = FW_VERSION;
  out_data->rst_cnt      = cfg.rst_cnt;
  out_data->rst_flag     = rst_flag;
  uint32_t rev_id = hexstr_to_dec(GIT_HEADREV_SHA, 6);
  memcpy(out_data->sw_rev_id, &rev_id, 3);
  memcpy(out_data->compiler_desc, COMPILER_DESC, MIN(3,strlen(COMPILER_DESC)));
  memcpy(out_data->fw_name, FW_NAME, MIN(8, strlen(FW_NAME)));
  memcpy(out_data->mcu_desc, MCU_DESC, MIN(12, strlen(MCU_DESC)));
}
/*---------------------------------------------------------------------------*/
