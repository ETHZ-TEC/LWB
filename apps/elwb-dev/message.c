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

/* functions related to DPP messages */

#include "main.h"


uint64_t utc_time = 0;
uint64_t utc_time_rx = 0;
uint16_t health_msg_period = HEALTH_MSG_PERIOD;


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
         dpp_message_type_t type,
         const uint8_t* data,
         uint8_t len,
         uint8_t send_to_bolt)
{
  /* compose the message header */
  dpp_message_t msg;
  msg.header.device_id   = node_id;
  msg.header.type        = type;
  if(!len) {
    switch(type) {
    case DPP_MSG_TYPE_COM_HEALTH:
      msg.header.payload_len = sizeof(dpp_com_health_t); break;
    case DPP_MSG_TYPE_CMD:
      msg.header.payload_len = 6; break;  /* default is 6 bytes */
    case DPP_MSG_TYPE_NODE_INFO:
      msg.header.payload_len = sizeof(dpp_node_info_t); break;
    case DPP_MSG_TYPE_TIMESYNC:
      msg.header.payload_len = sizeof(dpp_timestamp_t); break;
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
  if (msg.header.payload_len && data) {
    memcpy(msg.payload, data, msg.header.payload_len);
  }
  /* calculate and append the CRC */
  uint16_t msg_len = DPP_MSG_LEN(&msg);
  uint16_t crc = crc16((uint8_t*)&msg, msg_len - 2, 0);
  DPP_MSG_SET_CRC16(&msg, crc);

  /* forward the message either to BOLT or the LWB */
  if(send_to_bolt) {
    bolt_write((uint8_t*)&msg, msg_len);
    DEBUG_PRINT_VERBOSE("msg written to BOLT");
  } else {
    if(!lwb_send_pkt(recipient, 1, (uint8_t*)&msg, msg_len)) {
      DEBUG_PRINT_INFO("msg dropped (queue full)");
    } else {
      DEBUG_PRINT_INFO("msg for node %u queued for TX", recipient);
    }
  }
}
/*---------------------------------------------------------------------------*/ 
void
process_message(dpp_message_t* msg, uint8_t rcvd_from_bolt)
{
  /* check message type */
  if(msg->header.type & DPP_MSG_TYPE_MIN) {
    DEBUG_PRINT_WARNING("unsupported message type (dropped)");
    EVENT_WARNING(EVENT_CC430_INV_MSG, msg->header.type);
    return;
  }
  /* check message length and CRC */
  uint16_t msg_len = DPP_MSG_LEN(msg);
  if(msg_len > DPP_MSG_PKT_LEN || 
    DPP_MSG_GET_CRC16(msg) != crc16((uint8_t*)msg, msg_len - 2, 0)) {
    DEBUG_PRINT_WARNING("message with invalid length or CRC");
    EVENT_WARNING(EVENT_CC430_INV_MSG, 0);
    return;
  }
  DEBUG_PRINT_VERBOSE("processing msg (type: %u src: %u target: %u len: %uB)", 
                      msg->header.type, msg->header.device_id, 
                      msg->header.target_id, msg_len);
  
  /* only process the message if target ID matched the node ID */
  uint16_t forward = (msg->header.target_id == DPP_DEVICE_ID_BROADCAST);
  if(msg->header.target_id == node_id || forward) {
    if(msg->header.type == DPP_MSG_TYPE_CMD) {
      DEBUG_PRINT_VERBOSE("command received");
      uint8_t successful = 0;
      switch(msg->cmd.type) {
      case CMD_CC430_SET_ROUND_PERIOD:    /* value is in s */
        if(msg->cmd.arg16[0] > 0 && msg->cmd.arg16[0] <= 3600) {
          lwb_sched_set_period(msg->cmd.arg16[0]);
          DEBUG_PRINT_INFO("LWB period set to %us", msg->cmd.arg16[0]);
          successful = 1;
        }
        break;
      case CMD_CC430_SET_HEALTH_PERIOD:
        health_msg_period = msg->cmd.arg16[0]; /* set health period in s */
        successful = 1;
        break;
      case CMD_CC430_SET_EVENT_LEVEL:
        if (msg->cmd.arg16[0] < NUM_EVENT_LEVELS) {
          event_lvl = msg->cmd.arg16[0];
          successful = 1;
        }
        break;   
      case CMD_CC430_RESET:
        PMM_TRIGGER_BOR;
        break;
      default:
        /* unknown command */
        break;
      }
      uint32_t arg = (((uint32_t)msg->cmd.arg16[0]) << 16 | msg->cmd.type);
      if (successful) {
        DEBUG_PRINT_INFO("cmd processed, config changed");
        EVENT_INFO(EVENT_CC430_CFG_CHANGED, arg);
      } else {
        /* either unknown command or invalid argument */
        /*EVENT_WARNING(EVENT_CC430_INV_CMD, arg);
        DEBUG_PRINT_WARNING(
                      "unknown command or invalid parameter (0x%02x, 0x%02x)", 
                      msg->cmd.type, msg->cmd.arg16[0]);*/
        forward = 1;  /* forward to BOLT */
      }
#if TIMESYNC_HOST_RCV_UTC
    } else if (msg->header.type == DPP_MSG_TYPE_TIMESYNC) {
      if(node_id == HOST_ID) {
        utc_time    = msg->timestamp;
        utc_time_rx = bolt_captured_trq;
      } /* else: a source node does not handle this message type */
#endif /* TIMESYNC_HOST_RCV_UTC */

    /* unknown message type */
    } else {
      if (node_id != HOST_ID) {
        forward = 1;
      } else {
        /* ignore */      
        EVENT_WARNING(EVENT_CC430_MSG_IGNORED, msg->header.type);
      }
    }
  }
  /* forward the message */
  if(forward) {
    uint32_t arg = (msg->header.target_id | ((((uint32_t)msg_len) << 16) &
                    0x00ff0000));
    if(rcvd_from_bolt) {
      /* forward to network */
      /* queue is guaranteed to have at least one free spot (we check this 
        * before entering the wile loop */
      lwb_send_pkt(0, 1, (uint8_t*)msg, msg_len);
      DEBUG_PRINT_INFO("BOLT msg forwarded to network (target: %u)",
                       msg->header.target_id);
      arg |= 0x02000000;
    } else {
      /* forward to BOLT */      
      bolt_write((uint8_t*)msg, msg_len);
      DEBUG_PRINT_INFO("msg forwarded to BOLT (%uB)", msg_len);
      arg |= 0x01000000;
    }
    EVENT_VERBOSE(EVENT_CC430_MSG_FORWARDED, arg);
  }
}
/*---------------------------------------------------------------------------*/
void
send_timestamp(int64_t captured)
{ 
  dpp_timestamp_t timestamp = 0;
  
  /* timestamp request: calculate the timestamp and send it over BOLT */
  if(node_id == HOST_ID) {
    timestamp = captured * 1000000 / RTIMER_SECOND_LF + TIMESYNC_OFS;

  /* only send a timestamp if the node is connected to the LWB */
  } else {
    rtimer_clock_t local_t_rx = 0;
    uint64_t lwb_time_secs = lwb_get_time(&local_t_rx);
    if (lwb_time_secs > 0) {
      /* convert to LF ticks */
      rtimer_clock_t lf, hf;
      rtimer_now(&hf, &lf);
      local_t_rx = lf - (hf - local_t_rx) * RTIMER_SECOND_LF /RTIMER_SECOND_HF;
      /* if captured before last ref time update, then diff is > 0 and thus 
       * LWB time needs to be decreased by the difference */
      int64_t diff = (local_t_rx - captured);   /* in clock ticks */
      /* local t_rx is in clock ticks */
      timestamp = lwb_time_secs * 1000000 - diff * 1000000 / RTIMER_SECOND_LF;
    }
  }
  /* send message over BOLT */
  send_msg(node_id, DPP_MSG_TYPE_TIMESYNC, (uint8_t*)&timestamp, 0, 1);
  
  DEBUG_PRINT_INFO("timestamp sent: %llu", timestamp);
  captured = 0;
}
/*---------------------------------------------------------------------------*/
void
send_node_info(void)
{
  dpp_node_info_t node_info;
  memset((uint8_t*)&node_info, 0, sizeof(dpp_node_info_t));
  node_info.component_id = COMPONENT_ID;
  node_info.compiler_ver = COMPILER_VERSION_32;
  node_info.compile_date = COMPILE_TIME;          /* defined in makefile */
  node_info.fw_ver       = FW_VERSION;
  node_info.rst_cnt      = cfg.rst_cnt;
  node_info.rst_flag     = rst_flag;
  node_info.sw_rev_id    = hexstr_to_dec(GIT_HEADREV_SHA, 8);
  memcpy(node_info.compiler_desc, COMPILER_DESC, MIN(4,strlen(COMPILER_DESC)));
  memcpy(node_info.fw_name, FW_NAME, MIN(8, strlen(FW_NAME)));
  memcpy(node_info.mcu_desc, MCU_DESC, MIN(12, strlen(MCU_DESC)));
    
  send_msg(DPP_DEVICE_ID_SINK, DPP_MSG_TYPE_NODE_INFO, (uint8_t*)&node_info, 0,
           HOST_ID == node_id);   /* host sends message towards BOLT */
  DEBUG_PRINT_INFO("node info message generated");
}
/*---------------------------------------------------------------------------*/
void
send_node_health(void)
{
  dpp_com_health_t health_data;
  
  static int16_t          temp = 0;
#if ENERGEST_CONF_ON
  static rtimer_clock_t   last_energest_rst = 0;
#endif /* ENERGEST_CONF_ON */
  const lwb_statistics_t* lwb_stats = lwb_get_stats();

  while(REFCTL0 & REFGENBUSY);
  REFCTL0 |= REFON;
  while(REFCTL0 & REFGENBUSY);
  __delay_cycles(MCLK_SPEED / 25000);      /* let REF settle */

  temp = (temp + adc_get_temp()) / 2;      /* moving average (LP filter) */
  health_data.temp = temp;
  health_data.vcc  = adc_get_vcc();
  REFCTL0 &= ~REFON;             /* shut down REF module to save power */
  
  health_data.stack         = debug_print_get_max_stack_size();
  rtimer_clock_t now        = rtimer_now_lf();

  glossy_get_rssi(health_data.lwb_rssi);
  health_data.rf_snr        = glossy_get_snr();
  health_data.lwb_rx_cnt    = glossy_get_n_pkts_crcok();
  health_data.rf_per        = glossy_get_per();
  health_data.lwb_n_rx_hops = glossy_get_n_rx() |
                              (glossy_get_relay_cnt() << 4);
  health_data.lwb_fsr       = glossy_get_fsr();
  health_data.lwb_tx_buf    = lwb_get_send_buffer_state();
  health_data.lwb_rx_buf    = lwb_get_rcv_buffer_state();
  health_data.lwb_tx_drop   = lwb_stats->txbuf_drop; // - last_tx_drop;
  health_data.lwb_rx_drop   = lwb_stats->rxbuf_drop; // - last_rx_drop;
  health_data.lwb_sleep_cnt = lwb_stats->sleep_cnt;
  health_data.lwb_bootstrap_cnt = lwb_stats->bootstrap_cnt;
  health_data.uptime        = now / RTIMER_SECOND_LF;   // convert to seconds
  health_data.lwb_t_flood   = (uint16_t)(glossy_get_flood_duration() *100/325);
  health_data.lwb_t_to_rx   = (uint16_t)(glossy_get_t_to_first_rx()  *100/325);
  
  /* test value (can be changed in the future for debug purposes) */
  health_data.test          = lwb_stats->drift;

  /* duty cycle */
#if ENERGEST_CONF_ON
  health_data.cpu_dc        = (uint16_t)
                              (energest_type_time(ENERGEST_TYPE_CPU) *
                              1000 / (now - last_energest_rst));
  health_data.rf_dc         = (uint16_t)
                              ((energest_type_time(ENERGEST_TYPE_TRANSMIT) +
                              energest_type_time(ENERGEST_TYPE_LISTEN)) *
                              1000 / (now - last_energest_rst));
  last_energest_rst  = now;
  energest_type_set(ENERGEST_TYPE_CPU, 0);
  energest_type_set(ENERGEST_TYPE_TRANSMIT, 0);
  energest_type_set(ENERGEST_TYPE_LISTEN, 0);
#else  /* ENERGEST_CONF_ON */
  DCSTAT_CPU_OFF;
  health_data.cpu_dc        = DCSTAT_CPU_DC;
  health_data.rf_dc         = DCSTAT_RF_DC;
  DCSTAT_RESET;
  DCSTAT_CPU_ON;
#endif /* ENERGEST_CONF_ON */
    
  /* host must send it to BOLT, a source node into the network */
  send_msg(DPP_DEVICE_ID_SINK, DPP_MSG_TYPE_COM_HEALTH,
           (const uint8_t*)&health_data, 0, node_id == HOST_ID); 
  
  DEBUG_PRINT_INFO("health msg generated");
}
/*---------------------------------------------------------------------------*/
