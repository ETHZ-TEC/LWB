/*
 * Copyright (c) 2018, Swiss Federal Institute of Technology (ETH Zurich).
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
uint8_t  utc_time_updated = 0;

/* reduce stack usage by utilizing only one global data structure for all 
 * DPP messages */
dpp_message_t msg_tx;   /* only used for sending! */

static uint16_t rcvd_msg_cnt = 0;

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
    } else if(*str >= '1' && *str <= '9') {
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
#if IS_HOST
uint64_t
utc_timestamp(void)
{
  return utc_time + ((rtimer_now_lf() - utc_time_rx) *
                     (1000000 / RTIMER_SECOND_LF));
}
#endif /* IS_HOST */
/*---------------------------------------------------------------------------*/
/* Do not call this function from an interrupt context!
 * Note: data may be 0, in that case the function will use the payload in 
 *       the global struct msg_tx                                            */
uint8_t
send_msg(uint16_t recipient,
         dpp_message_type_t type,
         const uint8_t* data,
         uint8_t len,
         uint8_t send_to_bolt)
{
  /* separate sequence number for each interface */
  static uint16_t seq_no_lwb  = 0;
  static uint16_t seq_no_bolt = 0;

  /* check message length */
  if(len > DPP_MSG_PAYLOAD_LEN) {
    DEBUG_PRINT_WARNING("invalid message length");
    EVENT_WARNING(EVENT_CC430_INV_MSG, ((uint32_t)type) << 16 | 0xff00 | len);
    return 0;
  }

  /* compose the message header */
  msg_tx.header.device_id   = node_id;
  msg_tx.header.type        = type;
  msg_tx.header.payload_len = len;
  if(!len) {
    switch(type) {
    case DPP_MSG_TYPE_COM_HEALTH:
      msg_tx.header.payload_len = sizeof(dpp_com_health_t); break;
    case DPP_MSG_TYPE_CMD:
      msg_tx.header.payload_len = 6; break;  /* default is 6 bytes */
    case DPP_MSG_TYPE_EVENT:
      msg_tx.header.payload_len = sizeof(dpp_event_t); break;
    case DPP_MSG_TYPE_NODE_INFO:
      msg_tx.header.payload_len = sizeof(dpp_node_info_t); break;
    case DPP_MSG_TYPE_TIMESYNC:
      msg_tx.header.payload_len = sizeof(dpp_timestamp_t); break;
    case DPP_MSG_TYPE_LWB_HEALTH:
      msg_tx.header.payload_len = sizeof(dpp_lwb_health_t); break;
    default:
      break;
    }
  }
  msg_tx.header.target_id = recipient;
  if(send_to_bolt) {
    msg_tx.header.seqnr   = seq_no_bolt++;
  } else {
    msg_tx.header.seqnr   = seq_no_lwb++;
  }
#if IS_HOST
  msg_tx.header.generation_time = utc_timestamp();
#else /* IS_HOST */
  msg_tx.header.generation_time = elwb_get_timestamp();
#endif /* IS_HOST */
  
  /* copy the payload if valid */
  if(msg_tx.header.payload_len && data) {
    memcpy(msg_tx.payload, data, msg_tx.header.payload_len);
  }
  /* calculate and append the CRC */
  uint16_t msg_tx_len = DPP_MSG_LEN(&msg_tx);
  uint16_t crc = crc16((uint8_t*)&msg_tx, msg_tx_len - 2, 0);
  DPP_MSG_SET_CRC16(&msg_tx, crc);

  /* forward the message either to BOLT or the eLWB */
  if(send_to_bolt) {
    if(bolt_write((uint8_t*)&msg_tx, msg_tx_len)) {
      DEBUG_PRINT_VERBOSE("msg written to BOLT");
      return 1;
    }
    DEBUG_PRINT_INFO("msg dropped (BOLT queue full)");
  } else {
    if(elwb_send_pkt((uint8_t*)&msg_tx, msg_tx_len)) {
      DEBUG_PRINT_VERBOSE("msg added to TX queue");
      return 1;
    }
    DEBUG_PRINT_INFO("msg dropped (TX queue full)");
  }
  return 0;
}
/*---------------------------------------------------------------------------*/ 
/* returns 1 if processed (or dropped), 0 if forwarded */
uint8_t
process_message(dpp_message_t* msg, uint8_t rcvd_from_bolt)
{
  /* check message type, length and CRC */
  uint16_t msg_len = DPP_MSG_LEN(msg);
  if(msg->header.type & DPP_MSG_TYPE_MIN ||
     msg_len > DPP_MSG_PKT_LEN ||
     msg_len < (DPP_MSG_HDR_LEN + 2) ||
     DPP_MSG_GET_CRC16(msg) != crc16((uint8_t*)msg, msg_len - 2, 0)) {
    DEBUG_PRINT_WARNING("msg with invalid length or CRC (%ub, type %u)",
                        msg_len, msg->header.type);
    EVENT_WARNING(EVENT_CC430_INV_MSG, ((uint32_t)msg_len) << 16 |
                                       msg->header.device_id);
    return 1;
  }
  DEBUG_PRINT_VERBOSE("msg type: %u, src: %u, len: %uB", 
                      msg->header.type, msg->header.device_id, msg_len);
  
  /* only process the message if target ID matched the node ID */
  uint16_t forward = (msg->header.target_id == DPP_DEVICE_ID_BROADCAST);
  uint8_t cfg_changed = 0;
  if(msg->header.target_id == node_id || forward) {
    rcvd_msg_cnt++;
    if(msg->header.type == DPP_MSG_TYPE_CMD) {
      DEBUG_PRINT_VERBOSE("command received");
      uint8_t  successful = 0;
      uint16_t arg1 = msg->cmd.arg16[0];
      uint16_t arg2 = msg->cmd.arg16[1];

      switch(msg->cmd.type) {
      case DPP_COMMAND_RESET:
      case CMD_CC430_RESET:
  #if IS_HOST
        if(!forward) {
          PMM_TRIGGER_POR;  /* host ignores RESET broadcast */
        }
  #else /* IS_HOST */
        PMM_TRIGGER_POR;
  #endif /* IS_HOST */
        break;
      case CMD_CC430_SET_ROUND_PERIOD:    /* value is in s */
        if(arg1 >= ELWB_CONF_SCHED_PERIOD_MIN && 
           arg1 <= ELWB_SCHED_PERIOD_MAX) {
          elwb_sched_set_period(arg1);
          DEBUG_PRINT_INFO("LWB period set to %us", arg1);
          successful = 1;
        }
        break;
      case CMD_CC430_SET_HEALTH_PERIOD:
        /* note: even if the health msg period is smaller than the eLWB round 
         * period, only one health packet will be generated per round! set
         * the health period to 0 to disable health messages */
        health_msg_period = arg1; /* health period in s */
        successful = 1;
        break;
      case CMD_CC430_SET_EVENT_LEVEL:
        if(arg1 < NUM_EVENT_LEVELS) {
          event_lvl = arg1;
          successful = 1;
        }
        break;
      case CMD_CC430_SET_TX_POWER:
        if(arg1 < N_TX_POWER_LEVELS) {
          if(cfg.tx_pwr != arg1) {
            cfg.tx_pwr = (uint8_t)arg1;
            cfg_changed = 1;
            rf1a_set_tx_power(arg1);
            DEBUG_PRINT_INFO("TX power changed to %ddBm", 
                             rf1a_tx_power_val[arg1]);
          }
          successful = 1;
        }
        break;
      case CMD_CC430_SET_DBG_FLAGS:
        if(cfg.dbg_flags != (uint8_t)arg1) {
          cfg.dbg_flags = (uint8_t)arg1;
          cfg_changed = 1;
        }
        successful = 1;
        break;
      case CMD_CC430_DBG_READ_MEM:
        /* try to read the requested memory location */
        if(arg2 > 0 && arg2 < 33) {
          uint16_t buf[16 + DPP_COM_RESPONSE_HDR_LEN / 2];
          buf[0] = CMD_CC430_DBG_READ_MEM;
          buf[1] = arg1;
          memcpy(&buf[2], (uint8_t*)arg1, arg2);
          send_msg(DPP_DEVICE_ID_SINK, DPP_MSG_TYPE_COM_RESPONSE, 
                   (uint8_t*)buf, arg2 + DPP_COM_RESPONSE_HDR_LEN, 
                   IS_HOST);
          successful = 1;
        }
        break;
  #if IS_HOST
      case CMD_CC430_ADD_NODE:
        if(arg1) {
          elwb_sched_process_req(arg1, 0);
          successful = 1;
        }
        break;
  #endif /* IS_HOST */
      default:
        /* unknown command */
        forward = 1;  /* forward to BOLT */
        break;
      }
      /* command successfully executed? */
      if(successful) {
        uint32_t val = (((uint32_t)arg1) << 16 | msg->cmd.type);
        DEBUG_PRINT_INFO("cmd %u processed", msg->cmd.type);
        EVENT_INFO(EVENT_CC430_CFG_CHANGED, val);
        /* if necessary, store the new config in the flash memory */
        if(cfg_changed) {
          if(!nvcfg_save(&cfg)) {
            EVENT_ERROR(EVENT_CC430_CORRUPTED_CONFIG, 2);
            DEBUG_PRINT_INFO("Failed to save config");
          }
        }
      }
  #if IS_HOST
    } else if(msg->header.type == DPP_MSG_TYPE_TIMESYNC) {
      utc_time    = msg->timestamp;
      utc_time_rx = bolt_captured_trq;
      utc_time_updated = 1;
  #endif /* IS_HOST */

    /* firmware message type (only continue if component ID matches) */
    } else if(msg->header.type == DPP_MSG_TYPE_FW &&
              msg->firmware.component_id == DPP_COMPONENT_ID_CC430) {
  #if FW_UPDATE_CONF_ON
    #if IS_HOST
      /* host node only processes this msg type if target == node_id */
      if(msg->header.target_id == node_id) {
        if(!fw_process_msg(msg)) {
          DEBUG_PRINT_INFO("failed to process FW msg type");
        }
        forward = 0;   /* do not forward FW packets that are for host nodes! */
      } /* else: just forward to the network */
    #else /* IS_HOST */
      if(!fw_process_msg(msg)) {
        DEBUG_PRINT_INFO("failed to process FW msg type");
      }
      forward = 0;  /* source node shall not forward this to the AP */
    #endif /* IS_HOST */
  #endif /* FW_UPDATE_CONF_ON */
      
    } else {
      /* unknown message type */
  #if !IS_HOST
        forward = 1;
  #else /* !IS_HOST */
        /* ignore */
        EVENT_WARNING(EVENT_CC430_MSG_IGNORED, msg->header.type);
  #endif /* !IS_HOST */
    }
  } else {
    /* target id is not this node */
    forward = 1;
  }
  /* forward the message */
  if(forward) {
    if(rcvd_from_bolt) {
      /* forward to network */
      /* queue is guaranteed to have at least one free spot (we check this 
        * before entering the wile loop */
      elwb_send_pkt((uint8_t*)msg, msg_len);
      DEBUG_PRINT_VERBOSE("BOLT msg forwarded to network (type: %u, dest: %u)",
                          msg->header.type, msg->header.target_id);
    } else {
      /* forward to BOLT */      
      bolt_write((uint8_t*)msg, msg_len);
      DEBUG_PRINT_VERBOSE("msg forwarded to BOLT (type: %u, len: %uB)", 
                          msg->header.type, msg_len);
    }
    return 0; /* forwarded */
  }
  return 1;   /* processed */
}
/*---------------------------------------------------------------------------*/
#if !IS_HOST
void
send_timestamp(int64_t captured)
{ 
  msg_tx.timestamp = 0;
  
  /* timestamp request: calculate the timestamp and send it over BOLT */
  /* only send a timestamp if the node is connected to the eLWB */
  rtimer_clock_t local_t_rx = 0;  /* in LF ticks */
  uint64_t elwb_time_secs = elwb_get_time(&local_t_rx);
  if(elwb_time_secs > 0) {
    /* get elapsed time in LF ticks and convert it to us */
    int64_t diff = ((int64_t)local_t_rx - captured) * 1000000/RTIMER_SECOND_LF;
    /* local t_rx is in clock ticks */
    msg_tx.timestamp = elwb_time_secs * 1000000 - diff;
  }
  /* send message over BOLT */
  send_msg(node_id, DPP_MSG_TYPE_TIMESYNC, 0, 0, 1);
  
  DEBUG_PRINT_INFO("timestamp sent: %llu", msg_tx.timestamp);
  captured = 0;
}
#endif /* IS_HOST */
/*---------------------------------------------------------------------------*/
void
send_node_info(void)
{
  memset((uint8_t*)&msg_tx.node_info, 0, sizeof(dpp_node_info_t));
  msg_tx.node_info.component_id = COMPONENT_ID;
  msg_tx.node_info.compiler_ver = COMPILER_VERSION_32;
  msg_tx.node_info.compile_date = COMPILE_TIME;       /* defined in makefile */
  msg_tx.node_info.fw_ver       = FW_VERSION;
  msg_tx.node_info.rst_cnt      = cfg.rst_cnt;
  msg_tx.node_info.rst_flag     = rst_flag;
  msg_tx.node_info.sw_rev_id    = hexstr_to_dec(GIT_HEADREV_SHA, 8);
  memcpy(msg_tx.node_info.compiler_desc, COMPILER_DESC, 
         MIN(4,strlen(COMPILER_DESC)));
  memcpy(msg_tx.node_info.fw_name, FW_NAME, MIN(8, strlen(FW_NAME)));
  memcpy(msg_tx.node_info.mcu_desc, MCU_DESC, MIN(12, strlen(MCU_DESC)));
  
  /* note: host sends message towards BOLT */
  send_msg(DPP_DEVICE_ID_SINK, DPP_MSG_TYPE_NODE_INFO, 0, 0, IS_HOST);
  DEBUG_PRINT_INFO("node info msg generated");
}
/*---------------------------------------------------------------------------*/
void
send_node_health(void)
{
  static uint8_t  tx_dropped_last = 0,
                  rx_dropped_last = 0;
  static uint16_t rx_cnt_last     = 0;
  static uint16_t max_stack_size  = 0;
  
  const elwb_stats_t* stats = elwb_get_stats();

  /* collect ADC values */
  ADC_REFOSC_ON;
  
  /* general stats */
  msg_tx.com_health.core_temp     = adc_get_temp();
  msg_tx.com_health.core_vcc      = adc_get_vcc();
  
  ADC_REFOSC_OFF;                      /* shut down REF module to save power */
  
  rtimer_clock_t now              = rtimer_now_lf();
  msg_tx.com_health.uptime        = now / RTIMER_SECOND_LF; 
                                    /* convert to seconds */
  msg_tx.com_health.msg_cnt       = rcvd_msg_cnt;
  rcvd_msg_cnt                    = 0;              /* reset */
  uint16_t stack_size = debug_print_get_max_stack_size();
  msg_tx.com_health.stack         = stack_size * 100 /
                                    (SRAM_END - DEBUG_CONF_STACK_GUARD - 7);
  /* print new stack size if it exceeds the current max. */
  if(stack_size > max_stack_size) {
    max_stack_size = stack_size;
    DEBUG_PRINT_INFO("max stack size: %ub (%u%%)", stack_size, 
                                                   msg_tx.com_health.stack);
  }

  /* radio / communication stats */
#if IS_HOST
  msg_tx.com_health.radio_snr     = glossy_get_snr();
  msg_tx.com_health.radio_rssi    = (uint8_t)(-stats->glossy_snr);
#else /* IS_HOST */
  msg_tx.com_health.radio_snr     = stats->glossy_snr;
  msg_tx.com_health.radio_rssi    = (uint8_t)(-glossy_get_rssi());
#endif /* IS_HOST */
  msg_tx.com_health.radio_tx_pwr  = rf1a_tx_power_val[cfg.tx_pwr];
  msg_tx.com_health.radio_per     = glossy_get_per();
  if(rx_cnt_last > stats->pkt_rcv) {
    msg_tx.com_health.rx_cnt      = (65535 - rx_cnt_last) + stats->pkt_rcv;
  } else {
    msg_tx.com_health.rx_cnt      = (stats->pkt_rcv - rx_cnt_last);
  }
  rx_cnt_last                     = stats->pkt_rcv;
                                    //glossy_get_n_pkts_crcok();
  msg_tx.com_health.tx_queue      = elwb_get_send_buffer_state();
  msg_tx.com_health.rx_queue      = elwb_get_rcv_buffer_state();
  msg_tx.com_health.tx_dropped    = stats->txbuf_drop - tx_dropped_last;
  tx_dropped_last = stats->txbuf_drop;
  msg_tx.com_health.rx_dropped    = stats->rxbuf_drop - rx_dropped_last;
  rx_dropped_last = stats->rxbuf_drop;
  
  /* duty cycle */
#if ENERGEST_CONF_ON
  static rtimer_clock_t   last_energest_rst = 0;
  msg_tx.com_health.cpu_dc        = (uint16_t)
                                    (energest_type_time(ENERGEST_TYPE_CPU) *
                                    1000 / (now - last_energest_rst));
  msg_tx.com_health.radio_rx_dc   = (uint16_t)
                                    (energest_type_time(ENERGEST_TYPE_LISTEN) *
                                    1000 / (now - last_energest_rst));
  msg_tx.com_health.radio_tx_dc   = (uint16_t)
                                    (energest_type_time(ENERGEST_TYPE_TRANSMIT)
                                    * 1000 / (now - last_energest_rst));
  last_energest_rst = now;
  energest_type_set(ENERGEST_TYPE_CPU, 0);
  energest_type_set(ENERGEST_TYPE_TRANSMIT, 0);
  energest_type_set(ENERGEST_TYPE_LISTEN, 0);
#else  /* ENERGEST_CONF_ON */
  DCSTAT_CPU_OFF;
  msg_tx.com_health.cpu_dc        = DCSTAT_CPU_DC;
  //msg_tx.com_health.radio_dc    = DCSTAT_RF_DC;
  msg_tx.com_health.radio_rx_dc   = DCSTAT_RFRX_DC;
  msg_tx.com_health.radio_tx_dc   = DCSTAT_RFTX_DC;
  DCSTAT_RESET;
  DCSTAT_CPU_ON;
#endif /* ENERGEST_CONF_ON */
    
  /* host must send it to BOLT, a source node into the network */
  send_msg(DPP_DEVICE_ID_SINK, DPP_MSG_TYPE_COM_HEALTH, 0, 0, IS_HOST);
  
  DEBUG_PRINT_INFO("health msg generated");
}
/*---------------------------------------------------------------------------*/
void
send_lwb_health(void)
{
  const elwb_stats_t* stats = elwb_get_stats();

  msg_tx.lwb_health.sleep_cnt     = stats->sleep_cnt;
  msg_tx.lwb_health.bootstrap_cnt = stats->bootstrap_cnt;
  msg_tx.lwb_health.fsr           = glossy_get_fsr();
  msg_tx.lwb_health.t_to_rx       = stats->glossy_t_to_rx;
  msg_tx.lwb_health.t_flood       = stats->glossy_t_flood;
  msg_tx.lwb_health.n_tx          = stats->glossy_n_tx;
  msg_tx.lwb_health.n_rx          = stats->glossy_n_rx;
  msg_tx.lwb_health.n_hops        = stats->relay_cnt;
  msg_tx.lwb_health.n_hops_max    = glossy_get_max_relay_cnt();
  msg_tx.lwb_health.unsynced_cnt  = stats->unsynced_cnt;
  msg_tx.lwb_health.drift         = stats->drift;
  msg_tx.lwb_health.bus_load      = stats->load;

  glossy_reset_stats();
    
  send_msg(DPP_DEVICE_ID_SINK, DPP_MSG_TYPE_LWB_HEALTH, 0, 0, IS_HOST);
  
  //DEBUG_PRINT_INFO("LWB health msg generated");
}
/*---------------------------------------------------------------------------*/
