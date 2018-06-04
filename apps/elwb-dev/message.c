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
uint16_t health_msg_period = HEALTH_MSG_PERIOD;
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
uint64_t
utc_timestamp(void)
{  
  return utc_time + ((rtimer_now_lf() - utc_time_rx) *
                     (1000000 / RTIMER_SECOND_LF));
}
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
  /* compose the message header */
  msg_tx.header.device_id     = node_id;
  msg_tx.header.type          = type;
    msg_tx.header.payload_len = len;
  if(!len) {
    switch(type) {
    case DPP_MSG_TYPE_COM_HEALTH:
      msg_tx.header.payload_len = sizeof(dpp_com_health_t); break;
    case DPP_MSG_TYPE_CMD:
      msg_tx.header.payload_len = 6; break;  /* default is 6 bytes */
    case DPP_MSG_TYPE_NODE_INFO:
      msg_tx.header.payload_len = sizeof(dpp_node_info_t); break;
    case DPP_MSG_TYPE_TIMESYNC:
      msg_tx.header.payload_len = sizeof(dpp_timestamp_t); break;
    case DPP_MSG_TYPE_LWB_HEALTH:
      msg_tx.header.payload_len = sizeof(dpp_lwb_health_t); break;
    default:
      msg_tx.header.payload_len = 0; break;
    }
  }
  msg_tx.header.target_id = recipient;
  if(send_to_bolt) {
    msg_tx.header.seqnr   = seq_no_bolt++;
  } else {
    msg_tx.header.seqnr   = seq_no_lwb++;
  }
#if node_id == HOST_ID
  msg_tx.header.generation_time = utc_timestamp();
#else
  msg_tx.header.generation_time = lwb_get_timestamp();
#endif 
  
  /* copy the payload if valid */
  if(msg_tx.header.payload_len && data) {
    memcpy(msg_tx.payload, data, msg_tx.header.payload_len);
  }
  /* calculate and append the CRC */
  uint16_t msg_tx_len = DPP_MSG_LEN(&msg_tx);
  uint16_t crc = crc16((uint8_t*)&msg_tx, msg_tx_len - 2, 0);
  DPP_MSG_SET_CRC16(&msg_tx, crc);

  /* forward the message either to BOLT or the LWB */
  if(send_to_bolt) {
    if(bolt_write((uint8_t*)&msg_tx, msg_tx_len)) {
      DEBUG_PRINT_VERBOSE("msg written to BOLT");
      return 1;
    }
  } else {
    if(lwb_send_pkt(recipient, 1, (uint8_t*)&msg_tx, msg_tx_len)) {
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
     DPP_MSG_GET_CRC16(msg) != crc16((uint8_t*)msg, msg_len - 2, 0)) {
     DEBUG_PRINT_WARNING("message with invalid length or CRC");
     EVENT_WARNING(EVENT_CC430_INV_MSG, 0);
    return 1;
  }
  DEBUG_PRINT_VERBOSE("processing msg (type: %u src: %u target: %u len: %uB)", 
                      msg->header.type, msg->header.device_id, 
                      msg->header.target_id, msg_len);
  
  /* only process the message if target ID matched the node ID */
  uint16_t forward = (msg->header.target_id == DPP_DEVICE_ID_BROADCAST);
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
        PMM_TRIGGER_POR;
        break;
      case CMD_CC430_SET_ROUND_PERIOD:    /* value is in s */
        if(arg1 > 0 && arg1 <= 3600) {
          lwb_sched_set_period(arg1);
          DEBUG_PRINT_INFO("LWB period set to %us", arg1);
          successful = 1;
        }
        break;
      case CMD_CC430_SET_HEALTH_PERIOD:
        health_msg_period = arg1; /* set health period in s */
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
          cfg.tx_pwr = (uint8_t)arg1;
          nvcfg_save((uint8_t*)&cfg);
          rf1a_set_tx_power(arg1);
          successful = 1; 
        }
        break;
      case CMD_CC430_SET_DBG_FLAGS:
        cfg.dbg_flags = (uint8_t)arg1;
        nvcfg_save((uint8_t*)&cfg);
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
                   HOST_ID == node_id);
          successful = 1;
        }
        break;
      default:
        /* unknown command */
        forward = 1;  /* forward to BOLT */
        break;
      }
      if(successful) {
        DEBUG_PRINT_INFO("cmd processed, config changed");
        EVENT_INFO(EVENT_CC430_CFG_CHANGED,
                   (((uint32_t)arg1) << 16 | msg->cmd.type));
      }
#if TIMESYNC_HOST_RCV_UTC && (NODE_ID == HOST_ID)
    } else if(msg->header.type == DPP_MSG_TYPE_TIMESYNC) {
      utc_time    = msg->timestamp;
      utc_time_rx = bolt_captured_trq;
      utc_time_updated = 1;
#endif /* TIMESYNC_HOST_RCV_UTC */

#if FW_UPDATE_CONF_ON
    /* firmware message type (only continue if component ID matches) */
    } else if(msg->header.type == DPP_MSG_TYPE_FW &&
              msg->firmware.component_id == DPP_COMPONENT_ID_CC430) {
  #if (node_id == HOST_ID)
      /* host node only processes this msg type if target == node_id */
      if(msg->header.target_id == node_id) {
        if(!fw_process_msg(msg)) {
          DEBUG_PRINT_INFO("failed to process FW msg type");
        }
        forward = 0;   /* do not forward FW packets that are for host nodes! */
      } /* else: just forward to the network */
  #else /* node_id == HOST_ID */
      if(!fw_process_msg(msg)) {
        DEBUG_PRINT_INFO("failed to process FW msg type");
      }
      forward = 0;  /* source node shall not forward this to the AP */
  #endif /* node_id == HOST_ID */
#endif /* FW_UPDATE_CONF_ON */
      
    } else {
      /* unknown message type */
      if(node_id != HOST_ID) {
        forward = 1;
      } else {
        /* ignore */      
        EVENT_WARNING(EVENT_CC430_MSG_IGNORED, msg->header.type);
      }
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
      lwb_send_pkt(0, 1, (uint8_t*)msg, msg_len);
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
void
send_timestamp(int64_t captured)
{ 
  msg_tx.timestamp = 0;
  
  /* timestamp request: calculate the timestamp and send it over BOLT */
  if(node_id == HOST_ID) {
    msg_tx.timestamp = captured * 1000000 / RTIMER_SECOND_LF + TIMESYNC_OFS;

  /* only send a timestamp if the node is connected to the LWB */
  } else {
    rtimer_clock_t local_t_rx = 0;
    uint64_t lwb_time_secs = lwb_get_time(&local_t_rx);
    if(lwb_time_secs > 0) {
      /* convert to LF ticks */
      rtimer_clock_t lf, hf;
      rtimer_now(&hf, &lf);
      local_t_rx = lf - (hf - local_t_rx) * RTIMER_SECOND_LF /RTIMER_SECOND_HF;
      /* if captured before last ref time update, then diff is > 0 and thus 
       * LWB time needs to be decreased by the difference */
      int64_t diff = (local_t_rx - captured);   /* in clock ticks */
      /* local t_rx is in clock ticks */
      msg_tx.timestamp = lwb_time_secs * 1000000 - 
                         diff * 1000000 / RTIMER_SECOND_LF;
    }
  }
  /* send message over BOLT */
  send_msg(node_id, DPP_MSG_TYPE_TIMESYNC, 0, 0, 1);
  
  DEBUG_PRINT_INFO("timestamp sent: %llu", msg_tx.timestamp);
  captured = 0;
}
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
    
  send_msg(DPP_DEVICE_ID_SINK, DPP_MSG_TYPE_NODE_INFO, 0, 0,
           HOST_ID == node_id);   /* host sends message towards BOLT */
  DEBUG_PRINT_INFO("node info message generated");
}
/*---------------------------------------------------------------------------*/
void
send_node_health(void)
{
  static uint8_t   tx_dropped_last = 0,
                   rx_dropped_last = 0;
  static uint16_t  rx_cnt_last = 0;
  const lwb_statistics_t* lwb_stats = lwb_get_stats();

  /* collect ADC values */
  while(REFCTL0 & REFGENBUSY);
  REFCTL0 |= REFON;
  while(REFCTL0 & REFGENBUSY);
  __delay_cycles(MCLK_SPEED / 25000);      /* let REF settle */
  
  msg_tx.com_health.core_temp     = adc_get_temp();
  msg_tx.com_health.core_vcc      = adc_get_vcc();
  
  REFCTL0 &= ~REFON;                   /* shut down REF module to save power */
  
  rtimer_clock_t now              = rtimer_now_lf();
  msg_tx.com_health.uptime        = now / RTIMER_SECOND_LF; 
                                    /* convert to seconds */
  msg_tx.com_health.msg_cnt       = rcvd_msg_cnt;
  rcvd_msg_cnt                    = 0;              /* reset */
  msg_tx.com_health.stack         = debug_print_get_max_stack_size() * 100 /
                                    (SRAM_END - DEBUG_CONF_STACK_GUARD - 7);
#if NODE_ID == HOST_ID
  msg_tx.com_health.radio_snr     = glossy_get_snr();
  msg_tx.com_health.radio_rssi    = (uint8_t)(-lwb_stats->glossy_snr);
#else
  msg_tx.com_health.radio_snr     = lwb_stats->glossy_snr;
  msg_tx.com_health.radio_rssi    = (uint8_t)(-glossy_get_rssi());
#endif 
  msg_tx.com_health.radio_tx_pwr  = rf1a_tx_power_val[RF_CONF_TX_POWER];
  msg_tx.com_health.radio_per     = glossy_get_per();
  if(rx_cnt_last > lwb_stats->pck_cnt) {
    msg_tx.com_health.rx_cnt      = (65535 - rx_cnt_last) + lwb_stats->pck_cnt;
  } else {
    msg_tx.com_health.rx_cnt      = (lwb_stats->pck_cnt - rx_cnt_last);
  }
  rx_cnt_last                     = msg_tx.com_health.rx_cnt;
                                    //glossy_get_n_pkts_crcok();
  msg_tx.com_health.tx_queue      = lwb_get_send_buffer_state();
  msg_tx.com_health.rx_queue      = lwb_get_rcv_buffer_state();
  msg_tx.com_health.tx_dropped    = lwb_stats->txbuf_drop - tx_dropped_last;
  tx_dropped_last = lwb_stats->txbuf_drop;
  msg_tx.com_health.rx_dropped    = lwb_stats->rxbuf_drop - rx_dropped_last;
  rx_dropped_last = lwb_stats->rxbuf_drop;
  
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
  send_msg(DPP_DEVICE_ID_SINK, DPP_MSG_TYPE_COM_HEALTH, 0, 0, 
           node_id == HOST_ID);
  
  DEBUG_PRINT_INFO("health msg generated");
}
/*---------------------------------------------------------------------------*/
void
send_lwb_health(void)
{
  static uint16_t max_hops = 0;
  const lwb_statistics_t* lwb_stats = lwb_get_stats();

  msg_tx.lwb_health.sleep_cnt     = lwb_stats->sleep_cnt;
  msg_tx.lwb_health.bootstrap_cnt = lwb_stats->bootstrap_cnt;
  msg_tx.lwb_health.fsr           = glossy_get_fsr();
  msg_tx.lwb_health.t_to_rx       = (uint16_t)(glossy_get_t_to_first_rx()  *
                                               100 / 325);
  msg_tx.lwb_health.t_flood       = (uint16_t)(glossy_get_flood_duration() *
                                               100 / 325);
  msg_tx.lwb_health.n_tx          = glossy_get_n_tx();
  msg_tx.lwb_health.n_rx          = glossy_get_n_rx();
  msg_tx.lwb_health.n_hops        = lwb_stats->relay_cnt;
  if(lwb_stats->relay_cnt > max_hops) {
    max_hops                = lwb_stats->relay_cnt;
  }
  msg_tx.lwb_health.n_hops_max  = max_hops;

  //TODO: add     lwb_stats->unsynced_cnt
  //TODO: add     lwb_stats->drift

  glossy_reset_stats();
    
  send_msg(DPP_DEVICE_ID_SINK, DPP_MSG_TYPE_LWB_HEALTH, 0, 0, 
           node_id == HOST_ID);
  
  //DEBUG_PRINT_INFO("LWB health msg generated");
}
/*---------------------------------------------------------------------------*/
