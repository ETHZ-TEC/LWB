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
void
source_run(void)
{
#if SEND_HEALTH_DATA
  static uint8_t bolt_buffer[BOLT_CONF_MAX_MSG_LEN];
  static lwb_stream_req_t health_stream = { 0, 0, STREAM_ID,
                                            LWB_CONF_SCHED_PERIOD_IDLE };
  static uint32_t t_last_health_pkt = 0;
  static uint32_t curr_time         = 0;
  static uint16_t health_period     = LWB_CONF_SCHED_PERIOD_IDLE;
  static uint8_t  ipi_changed       = 1;
  static uint8_t  node_info_sent    = 0;
  static uint16_t last_error_cnt    = 0;
  message_t msg;
  health_stream.id = node_id;

  curr_time = rtimer_now_lf() / RTIMER_SECOND_LF;

  /* adjust the IPI in case the fill level of the output queue reaches a
   * certain threshold */
  if(health_stream.ipi == health_period) {
    if(lwb_get_send_buffer_state() > (LWB_CONF_OUT_BUFFER_SIZE / 2)) {
      health_stream.ipi = health_period / 2; /* reduce the IPI */
      ipi_changed = 1;
    }
  } else if(lwb_get_send_buffer_state() < 2) {
    /* only 0 or 1 element left in the queue -> set IPI back to default */
    health_stream.ipi = health_period;
    ipi_changed = 1;
  }

  /* only send data if the stream is active */
  if(lwb_stream_get_state(STREAM_ID) == LWB_STREAM_STATE_ACTIVE) {
    /* node info already sent? */
    if(!node_info_sent) {
      /* send a node info message */
      get_node_info(&msg.node_info);
      send_msg(DEVICE_ID_SINK, MSG_TYPE_NODE_INFO,
               (uint8_t*)&msg.node_info, 0, 0);
      node_info_sent = 1;
    }
    if((curr_time - t_last_health_pkt) >= health_period) {
      /* generate a node health packet and schedule it for transmission */
      get_node_health(&msg.comm_health);
      send_msg(DEVICE_ID_SINK, MSG_TYPE_COMM_HEALTH,
               (const uint8_t*)&msg.comm_health, 0, 0);
      t_last_health_pkt = curr_time;
      DEBUG_PRINT_INFO("health packet generated");
    }
  }

  /* is there a packet to read? */
  uint8_t   count = 0;
  while(lwb_rcv_pkt((uint8_t*)&msg, 0, 0)) {
    count++;
    if(msg.header.target_id == node_id ||
       msg.header.target_id == DEVICE_ID_BROADCAST) {
      if(msg.header.type == MSG_TYPE_COMM_CMD) {
        switch(msg.comm_cmd.type) {
        case COMM_CMD_LWB_SET_HEALTH_PERIOD:
          /* change health/status report interval */
          health_period = msg.comm_cmd.value;
          health_stream.ipi = msg.comm_cmd.value;
          ipi_changed = 1;
          break;
        case COMM_CMD_LWB_SET_TX_PWR:
          if(msg.comm_cmd.value < N_TX_POWER_LEVELS) {
            rf1a_set_tx_power((rf1a_tx_powers_t)msg.comm_cmd.value);
          }
          break;
        case COMM_CMD_NODE_RESET:
          PMM_TRIGGER_BOR;
          break;
        default:
          /* unknown command */
          break;
        }
        LOG_INFO(LOG_EVENT_CFG_CHANGED, msg.comm_cmd.value);
      } else if(msg.header.type == MSG_TYPE_APP_FW) {
        /* forward to BOLT */
        BOLT_WRITE((uint8_t*)&msg, MSG_LEN(msg));
        DEBUG_PRINT_MSG_NOW("packet forwarded to BOLT");
      }
    } /* else: target ID does not match node ID */
  }
  if(count) {
    DEBUG_PRINT_INFO("%d packet(s) received", count);
  }

  while(BOLT_DATA_AVAILABLE) {
    uint8_t msg_len = 0;
    BOLT_READ(bolt_buffer, msg_len);
    /* sanity check for payload length */
    if(msg_len) {
      memcpy(&msg, bolt_buffer, MSG_PKT_LEN);
      /* just forward the message to the LWB */
      if(!lwb_send_pkt(msg.header.target_id, STREAM_ID,
                       (uint8_t*)&msg, MSG_LEN(msg))) {
        DEBUG_PRINT_INFO("message from BOLT dropped (LWB queue full)");
      } else {
        DEBUG_PRINT_INFO("message from BOLT forwarded to LWB");
      }
    }
  }

  uint16_t glossy_error = glossy_get_n_errors();
  if(last_error_cnt != glossy_error) {
    last_error_cnt = glossy_error;
    LOG_ERROR(LOG_EVENT_GLOSSY_ERROR, last_error_cnt);
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
