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

/* TODO: use different streams for different message types */
#define STREAM_ID    1

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
    default: break;
    }
  } else {
    msg.header.payload_len = len;
  }
  msg.header.target_id     = recipient;
  if(send_to_bolt) {
    msg.header.seqnr       = seq_no_bolt++;
  } else {
    msg.header.seqnr       = seq_no_lwb++;
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
  static lwb_stream_req_t health_stream = { node_id, 0, STREAM_ID,
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
  if(lwb_stream_get_state(STREAM_ID) ==
     LWB_STREAM_STATE_ACTIVE) {
    if((curr_time - t_last_health_pkt) >= health_period) {
      /* generate a node health packet and schedule it for transmission */
      comm_health_t node_health;
      if(get_node_health(&node_health)) {
        send_msg(DEVICE_ID_SINK, MSG_TYPE_COMM_HEALTH,
                 (const uint8_t*)&node_health, 0, 0);
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
        LOG_INFO(LOG_EVENT_CFG_CHANGED, msg_buffer.comm_cmd.value);
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
