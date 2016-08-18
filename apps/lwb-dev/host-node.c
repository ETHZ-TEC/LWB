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

/**
 * @brief Low-Power Wireless Bus Test Application
 *
 * A simple range test application. Each source node sends some status data
 * (RSSI, battery voltage, temperature, ...) to the host in each round.
 */


#include "contiki.h"
#include "platform.h"

#include "../lwb-dev/packet.h"                    /* packet structure and message types */
/*---------------------------------------------------------------------------*/
void
send_msg(uint16_t recipient, const message_t* data, uint8_t len)
{
#define LWB_STREAM_ID_STATUS_MSG        1

  tl_packet_t tl;
  tl.header.device_id               = node_id;
  tl.header.target_id               = recipient;
  tl.header.payload_len             = len;
  //msg_buffer.header.seqnr           = seq_no++;
  if(data) {
    memcpy(&tl.message, data, len);
  }
  if(!lwb_put_data(LWB_RECIPIENT_SINKS, LWB_STREAM_ID_STATUS_MSG,
                   (uint8_t*)&tl, TL_HDR_LEN + tl.header.payload_len)) {
    DEBUG_PRINT_WARNING("message dropped (queue full)");
  } else {
    DEBUG_PRINT_INFO("message for node %u added to TX queue", recipient);
  }
}
/*---------------------------------------------------------------------------*/
void
host_init(void)
{
#if LWB_CONF_USE_LF_FOR_WAKEUP
  SVS_DISABLE;
#endif /* LWB_CONF_USE_LF_FOR_WAKEUP */

  DEBUG_PRINT_MSG_NOW("msg: %u, tl: %u, lwb: %u",
                      sizeof(message_t), sizeof(tl_packet_t),
            LWB_CONF_MAX_DATA_PKT_LEN);
}
/*---------------------------------------------------------------------------*/
void
host_run(void)
{
  /* for BOLT messages */
  uint8_t   bolt_buffer[BOLT_CONF_MAX_MSG_LEN];
  message_t bolt_msg;

  /* analyze and print the received data */
  uint8_t pkt_len = 0;
  tl_packet_t pkt;
  do {
    uint8_t pkt_len = lwb_get_data((uint8_t*)&pkt, 0, 0);
    if(pkt_len) {
      message_t* msg = (message_t*)pkt.payload;
      /* use DEBUG_PRINT_MSG_NOW to prevent a queue overflow */
      DEBUG_PRINT_MSG_NOW("data packet received from node %u",
                          pkt.header.device_id);
      if(msg->header.type == MSG_TYPE_CC430_HEALTH) {
        /* forward the packet: write it to BOLT */
        BOLT_WRITE(pkt.payload, pkt.header.payload_len);
      }
    }
  } while(pkt_len);

  /* handle timestamp requests */
  uint64_t time_last_req = bolt_handle_timereq();
  if(time_last_req) {
    /* write the timestamp to BOLT (convert to us) */
    bolt_msg.header.type = MSG_TYPE_TIMESTAMP;
    uint64_t timestamp     = time_last_req * 1000000 /
                              ACLK_SPEED + LWB_CLOCK_OFS;
    memcpy(bolt_msg.payload, &timestamp, sizeof(uint64_t));
    BOLT_WRITE((uint8_t*)&bolt_msg, MSG_HDR_LEN);
  }
  /* msg available from BOLT? */
  uint16_t msg_cnt = 0;
  while(BOLT_DATA_AVAILABLE) {
    uint8_t msg_len = 0;
    BOLT_READ(bolt_buffer, msg_len);
    if(msg_len) {
      msg_cnt++;
      memcpy(&bolt_msg, bolt_buffer, MSG_PKT_LEN);
      if(bolt_msg.header.type == MSG_TYPE_LWB_CMD) {
        if(bolt_msg.payload[0] == LWB_CMD_SET_SCHED_PERIOD) {
          /* adjust the period */
          lwb_sched_set_period(bolt_msg.payload[1]);
          DEBUG_PRINT_INFO("LWB period set to %us",
              bolt_msg.payload[1]);
        } else if(bolt_msg.payload[0] == LWB_CMD_SET_STATUS_PERIOD) {
          /* broadcast the message */
          send_msg(bolt_msg.payload[4], &bolt_msg, MSG_HDR_LEN + 4);
        } else if(bolt_msg.payload[0] == LWB_CMD_PAUSE) {
          /* stop */
          while(BOLT_DATA_AVAILABLE) {        /* flush the queue */
            BOLT_READ(bolt_buffer, msg_len);
          }
          /* configure a port interrupt for the IND pin */
          __dint();
          PIN_IES_RISING(BOLT_CONF_IND_PIN);
          PIN_CLR_IFG(BOLT_CONF_IND_PIN);
          PIN_INT_EN(BOLT_CONF_IND_PIN);
          if(BOLT_DATA_AVAILABLE) {
            DEBUG_PRINT_MSG_NOW("failed to stop LWB");
            __eint();
          } else {
            lwb_pause();
            DEBUG_PRINT_MSG_NOW("LWB paused");
            __bis_status_register(GIE | SCG0 | SCG1 | CPUOFF);
            __no_operation();
            DEBUG_PRINT_MSG_NOW("LWB resumed");
            lwb_resume();
            continue;
          }
        }
      }
    }
  }
  if(msg_cnt) {
    DEBUG_PRINT_MSG_NOW("%u messages read from BOLT", msg_cnt);
  }
}
/*---------------------------------------------------------------------------*/
