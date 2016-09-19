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

/**
 * @brief Low-Power Wireless Bus Test Application
 *
 * A simple range test application. Each source node sends some status data
 * (RSSI, battery voltage, temperature, ...) to the host in each round.
 */


#include "main.h"

/* TODO use different stream IDs for different message types */
#define LWB_STREAM_ID_HOST  1

static uint16_t seq_no = 0;
/* tmp buffer to read from BOLT */
static uint8_t  bolt_buffer[BOLT_CONF_MAX_MSG_LEN];
/*---------------------------------------------------------------------------*/
void
send_msg(uint16_t recipient,
         const uint8_t* data,
         uint8_t len,
         message_type_t type)
{
  if(!data || !len) {
    return;
  }
  message_t msg;
  msg.header.device_id   = node_id;
  msg.header.type        = type;
  msg.header.payload_len = len;
  msg.header.target_id   = recipient;
  msg.header.seqnr       = seq_no++;
  msg.header.generation_time = 0;     /* currently not used */
  memcpy(msg.payload, data, len);
  uint16_t crc = crc16((uint8_t*)&msg, msg.header.payload_len + MSG_HDR_LEN,0);
  MSG_SET_CRC16(&msg, crc);

  if(!lwb_send_pkt(recipient, LWB_STREAM_ID_HOST,
                   (uint8_t*)&msg, MSG_LEN(msg))) {
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

  /* init the ADC */
  adc_init();
  REFCTL0 &= ~REFON;             /* shut down REF module to save power */
}
/*---------------------------------------------------------------------------*/
void
host_run(void)
{
  /* analyze and print the received data */
  message_t msg;
  while(lwb_rcv_pkt((uint8_t*)&msg, 0, 0)) {
    /* use DEBUG_PRINT_MSG_NOW to prevent a queue overflow */
    DEBUG_PRINT_MSG_NOW("data packet received from node %u (%u, %u)",
                        msg.header.device_id, msg.comm_health.lwb_t_to_rx, msg.comm_health.lwb_t_flood);
    /* forward the packet: write it to BOLT */
    BOLT_WRITE((uint8_t*)&msg, MSG_LEN(msg));
  }

#if SEND_HEALTH_DATA
  /* generate a health message */
  if(get_node_health(&msg.comm_health)) {
    /* calculate the timestamp, convert to microseconds */
    rtimer_clock_t round_start;
    uint64_t       lwb_time_seconds = lwb_get_time(&round_start);
    round_start = (rtimer_now_hf() - round_start) * 1000000 / SMCLK_SPEED;
    msg.header.device_id       = node_id;
    msg.header.type            = MSG_TYPE_COMM_HEALTH;
    msg.header.payload_len     = sizeof(comm_health_t);
    msg.header.target_id       = DEVICE_ID_SINK;
    msg.header.seqnr           = seq_no++;
    msg.header.generation_time = lwb_time_seconds * 1000000 + round_start;
    uint16_t crc = crc16((uint8_t*)&msg, msg.header.payload_len + MSG_HDR_LEN,
                         0);
    MSG_SET_CRC16(&msg, crc);
    BOLT_WRITE((uint8_t*)&msg, MSG_LEN(msg));
  }
#endif /* SEND_HEALTH_DATA */

  /* handle timestamp requests */
  uint64_t time_last_req;
  if(bolt_handle_timereq(&time_last_req)) {
    /* write the timestamp to BOLT (convert to us) */
    msg.header.type            = MSG_TYPE_TIMESYNC;
    msg.header.payload_len     = 0;
    msg.header.generation_time = time_last_req * 1000000 / ACLK_SPEED;
    msg.payload16[0] = crc16((uint8_t*)&msg, MSG_HDR_LEN, 0);
    BOLT_WRITE((uint8_t*)&msg, MSG_HDR_LEN + 2);
    DEBUG_PRINT_INFO("timestamp sent");
  } //else: no timestamp request triggered or other unknown error

  /* msg available from BOLT? */
  uint16_t msg_cnt = 0;
  while(BOLT_DATA_AVAILABLE) {
    uint8_t msg_len = 0;
    BOLT_READ(bolt_buffer, msg_len);
    if(msg_len) {
      msg_cnt++;
      memcpy(&msg, bolt_buffer, MSG_PKT_LEN);
      if(msg.header.target_id == NODE_ID ||
         msg.header.target_id == DEVICE_ID_BROADCAST) {
        if(msg.header.type == MSG_TYPE_COMM_CMD) {
          DEBUG_PRINT_INFO("command received");
          switch(msg.comm_cmd.type) {
          case COMM_CMD_LWB_SET_ROUND_PERIOD:
            /* adjust the period */
            lwb_sched_set_period(msg.comm_cmd.value);
            DEBUG_PRINT_INFO("LWB period set to %us", msg.comm_cmd.value);
            break;
          case COMM_CMD_LWB_SET_HEALTH_PERIOD:
            /* broadcast the message */
            send_msg(msg.header.target_id, msg.payload,
                     sizeof(comm_cmd_t), MSG_TYPE_COMM_CMD);
            break;
          case COMM_CMD_LWB_PAUSE:
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
            }
            break;
          case COMM_CMD_LWB_SET_TX_PWR:
            glossy_set_tx_pwr(msg.comm_cmd.value);
            break;
          default:
            /* unknown command */
            break;
          }
        } /* else: unknown message type */
      } else {
        /* all other message types: forward to LWB */
        if(!lwb_send_pkt(msg.header.target_id, LWB_STREAM_ID_HOST,
                         (uint8_t*)&msg, MSG_LEN(msg))) {
          DEBUG_PRINT_WARNING("message dropped (queue full)");
        } else {
          DEBUG_PRINT_INFO("message for node %u added to TX queue",
                           msg.header.target_id);
        }
      } // else: invalid target ID
    }
  }
  if(msg_cnt) {
    DEBUG_PRINT_INFO("%u messages read from BOLT", msg_cnt);
  }
}
/*---------------------------------------------------------------------------*/
#if !DEBUG_INTERRUPT_ENABLE
ISR(PORT2, port2_interrupt)
{
  ENERGEST_ON(ENERGEST_TYPE_CPU);

  if(PIN_IFG(BOLT_CONF_IND_PIN)) {
    while(BOLT_DATA_AVAILABLE) {
      uint8_t msg_len = 0;
      BOLT_READ(bolt_buffer, msg_len);
      message_t* msg = (message_t*)bolt_buffer;
      if(msg_len && msg->header.type == MSG_TYPE_COMM_CMD &&
         msg->comm_cmd.type == COMM_CMD_LWB_RESUME) {
        PIN_INT_OFF(BOLT_CONF_IND_PIN);
        __bic_status_register_on_exit(SCG0 | SCG1 | CPUOFF);
        break;
      }
    }
    PIN_CLR_IFG(BOLT_CONF_IND_PIN);
  }

  ENERGEST_OFF(ENERGEST_TYPE_CPU);
}
#endif /* DEBUG_INTERRUPT_ENABLE */
/*---------------------------------------------------------------------------*/
