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

/* tmp buffer to read from BOLT */
static uint8_t bolt_buffer[BOLT_CONF_MAX_MSG_LEN];
/*---------------------------------------------------------------------------*/
void
host_run(void)
{
  static uint32_t t_last_health_pkt = 0;
  static uint16_t health_period     = LWB_CONF_SCHED_PERIOD_IDLE;
  static uint16_t rcv_cnt           = 0;
  message_t msg;

  /* forward the received data */
  while(lwb_rcv_pkt((uint8_t*)&msg, 0, 0)) {
    /* use DEBUG_PRINT_MSG_NOW to prevent a queue overflow */
    rcv_cnt++;
    DEBUG_PRINT_MSG_NOW("data packet received from node %u (rcv_cnt: %u)",
                        msg.header.device_id, rcv_cnt);
    /* forward the packet: write it to BOLT */
    BOLT_WRITE((uint8_t*)&msg, MSG_LEN(msg));
  }

#if SEND_HEALTH_DATA
  if((lwb_get_time(0) - t_last_health_pkt) >= health_period) {
    /* generate a health message */
    get_node_health(&msg.comm_health);
    send_msg(DEVICE_ID_SINK, MSG_TYPE_COMM_HEALTH,
             (const uint8_t*)&msg.comm_health, 0, 1);
    t_last_health_pkt = lwb_get_time(0);
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
    //LOG_INFO(LOG_EVENT_COMM_TIMESTAMP_SENT, 0);
  } //else: no timestamp request triggered or other unknown error

  /* msg available from BOLT? */
  /* only read as many packets from BOLT as there are spaces in the TX queue */
  uint16_t msg_cnt = 0;
  while(BOLT_DATA_AVAILABLE &&
        (lwb_get_send_buffer_state() < LWB_CONF_OUT_BUFFER_SIZE)) {
    uint8_t msg_len = 0;
    BOLT_READ(bolt_buffer, msg_len);
    if(msg_len) {
      msg_cnt++;
      memcpy(&msg, bolt_buffer, MSG_PKT_LEN);
      if(msg.header.target_id == node_id ||
         msg.header.target_id == DEVICE_ID_BROADCAST) {
        if(msg.header.type == MSG_TYPE_COMM_CMD) {
          DEBUG_PRINT_INFO("command received");
          switch(msg.comm_cmd.type) {
          case COMM_CMD_LWB_SET_ROUND_PERIOD:
            /* adjust the period */
            if(msg.comm_cmd.value < 600) {    /* 10min is max */
              lwb_sched_set_period(msg.comm_cmd.value);
              DEBUG_PRINT_INFO("LWB period set to %us", msg.comm_cmd.value);
            }
            break;
          case COMM_CMD_LWB_SET_HEALTH_PERIOD:
            health_period = msg.comm_cmd.value; /* adjust own health period */
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
              __bis_status_register(GIE | LPM3_bits);
              __no_operation();
              DEBUG_PRINT_MSG_NOW("LWB resumed");
              lwb_resume();
            }
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
        } /* else: unknown message type */
      }
      if(msg.header.target_id != node_id) {
        /* all other message types: forward to LWB */
        if(!lwb_send_pkt(msg.header.target_id, 0,
                         (uint8_t*)&msg, MSG_LEN(msg))) {
          DEBUG_PRINT_WARNING("message dropped (queue full)");
        } else {
          DEBUG_PRINT_INFO("message from BOLT forwarded to LWB (target: %u)",
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
/*ISR(PORT2, port2_interrupt)
{
  DEBUG_ISR_ENTRY;
  DCSTAT_CPU_ON;
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
  DCSTAT_CPU_OFF;
  DEBUG_ISR_EXIT;
}*/
#endif /* DEBUG_INTERRUPT_ENABLE */
/*---------------------------------------------------------------------------*/
