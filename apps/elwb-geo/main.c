/*
 * Copyright (c) 2017, Swiss Federal Institute of Technology (ETH Zurich).
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
 *          Felix Sutton
 */
 
/*
 * eLWB app, runs on the DPP
 */

#include "main.h"

/*---------------------------------------------------------------------------*/
#ifdef APP_TASK_ACT_PIN
#define TASK_ACTIVE             PIN_SET(APP_TASK_ACT_PIN)
#define TASK_SUSPENDED          PIN_CLR(APP_TASK_ACT_PIN)
#else  /* APP_TASK_ACT_PIN */
#define TASK_ACTIVE
#define TASK_SUSPENDED
#endif /* APP_TASK_ACT_PIN */
/*---------------------------------------------------------------------------*/
static message_t msg_buffer;
static int64_t captured = 0;    // last captured timestamp
/*---------------------------------------------------------------------------*/
void
process_message(message_t* msg, uint8_t rcvd_from_bolt)
{
  /* check message type */
  if(msg->header.type & MSG_TYPE_MIN) {
    DEBUG_PRINT_WARNING("unsupported message type (dropped)");
    return;
  }  
  /* verify CRC */
  if(MSG_GET_CRC16(msg) !=
     crc16((uint8_t*)msg, MSG_LEN_PTR(msg) - 2, 0)) {
    DEBUG_PRINT_WARNING("invalid CRC (dropped)");
    return;
  }
  /* check message length */
  uint16_t msg_len = MSG_LEN_PTR(msg);
  if(msg_len > MSG_PKT_LEN) {
    DEBUG_PRINT_WARNING("invalid message length (dropped)");
    return;
  }
  DEBUG_PRINT_INFO("msg rcvd from node %u (%u bytes)", 
                   msg_buffer.header.device_id,  
                   msg_len);
  
  /* extract the message type and handle the message */
  uint8_t msg_type = msg->header.type;
  if(msg_type == MSG_TYPE_COMM_CMD) {
    /* don't check target ID, assume commands are for this node */
    DEBUG_PRINT_INFO("command received");
    switch(msg->comm_cmd.type) {
    /*case COMM_CMD_LWB_PAUSE:
      ...
      break;
    */
    default:
      /* unknown command */
      break;
    }
  } else if (node_id == HOST_ID && msg_type == MSG_TYPE_TIMESYNC) {
    /* update own time */
    msg->timestamp /= 1000000;
    lwb_sched_set_time(msg->timestamp);
    DEBUG_PRINT_INFO("time updated");
    
  } else {
    /* all other message types, can't be handled on this node -> forward */
    if(rcvd_from_bolt) {
      /* received over Bolt -> forward to LWB */
      lwb_send_pkt(0, 1, (uint8_t*)msg, msg_len);
      DEBUG_PRINT_INFO("message forwarded to eLWB (%u bytes)", msg_len);
    } else {
      /* forward the packet to the app processor */
      bolt_write((uint8_t*)msg, msg_len);
      DEBUG_PRINT_INFO("message forwarded to BOLT (%u bytes)", msg_len);
    }
  }
}
/*---------------------------------------------------------------------------*/
void
bolt_read_msg(void)
{
  /* uint16_t to avoid aliasing issues */
  static uint16_t read_buffer[(BOLT_CONF_MAX_MSG_LEN + 1) / 2];
  uint16_t max_ops = 10;
  while(BOLT_DATA_AVAILABLE && max_ops) {
    if(bolt_read((uint8_t*)read_buffer)) {
      process_message((message_t*)read_buffer, 1);
    }
    max_ops--;
  }
}
/*---------------------------------------------------------------------------*/
void
bolt_capture_timestamp(void)
{ 
  /* simply store the timestamp, do calculations afterwards */
#if BOLT_CONF_TIMEREQ_HF_MODE
  rtimer_clock_t now = rtimer_now_hf();
#else
  rtimer_clock_t now = rtimer_now_lf();
#endif /* BOLT_CONF_TIMEREQ_HF_MODE */
  captured = now - ((uint16_t)(now & 0xffff) - BOLT_CONF_TIMEREQ_CCR);
}
/*---------------------------------------------------------------------------*/
void
bolt_send_timestamp(void)
{
  if (captured == 0) return;
  
  msg_buffer.header.device_id   = node_id;
  msg_buffer.header.type        = MSG_TYPE_TIMESYNC;
  msg_buffer.header.payload_len = sizeof(timestamp_t);
  
  /* timestamp request: calculate the timestamp and send it over BOLT */
  if(node_id == HOST_ID) {
    msg_buffer.timestamp = captured * 1000000 / 
      (BOLT_CONF_TIMEREQ_HF_MODE ? RTIMER_SECOND_HF : RTIMER_SECOND_LF) + 
      TIMESYNC_OFS;

  /* only send a timestamp if the node is connected to the LWB */
  } else {
    msg_buffer.timestamp = 0;
    if (lwb_get_state() == LWB_STATE_CONNECTED) {
      rtimer_clock_t local_t_rx = 0;
      uint64_t lwb_time_secs = lwb_get_time(&local_t_rx);
    #if !BOLT_CONF_TIMEREQ_HF_MODE
      /* convert to LF ticks */
      rtimer_clock_t lf, hf;
      rtimer_now(&hf, &lf);
      local_t_rx = lf - (hf - local_t_rx) * RTIMER_SECOND_LF /RTIMER_SECOND_HF;
    #endif /* BOLT_CONF_TIMEREQ_HF_MODE */
      /* if captured before last ref time update, then diff is > 0 and thus LWB time
       * needs to be decreased by the difference */
      int64_t diff = (local_t_rx - captured);   /* in clock ticks */
      /* local t_rx is in clock ticks */
      msg_buffer.timestamp = lwb_time_secs * 1000000 - diff * 1000000 / 
        (BOLT_CONF_TIMEREQ_HF_MODE ? RTIMER_SECOND_HF : RTIMER_SECOND_LF);
    }
  }
  bolt_write((uint8_t*)&msg_buffer, MSG_LEN(msg_buffer));
  DEBUG_PRINT_MSG_NOW("timestamp sent: %llu", msg_buffer.timestamp);
  captured = 0;
}
/*---------------------------------------------------------------------------*/
PROCESS(bolt_process, "Bolt Task");
PROCESS_THREAD(bolt_process, ev, data) 
{  
  PROCESS_BEGIN();
  
  while(1) {  
    TASK_SUSPENDED;
    PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_POLL);
    TASK_ACTIVE;
    
    /* read messages from Bolt */
    bolt_read_msg();
  }
  
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
PROCESS(app_process, "Application Task");
AUTOSTART_PROCESSES(&app_process);
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(app_process, ev, data) 
{  
  PROCESS_BEGIN();
  
  /* error checks */
  if(sizeof(message_t) != MSG_PKT_LEN) {
    DEBUG_PRINT_MSG_NOW("ERROR: message_t is too long (%u vs %u bytes)",
                        sizeof(message_t), MSG_PKT_LEN);
  }

#if TIMESYNC_INTERRUPT_BASED
  bolt_set_timereq_callback(bolt_capture_timestamp);
#endif /* TIMESYNC_INTERRUPT_BASED */
  
  /* start the LWB thread with a pre and post processing task */
  lwb_start(&bolt_process, &app_process);
  process_start(&bolt_process, NULL);

  /* main loop of the application task */
  while(1) {
    /* the app task should not do anything until it is explicitly granted 
     * permission (by receiving a poll event) by the LWB task */
    TASK_SUSPENDED;
    PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_POLL);
    TASK_ACTIVE;      /* application task runs now */
    
    /* read all messages received via the LWB */
    while(1) {
      if(!lwb_rcv_pkt((uint8_t*)&msg_buffer, 0, 0)) {
        break;
      }
      //DEBUG_PRINT_MSG_NOW("msg received");
      process_message(&msg_buffer, 0);
    }
    /* read all messages from Bolt */
    bolt_read_msg();
          
  #if !TIMESYNC_INTERRUPT_BASED
    if(bolt_handle_timereq((rtimer_clock_t*)&captured)) {
      bolt_send_timestamp();
    }
  #else 
    bolt_send_timestamp();
  #endif /* TIMESYNC_INTERRUPT_BASED */

  #if LWB_CONF_USE_LF_FOR_WAKEUP
    if(lwb_get_state() != LWB_STATE_INIT) {
      LWB_BEFORE_DEEPSLEEP();     /* prepare to go to LPM3 */
    }
  #endif /* LWB_CONF_USE_LF_FOR_WAKEUP */
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
