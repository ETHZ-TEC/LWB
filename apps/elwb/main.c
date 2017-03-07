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
static message_min_t msg_buffer;
static uint16_t t_offset = 0;
/*---------------------------------------------------------------------------*/
void
process_message(message_min_t* msg)
{
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
  /* check message type */
  if((msg->header.type & MSG_TYPE_MIN) == 0) {
    // not supported message type!
    DEBUG_PRINT_WARNING("message type not supported!");
    return;
  }
  msg->header.type &= ~MSG_TYPE_MIN;    // clear the last bit
  /* handle message */
  if(msg->header.type == MSG_TYPE_COMM_CMD) {
    DEBUG_PRINT_INFO("command received");
    switch(msg->comm_cmd.type) {
    case COMM_CMD_LWB_PAUSE:
      lwb_pause();
      /* disable the time request CCR interrupt and enable the port interrupt
       * instead! */
      rtimer_wait_for_event(BOLT_CONF_TIMEREQ_TIMERID, 0);
      PIN_UNSEL(BOLT_CONF_TIMEREQ_PIN);
      PIN_CFG_INT(BOLT_CONF_TIMEREQ_PIN);
      DEBUG_PRINT_MSG_NOW("LWB paused");
      break;
    default:
      /* unknown command */
      break;
    }
  } else if(msg->header.type == MSG_TYPE_AE_EVENT) {
    if(node_id == HOST_ID) {
      // host shall forward these packets to Bolt
      DEBUG_PRINT_INFO("AE event received (ID: %u, timestamp: %llu)",
                       msg->ae_evt.event_id,
                       msg->ae_evt.generation_time);
      /* forward the packet to the app processor */
      bolt_write((uint8_t*)msg, MSG_LEN_PTR(msg));
    } else
    {
      // source node shall forward these packets to LWB
      lwb_send_pkt(0, 1, (uint8_t*)msg, MSG_LEN_PTR(msg));
    }
  } else {
    DEBUG_PRINT_INFO("unknown message type received");
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
    uint8_t msg_len = bolt_read((uint8_t*)read_buffer);
    if(msg_len) {
      DEBUG_PRINT_INFO("message read from Bolt (%db)", msg_len);
      process_message((message_min_t*)read_buffer);
    }
    max_ops--;
  }
}
/*---------------------------------------------------------------------------*/
#if TIMESYNC_INTERRUPT_BASED
void
bolt_timereq_cb(void)
{
  /* timestamp request: calculate the timestamp and send it over BOLT */
  rtimer_clock_t now = rtimer_now_hf();

  int64_t captured;
  if(t_offset) {
    captured = -t_offset;
  } else {
    captured = now - ((uint16_t)(now & 0xffff) - BOLT_CONF_TIMEREQ_CCR);
  }
  if(node_id == HOST_ID) {
    msg_buffer.header.device_id   = node_id;
    msg_buffer.header.type        = MSG_TYPE_MIN | MSG_TYPE_TIMESYNC;
    msg_buffer.header.payload_len = sizeof(timestamp_t);
    bolt_write((uint8_t*)&msg_buffer, MSG_LEN(msg_buffer));

  /* only send a timestamp if the node is connected to the LWB */
  } else if(lwb_get_state() == LWB_STATE_CONNECTED) {
    
    rtimer_clock_t local_t_rx = 0;
    int64_t lwb_time_secs     = lwb_get_time(&local_t_rx);
    /* local t_rx is in HF clock ticks */
    int64_t  ofs              = lwb_time_secs * RTIMER_SECOND_HF - local_t_rx;
    msg_buffer.timestamp      = captured + ofs + TIMESYNC_OFS;
    /* drift compensation */
    const lwb_statistics_t* lwb_stats = lwb_get_stats();
    if(lwb_stats->drift) {
      int32_t elapsed         = (int64_t)captured - local_t_rx;
      /* drift is in HF ticks per second, only when LF not used for wakeup */
      int16_t drift_comp      = (int16_t)(elapsed *
                                (int32_t)lwb_stats->drift);
      msg_buffer.timestamp   -= drift_comp;
      /* debug only */
      //DEBUG_PRINT_INFO("drift compensation: %d", drift_comp);
    }
    /* convert from clock ticks to us */
    msg_buffer.timestamp          = msg_buffer.timestamp * 1000000 /
                                    RTIMER_SECOND_HF - t_offset;
    msg_buffer.header.device_id   = node_id;
    msg_buffer.header.type        = MSG_TYPE_MIN | MSG_TYPE_TIMESYNC;
    msg_buffer.header.payload_len = sizeof(timestamp_t);
    bolt_write((uint8_t*)&msg_buffer, MSG_LEN(msg_buffer));

    t_offset = 0;
    DEBUG_PRINT_MSG_NOW("timestamp sent: %llu", msg_buffer.timestamp);

  } // else: not yet synced, can't send timestamp
}
#endif /* TIMESYNC_INTERRUPT_BASED */
/*---------------------------------------------------------------------------*/
void
host_run(void)
{
  /* print out the received data */
  uint8_t pkt_len;
  do {
    pkt_len = lwb_rcv_pkt((uint8_t*)&msg_buffer, 0, 0);
    if(pkt_len) {
      DEBUG_PRINT_INFO("msg (%u bytes) rcvd from node %u", 
                       pkt_len, 
                       msg_buffer.header.device_id);
      process_message(&msg_buffer);
    }
  } while(pkt_len);
  
  bolt_read_msg();

#if !TIMESYNC_INTERRUPT_BASED
  rtimer_clock_t bolt_timestamp = 0;
  if(bolt_handle_timereq(&bolt_timestamp)) { 
    msg_buffer.header.device_id   = node_id;
    msg_buffer.header.type        = MSG_TYPE_TIMESYNC;
    msg_buffer.header.payload_len = sizeof(timestamp_t);
    msg_buffer.timestamp          = bolt_timestamp;
    BOLT_WRITE((uint8_t*)&msg_buffer, MSG_LEN(msg_buffer));
  }
#endif /* TIMESYNC_INTERRUPT_BASED */
}
/*---------------------------------------------------------------------------*/
void
source_run(void)
{
  if(lwb_rcv_pkt((uint8_t*)&msg_buffer, 0, 0)) {
    DEBUG_PRINT_INFO("pkt received");
    process_message(&msg_buffer);
  }
  if(t_offset) {
    bolt_timereq_cb();
  }

  bolt_read_msg();
    
#if !TIMESYNC_INTERRUPT_BASED
  rtimer_clock_t bolt_timestamp = 0;
  if(bolt_handle_timereq(&bolt_timestamp)) {
    rtimer_clock_t local_rx_time = 0;
    uint32_t lwb_time_secs = lwb_get_time(&local_rx_time);
    int64_t lwb_time   = (uint64_t)lwb_time_secs * RTIMER_SECOND_HF;
    int64_t ofs        = lwb_time - local_rx_time;
    bolt_timestamp     = bolt_timestamp + ofs + TIMESYNC_OFS;
    msg_buffer.header.device_id   = node_id;
    msg_buffer.header.type        = MSG_TYPE_MIN | MSG_TYPE_TIMESYNC;
    msg_buffer.header.payload_len = sizeof(timestamp_t);
    msg_buffer.timestamp          = bolt_timestamp;
    BOLT_WRITE((uint8_t*)&msg_buffer, MSG_LEN(msg_buffer));
  }
#endif /* TIMESYNC_INTERRUPT_BASED */
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
  if(sizeof(message_min_t) != MSG_PKT_LEN) {
    DEBUG_PRINT_MSG_NOW("ERROR: message_min_t is too long");
  }

#if TIMESYNC_INTERRUPT_BASED
  bolt_set_timereq_callback(bolt_timereq_cb);
#endif /* TIMESYNC_INTERRUPT_BASED */
  
  /* start the LWB thread with a pre and post processing task */
  lwb_start(&bolt_process, &app_process);
  process_start(&bolt_process, NULL);

  /* main loop of the application task */
  while(1) {
    /* the app task should not do anything until it is explicitly granted 
     * permission (by receiving a poll event) by the LWB task */
    PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_POLL);
    TASK_ACTIVE;      /* application task runs now */
        
    if(HOST_ID == node_id) {
      host_run();
    } else {
      source_run();
    }

    if(lwb_get_state() == LWB_STATE_SUSPENDED) {
      /* disable the time request CCR interrupt and enable the port
       * interrupt instead */
      rtimer_wait_for_event(BOLT_CONF_TIMEREQ_TIMERID, 0);
      PIN_UNSEL(BOLT_CONF_TIMEREQ_PIN);
      PIN_CFG_INT(BOLT_CONF_TIMEREQ_PIN);
    }
  #if LWB_CONF_USE_LF_FOR_WAKEUP
    if(!glossy_is_active()) {
      LWB_BEFORE_DEEPSLEEP();     /* prepare to go to LPM3 */
    }
  #endif /* LWB_CONF_USE_LF_FOR_WAKEUP */
    TASK_SUSPENDED;
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
ISR(PORT2, port2_interrupt)
{
  /* TRQ ISR (CCR interrupt) */
  /* from trigger to end of HFXT stabilization ~0.2ms
   * ~40us from trigger to first instruction in this function, 20 ~ 32us from
   * trigger edge to the first MCLK edge! -> too high
   * according to the datasheet, the wake-up time (that is to the first rising
   * edge of the MCLK) from LPM3 should not exceed 6us
   * -> CCR seems to be the cause; port interrupt only takes 2.8us to first
   * MCLK cycle
   * A delay loop instead of wait for the XT2 to stabilize is possible, but
   * the DCO frequency is very unstable as it seems (> 1 LFXT tick variation
   * for 200us) -> really need to count the cycles until the LFCLK changes */

  if(PIN_IFG(BOLT_CONF_TIMEREQ_PIN)) {

#if LWB_CONF_USE_LF_FOR_WAKEUP
    /* MCLK runs from DCO, DCO is sourced by LFXT (=> in sync) */
    uint16_t ta1hw = TA1R;
    uint16_t cnt = 0;
    /* wake-up time: ~6.5us, incl. all instructions until beg. of loop */
    /* ~1.1us per loop pass (see disassembly for cycle count) */
    while(ta1hw == TA1R) { cnt++; }
    //rtimer_clock_t now = rtimer_now_lf();
    uint16_t us = (cnt * 11 + 65) / 10;
    t_offset = us;

    /* start the HFXT Osc. if necessary */
    if(UCSCTL6 & XT2OFF) {
      ENABLE_XT2();
      /* wait for the HFXT to stabilize (takes ~170us) */
      WAIT_FOR_OSC();
      UCSCTL4  = SELA | SELS | SELM;
      /*__delay_cycles(100);*/ /* errata PMM11/12? */
      /*UCSCTL5  = DIVA | DIVS | DIVM;*/
      UCSCTL7  = 0; /* errata UCS11 */
      P1SEL    = (BIT2 | BIT3 | BIT4 | BIT5 | BIT6 | BIT7);
      PIN_XOR(COM_MCU_INT2);
      TA0CTL   = TASSEL_2 | MC_2 | ID__1 | TACLR | TAIE;

      // use TA0CCR2 to catch the rising edge of ACLK, set CCI2B
      TA0CCTL2 = CAP | CM_1 | SCS | CCIS_1;
      while (!(TA0CCTL2 & CCIFG));
      uint16_t lf_ticks = TA1R;
      uint16_t hf_ticks = TA0CCR2;
      TA0CCTL2 = 0;
      t_offset = (uint16_t)((uint32_t)(lf_ticks - ta1hw - 1) * 1000000 / 32768
                            + us - (hf_ticks * 100 / 325));
      DEBUG_PRINT_MSG_NOW("LF/HF pair: lf=%u hf=%u, elapsed: %u",
                          lf_ticks,
                          hf_ticks,
                          t_offset);
    }
    //DEBUG_PRINT_MSG_NOW("timestamp: %u (%lu), count: %u, offset: %u", ta1hw,
    //                    (uint32_t)(ta1hw) * 31, cnt, us);

// else: nothing to convert or calculate
#endif /* LWB_CONF_USE_LF_FOR_WAKEUP */

    /* not synced, can't send a timestamp now */

    /* reenable timestamp request CCR */
    PIN_INT_DIS(BOLT_CONF_TIMEREQ_PIN);
    PIN_SEL(BOLT_CONF_TIMEREQ_PIN);
    //PIN_MAP_AS_INPUT(BOLT_CONF_TIMEREQ_PIN, BOLT_CONF_TIMEREQ_PINMAP);
    rtimer_wait_for_event(BOLT_CONF_TIMEREQ_TIMERID,
                          (rtimer_callback_t)bolt_timereq_cb);
    lwb_resume();
    DEBUG_PRINT_MSG_NOW("port interrupt");
    PIN_CLR_IFG(BOLT_CONF_TIMEREQ_PIN);
  }
}
/*---------------------------------------------------------------------------*/
