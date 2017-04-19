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
 *          Felix Sutton
 *          Tonio Gsell
 *          Roman Lim
 */
 
/*
 * eLWB app for the nano-tera demo in March 2017
 */

#include "main.h"

/*---------------------------------------------------------------------------*/
#ifdef APP_TASK_ACT_PIN
#define TASK_ACTIVE             PIN_CLR(APP_TASK_ACT_PIN); \
                                PIN_SET(APP_TASK_ACT_PIN)
#define TASK_SUSPENDED          PIN_CLR(APP_TASK_ACT_PIN)
#else  /* APP_TASK_ACT_PIN */
#define TASK_ACTIVE
#define TASK_SUSPENDED
#endif /* APP_TASK_ACT_PIN */
/*---------------------------------------------------------------------------*/
static message_min_t msg_buffer;
/*---------------------------------------------------------------------------*/
void
host_run(void)
{
  /* print out the received data */
  uint64_t ae_timestamps[2] = { 0 };
  uint8_t msg_cnt = 0, msg_inv = 0;
  
  while (1) {
    uint8_t pkt_len = lwb_rcv_pkt((uint8_t*)&msg_buffer, 0, 0);
    if(pkt_len) {
      /* verify length and CRC */
      if(msg_buffer.header.payload_len > (pkt_len - 2 - MSG_HDR_LEN)) {
        DEBUG_PRINT_VERBOSE("invalid msg length (dropped)");
        msg_inv++;
      } else if(MSG_GET_CRC16(&msg_buffer) != 
                crc16((uint8_t*)&msg_buffer, pkt_len - 2, 0)) {
        DEBUG_PRINT_VERBOSE("invalid CRC (dropped)");
        msg_inv++;
      } else if(msg_buffer.header.type == (MIN_MSG_TYPE | MSG_TYPE_AE_EVENT)) {
        DEBUG_PRINT_INFO("AE event received (node: %u, event_id: %u, "
                         "timestamp: %llu)",
                         msg_buffer.header.device_id,
                         msg_buffer.ae_evt.event_id, 
                         msg_buffer.ae_evt.generation_time);
        if(msg_buffer.header.device_id == 32) {
          ae_timestamps[0] = msg_buffer.ae_evt.generation_time;
        } else if (msg_buffer.header.device_id == 20) {
          ae_timestamps[1] = msg_buffer.ae_evt.generation_time;
        }
        /* forward the packet to the app processor */
        BOLT_WRITE((uint8_t*)&msg_buffer, pkt_len);
        msg_cnt++;
      } else if(msg_buffer.header.type == (MIN_MSG_TYPE | MSG_TYPE_AE_DATA)) {
        /* forward the packet to the app processor */
        BOLT_WRITE((uint8_t*)&msg_buffer, pkt_len);
        msg_cnt++;
      }
    } else {
      break;
    }
  }
  if(msg_cnt) {
    DEBUG_PRINT_INFO("%u msg rcvd and forwarded to Bolt, %u msg dropped", 
                     msg_cnt, msg_inv);
  }
  
  if(ae_timestamps[0] != 0 && ae_timestamps[1] != 0) {
    /* estimate the distance to the AE source 
     * speed of sound in stones:
     * - thick granite: assume ~3000m/s
     * - thin stone: assume ~1600m/s */
    uint16_t diff;
    uint16_t closer_to_node = 0;
    if(ae_timestamps[0] > ae_timestamps[1]) {
      diff = (uint16_t)(ae_timestamps[0] - ae_timestamps[1]);
      closer_to_node = 20;
    } else {
      diff = (uint16_t)(ae_timestamps[1] - ae_timestamps[0]);
      closer_to_node = 32;
    }
    if(diff == 0) {
      closer_to_node = 0;
    }
    DEBUG_PRINT_INFO("diff: %dus (~ %umm), event was closer to node %u, pos: %dmm",
                     diff, 
                     18 * diff / 10,
                     closer_to_node,
                     18 * diff / 20 * (closer_to_node == 32 ? -1 : 1));
  }
  
#if !TIMESYNC_INTERRUPT_BASED
  rtimer_clock_t bolt_timestamp = 0;
  if(bolt_handle_timereq(&bolt_timestamp)) { 
    msg_buffer.header.device_id   = node_id;
    msg_buffer.header.type        = MIN_MSG_TYPE | MSG_TYPE_TIMESYNC;
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
  }
    
#if !TIMESYNC_INTERRUPT_BASED
  rtimer_clock_t bolt_timestamp = 0;
  if(bolt_handle_timereq(&bolt_timestamp)) {
    rtimer_clock_t local_rx_time = 0;
    uint32_t lwb_time_secs = lwb_get_time(&local_rx_time);
    int64_t lwb_time   = (uint64_t)lwb_time_secs * RTIMER_SECOND_HF;
    int64_t ofs        = lwb_time - local_rx_time;
    bolt_timestamp     = bolt_timestamp + ofs + TIMESYNC_OFS;
    msg_buffer.header.device_id   = node_id;
    msg_buffer.header.type        = MIN_MSG_TYPE | MSG_TYPE_TIMESYNC;
    msg_buffer.header.payload_len = sizeof(timestamp_t);
    msg_buffer.timestamp          = bolt_timestamp;
    BOLT_WRITE((uint8_t*)&msg_buffer, MSG_LEN(msg_buffer));
  }
#endif /* TIMESYNC_INTERRUPT_BASED */
}
/*---------------------------------------------------------------------------*/
#if TIMESYNC_INTERRUPT_BASED
void
bolt_timereq_cb(void)
{
  /* timestamp request: calculate the timestamp and send it over BOLT */
  rtimer_clock_t now            = rtimer_now_hf();
  if(node_id == HOST_ID) {
    msg_buffer.timestamp        = now - ((uint16_t)(now & 0xffff) -
                                  BOLT_CONF_TIMEREQ_CCR);
  } else {
    rtimer_clock_t captured     = now - ((uint16_t)(now & 0xffff) -
                                  BOLT_CONF_TIMEREQ_CCR);
    /* convert the local timestamp into global (LWB) time */
    rtimer_clock_t local_t_rx   = 0;
    uint32_t lwb_time_secs      = lwb_get_time(&local_t_rx);
    int64_t  lwb_time           = (uint64_t)lwb_time_secs * RTIMER_SECOND_HF;
    int64_t  ofs                = lwb_time - local_t_rx;
    msg_buffer.timestamp        = captured + ofs + TIMESYNC_OFS;
    /* drift compensation */
    const lwb_statistics_t* lwb_stats = lwb_get_stats();
    /* drift is in HF ticks per second */
    if(lwb_stats->drift) {
      int32_t elapsed           = (int64_t)captured - local_t_rx;
      int16_t drift_comp        = (int16_t)(elapsed *
                                  (int32_t)lwb_stats->drift /
                                  (int32_t)RTIMER_SECOND_HF); 
      msg_buffer.timestamp     -= drift_comp;
      DEBUG_PRINT_INFO("drift: %d, elapsed: %ld, comp: %d)", 
                       lwb_stats->drift, elapsed, drift_comp);
    }
  }
  /* scale to microseconds */
  msg_buffer.timestamp          = msg_buffer.timestamp * 1000000 /
                                  RTIMER_SECOND_HF;
  /* compose message and send over Bolt */
  msg_buffer.header.device_id   = node_id;
  msg_buffer.header.type        = MIN_MSG_TYPE | MSG_TYPE_TIMESYNC;
  msg_buffer.header.payload_len = sizeof(timestamp_t);
  uint16_t crc = crc16((uint8_t*)&msg_buffer, MSG_LEN(msg_buffer) - 2, 0);
  MSG_SET_CRC16(&msg_buffer, crc);
  BOLT_WRITE((uint8_t*)&msg_buffer, MSG_LEN(msg_buffer));
  DEBUG_PRINT_INFO("timestamp sent (%llu)", 
                   msg_buffer.timestamp);
}
#endif /* TIMESYNC_INTERRUPT_BASED */
/*---------------------------------------------------------------------------*/
PROCESS(bolt_process, "Bolt Task");
PROCESS_THREAD(bolt_process, ev, data) 
{  
  /* uint16_t to avoid aliasing issues */
  static uint16_t bolt_buffer[(BOLT_CONF_MAX_MSG_LEN + 1) / 2];

  PROCESS_BEGIN();
  
  while(1) {  
    TASK_SUSPENDED;
    PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_POLL);
    TASK_ACTIVE;
        
    uint16_t max_ops = 20, cnt = 0, bytes = 0;
    while(BOLT_DATA_AVAILABLE && max_ops) {
      uint8_t msg_len = 0;
      BOLT_READ((uint8_t*)bolt_buffer, msg_len);
      if(msg_len) {
        /* correct the message length (cut excess characters) */
        msg_len = MSG_LEN_PTR((message_min_t*)bolt_buffer);
        if(msg_len > 2 && msg_len <= BOLT_CONF_MAX_MSG_LEN) {
          /* check the CRC -> skip, takes too long */
          /*if(MSG_GET_CRC16((message_min_t*)bolt_buffer) != 
            crc16((uint8_t*)bolt_buffer, msg_len - 2, 0)) {
            DEBUG_PRINT_WARNING("invalid CRC (dropped)");
          } else {*/
            lwb_send_pkt(0, 1, (uint8_t*)bolt_buffer, msg_len);
            /* wait for the DMA transfer to complete */
            xmem_wait_until_ready();
          //}
          cnt++;
          bytes += msg_len;
        } else {
          DEBUG_PRINT_WARNING("invalid msg length (dropped)");
        }
      }
      max_ops--;
    }
    if(cnt) {
      DEBUG_PRINT_INFO("%u message(s) read from Bolt (%db)", cnt, bytes);
    }
  }
  
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
PROCESS(app_process, "Application Task");
AUTOSTART_PROCESSES(&app_process);
PROCESS_THREAD(app_process, ev, data) 
{  
  PROCESS_BEGIN();

  /* error checks */
  if(sizeof(message_min_t) != MSG_PKT_LEN) {
    DEBUG_PRINT_MSG_NOW("ERROR: message_min_t is too long");
    while(1);
  }
  /* start the LWB thread with a pre and post processing task */
  lwb_start(&bolt_process, &app_process);
  process_start(&bolt_process, NULL);

#if TIMESYNC_INTERRUPT_BASED
  bolt_set_timereq_callback(bolt_timereq_cb);
#endif /* TIMESYNC_INTERRUPT_BASED */
  
  /* main loop of the application task */
  while(1) {
    /* the app task should not do anything until it is explicitly granted 
     * permission (by receiving a poll event) by the LWB task */
    TASK_SUSPENDED;
    PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_POLL);
    TASK_ACTIVE;      /* application task runs now */
        
    if(HOST_ID == node_id) {
      host_run();
    } else {
      source_run();
    }

  #if LWB_CONF_USE_LF_FOR_WAKEUP
    LWB_BEFORE_DEEPSLEEP();     /* prepare to go to LPM3 */
  #endif /* LWB_CONF_USE_LF_FOR_WAKEUP */
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
