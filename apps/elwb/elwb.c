/*
 * Copyright (c) 2016, Swiss Federal Institute of Technology (ETH Zurich).
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *  notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *  notice, this list of conditions and the following disclaimer in the
 *  documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *  contributors may be used to endorse or promote products derived
 *  from this software without specific prior written permission.
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

/**
 * @file
 *
 * a modified implementation of the Low-Power Wireless Bus called e-LWB
 * (event-based/triggered LWB)
 * 
 * header length is 0, neither recipient node ID nor stream ID are required 
 * since all the data flows to the sinks
 */
 
#include "contiki.h"


#if LWB_VERSION == 0

/* parameter checks */
#if !defined(LWB_SCHED_AE)
#error "LWB_MOD only support the 'AE' scheduler"
#endif

/*---------------------------------------------------------------------------*/
#define LWB_CONF_PERIOD_SCALE       100         /* also change in sched-ae.c */
#define LWB_CONF_PERIOD_MIN         4

/* expected packet length of a slot request */         
#ifndef LWB_CONF_SRQ_PKT_LEN
#define LWB_CONF_SRQ_PKT_LEN        1 
#endif /* LWB_CONF_SRQ_PKT_LEN */
/*---------------------------------------------------------------------------*/
/* internal sync state of the LWB */
typedef enum {
  BOOTSTRAP = 0,
  SYNCED,
  UNSYNCED,
  UNSYNCED2,
  NUM_OF_SYNC_STATES
} lwb_sync_state_t;
/*---------------------------------------------------------------------------*/
typedef enum {
  EVT_SCHED_RCVD = 0,
  EVT_SCHED_MISSED,
  NUM_OF_SYNC_EVENTS
} sync_event_t; 
/*---------------------------------------------------------------------------*/
/**
 * @brief the finite state machine for the time synchronization on a source 
 * node the next state can be retrieved from the current state (column) and
 * the latest event (row)
 * @note  undefined transitions force the SM to go back into bootstrap
 */
static const 
lwb_sync_state_t next_state[NUM_OF_SYNC_EVENTS][NUM_OF_SYNC_STATES] = 
{/* STATES:                                         EVENTS:         */
 /* BOOTSTRAP, SYNCED,   UNSYNCED,  UNSYNCED2                       */
  { SYNCED,    SYNCED,   SYNCED,    SYNCED    }, /* schedule rcvd   */
  { BOOTSTRAP, UNSYNCED, UNSYNCED2, BOOTSTRAP }  /* schedule missed */
};
/* note: syn2 = already synced */
static const char* lwb_sync_state_to_string[NUM_OF_SYNC_STATES] = 
{ "BOOTSTRAP", "SYN", "USYN", "USYN2" };
static const uint32_t guard_time[NUM_OF_SYNC_STATES] = {
/*BOOTSTRAP,        SYNCED,           UNSYNCED,           UNSYNCED2 */
  LWB_CONF_T_GUARD, LWB_CONF_T_GUARD, LWB_CONF_T_GUARD_1, LWB_CONF_T_GUARD_2
};
/*---------------------------------------------------------------------------*/
#ifdef LWB_CONF_TASK_ACT_PIN
  #define LWB_TASK_RESUMED        PIN_SET(LWB_CONF_TASK_ACT_PIN)
  #define LWB_TASK_SUSPENDED      PIN_CLR(LWB_CONF_TASK_ACT_PIN)
#else
  #define LWB_TASK_RESUMED     
  #define LWB_TASK_SUSPENDED  
#endif
/*---------------------------------------------------------------------------*/
#define LWB_T_SLOT_START(i)       ((LWB_CONF_T_SCHED + LWB_CONF_T_GAP) + \
                                   (t_slot + LWB_CONF_T_GAP) * i)
#define LWB_DATA_RCVD             (glossy_get_n_rx() > 0)
#define RTIMER_CAPTURE            (t_now = rtimer_now_hf())
#define RTIMER_ELAPSED            ((rtimer_now_hf() - t_now) * 1000 / 3250)    
#define GET_EVENT                 (glossy_is_t_ref_updated() ? \
                                   EVT_SCHED_RCVD : EVT_SCHED_MISSED)
/*---------------------------------------------------------------------------*/
#define LWB_SEND_SCHED() \
{\
  glossy_start(node_id, (uint8_t *)&schedule, schedule_len, \
               LWB_CONF_TX_CNT_SCHED, GLOSSY_WITH_SYNC, GLOSSY_WITH_RF_CAL);\
  LWB_WAIT_UNTIL(rt->time + LWB_CONF_T_SCHED);\
  glossy_stop();\
}   
#define LWB_RCV_SCHED() \
{\
  glossy_start(GLOSSY_UNKNOWN_INITIATOR, (uint8_t *)&schedule, payload_len, \
               LWB_CONF_TX_CNT_SCHED, GLOSSY_WITH_SYNC, GLOSSY_WITH_RF_CAL);\
  LWB_WAIT_UNTIL(rt->time + LWB_CONF_T_SCHED + t_guard);\
  glossy_stop();\
}   
#define LWB_SEND_PACKET() \
{\
  glossy_start(node_id, (uint8_t*)glossy_payload, payload_len, \
               LWB_CONF_TX_CNT_DATA, GLOSSY_WITHOUT_SYNC, \
               GLOSSY_WITHOUT_RF_CAL);\
  LWB_WAIT_UNTIL(rt->time + t_slot);\
  glossy_stop();\
}
#define LWB_RCV_PACKET() \
{\
  glossy_start(GLOSSY_UNKNOWN_INITIATOR, (uint8_t*)glossy_payload, \
               payload_len, \
               LWB_CONF_TX_CNT_DATA, GLOSSY_WITHOUT_SYNC, \
               GLOSSY_WITHOUT_RF_CAL);\
  LWB_WAIT_UNTIL(rt->time + t_slot + t_guard);\
  glossy_stop();\
}
/*---------------------------------------------------------------------------*/
/* suspend the LWB proto-thread until the rtimer reaches the specified time */
#define LWB_WAIT_UNTIL(time) \
{\
  rtimer_schedule(LWB_CONF_RTIMER_ID, time, 0, callback_func);\
  LWB_TASK_SUSPENDED;\
  PT_YIELD(&lwb_pt);\
  LWB_TASK_RESUMED;\
}
/* same as LWB_WAIT_UNTIL, but use the LF timer to schedule the wake-up */
#define LWB_LF_WAIT_UNTIL(time) \
{\
  rtimer_schedule(LWB_CONF_LF_RTIMER_ID, time, 0, callback_func);\
  LWB_TASK_SUSPENDED;\
  PT_YIELD(&lwb_pt);\
  LWB_AFTER_DEEPSLEEP();\
  LWB_TASK_RESUMED;\
}
#define LWB_UPDATE_SYNC_STATE \
{\
  /* get the new state based on the event */\
  sync_state = next_state[GET_EVENT][sync_state];\
  t_guard = guard_time[sync_state];         /* adjust the guard time */\
  if(BOOTSTRAP == sync_state) {\
    continue;  /* something went wrong */\
  }\
  if(sync_state == UNSYNCED) {\
    stats.unsynced_cnt++;\
  }\
}
#ifndef LWB_BEFORE_DEEPSLEEP
#define LWB_BEFORE_DEEPSLEEP() 
#endif /* LWB_PREPARE_DEEPSLEEP */
#ifndef LWB_AFTER_DEEPSLEEP
#define LWB_AFTER_DEEPSLEEP()
#endif /* LWB_AFTER_DEEPSLEEP */
/*---------------------------------------------------------------------------*/
static struct pt        lwb_pt;
static struct process*  post_proc;
static void             (*pre_proc)(void);
static lwb_sync_state_t sync_state;
static rtimer_clock_t   rx_timestamp;
static uint32_t         global_time;
static lwb_statistics_t stats = { 0 };
static uint8_t          running = 0;
/* allocate memory in the SRAM (+1 to store the message length) */
static uint8_t          in_buffer_mem[LWB_CONF_IN_BUFFER_SIZE * 
                                      (LWB_CONF_MAX_DATA_PKT_LEN + 1)];  
static uint8_t          out_buffer_mem[LWB_CONF_OUT_BUFFER_SIZE * 
                                       (LWB_CONF_MAX_DATA_PKT_LEN + 1)]; 
FIFO(in_buffer, LWB_CONF_MAX_DATA_PKT_LEN + 1, LWB_CONF_IN_BUFFER_SIZE);
FIFO(out_buffer, LWB_CONF_MAX_DATA_PKT_LEN + 1, LWB_CONF_OUT_BUFFER_SIZE);
/*---------------------------------------------------------------------------*/
/* store a received message in the incoming queue, returns 1 if successful, 
 * 0 otherwise */
uint8_t 
lwb_in_buffer_put(const uint8_t * const data, uint8_t len)
{  
  if(len > LWB_CONF_MAX_DATA_PKT_LEN) {
    len = LWB_CONF_MAX_DATA_PKT_LEN;
    DEBUG_PRINT_WARNING("received data packet is too big"); 
  }
  /* received messages will have the max. length LWB_CONF_MAX_DATA_PKT_LEN */
  uint32_t pkt_addr = fifo_put(&in_buffer);
  if(FIFO_ERROR != pkt_addr) {
    /* copy the data into the queue */
    uint8_t* next_msg = (uint8_t*)((uint16_t)pkt_addr);
    memcpy(next_msg, data, len);
    /* last byte holds the payload length */
    *(next_msg + LWB_CONF_MAX_DATA_PKT_LEN) = len;    
    return 1;
  }
  DEBUG_PRINT_VERBOSE("in queue full");
  return 0;
}
/*---------------------------------------------------------------------------*/
/* fetch the next 'ready-to-send' message from the outgoing queue
 * returns the message length in bytes */
uint8_t 
lwb_out_buffer_get(uint8_t* out_data)
{   
  /* messages have the max. length LWB_CONF_MAX_DATA_PKT_LEN and are already
   * formatted */
  uint32_t pkt_addr = fifo_get(&out_buffer);
  if(FIFO_ERROR != pkt_addr) {
    /* assume pointers are always 16-bit */
    uint8_t* next_msg = (uint8_t*)((uint16_t)pkt_addr);  
    memcpy(out_data, next_msg, *(next_msg + LWB_CONF_MAX_DATA_PKT_LEN));
    return *(next_msg + LWB_CONF_MAX_DATA_PKT_LEN);
  }
  DEBUG_PRINT_VERBOSE("out queue empty");
  return 0;
}
/*---------------------------------------------------------------------------*/
/* puts a message into the outgoing queue, returns 1 if successful, 
 * 0 otherwise;
 * needs to have all 4 parameters to be compatible with lwb.h */
uint8_t
lwb_send_pkt(uint16_t recipient,
             uint8_t stream_id,
             const uint8_t* const data,
             uint8_t len)
{
  /* data has the max. length LWB_CONF_MAX_DATA_PKT_LEN, lwb header needs 
   * to be added before the data is inserted into the queue */
  if(len > LWB_CONF_MAX_DATA_PKT_LEN || !data) {
    return 0;
  }
  uint32_t pkt_addr = fifo_put(&out_buffer);
  if(FIFO_ERROR != pkt_addr) {
    /* assume pointers are 16-bit */
    uint8_t* next_msg = (uint8_t*)((uint16_t)pkt_addr);
    *(next_msg + LWB_CONF_MAX_DATA_PKT_LEN) = len;
    memcpy(next_msg, data, len);
    return 1;
  }
  DEBUG_PRINT_VERBOSE("out queue full");
  return 0;
}
/*---------------------------------------------------------------------------*/
/* copies the oldest received message in the queue into out_data and returns 
 * the message size (in bytes); needs to have all 3 parameters to be 
 * compatible with lwb.h */
uint8_t
lwb_rcv_pkt(uint8_t* out_data,
            uint16_t * const out_node_id,
            uint8_t * const out_stream_id)
{ 
  if(!out_data) { return 0; }
  /* messages in the queue have the max. length LWB_CONF_MAX_DATA_PKT_LEN, 
   * lwb header needs to be stripped off; payload has max. length
   * LWB_CONF_MAX_DATA_PKT_LEN */
  uint32_t pkt_addr = fifo_get(&in_buffer);
  if(FIFO_ERROR != pkt_addr) {
    /* assume pointers are 16-bit */
    uint8_t* next_msg = (uint8_t*)((uint16_t)pkt_addr); 
    memcpy(out_data, next_msg, *(next_msg + LWB_CONF_MAX_DATA_PKT_LEN));
    return *(next_msg + LWB_CONF_MAX_DATA_PKT_LEN);
  }
  DEBUG_PRINT_VERBOSE("in queue empty");
  return 0;
}
/*---------------------------------------------------------------------------*/
uint8_t
lwb_get_rcv_buffer_state(void)
{
  return FIFO_CNT(&in_buffer);
}
/*---------------------------------------------------------------------------*/
uint8_t
lwb_get_send_buffer_state(void)
{
  return FIFO_CNT(&out_buffer);
}
/*---------------------------------------------------------------------------*/
const lwb_statistics_t * const
lwb_get_stats(void)
{
  return &stats;
}
/*-----------------------------------------------F----------------------------*/
lwb_conn_state_t
lwb_get_state(void)
{
  if(sync_state < SYNCED) { return LWB_STATE_INIT; }
  else if(sync_state < UNSYNCED) { return LWB_STATE_CONNECTED; }
  return LWB_STATE_CONN_LOST;
}
/*---------------------------------------------------------------------------*/
uint32_t 
lwb_get_time(rtimer_clock_t* reception_time)
{
  if(reception_time) {
    *reception_time = rx_timestamp;
  }
  return global_time;
}
/*---------------------------------------------------------------------------*/
/**
 * @brief thread of the host node
 */
PT_THREAD(lwb_thread_host(rtimer_t *rt)) 
{  
  /* all variables must be static */
  static lwb_schedule_t schedule;
  static rtimer_clock_t t_start; 
  static rtimer_clock_t t_now;
#if LWB_CONF_USE_LF_FOR_WAKEUP
  static rtimer_clock_t t_start_lf;
#endif /* LWB_CONF_USE_LF_FOR_WAKEUP */
  /* packet buffer: use uint16_t to force word boundary alignment */
  static uint16_t glossy_payload[(LWB_CONF_MAX_PKT_LEN + 1) / 2];
  /* constant guard time for the host */
  static const uint32_t t_guard = LWB_CONF_T_GUARD; 
  static uint32_t t_preprocess = 0;
  static uint32_t t_slot = LWB_CONF_T_DATA;
  static uint16_t curr_period = 0;
  static uint16_t srq_cnt = 0;
  static uint16_t i = 0;
  static uint8_t slot_idx;
  static uint8_t streams_to_update[LWB_CONF_MAX_DATA_SLOTS];
  static uint8_t schedule_len, 
                 payload_len;
  static int8_t  glossy_rssi = 0;
  static const void* callback_func = lwb_thread_host;
  
  PT_BEGIN(&lwb_pt);
    
  memset(&schedule, 0, sizeof(schedule));
  
  /* initialization specific to the host node */
  schedule_len = lwb_sched_init(&schedule);
  sync_state = SYNCED;  /* the host is always 'synced' */
  running = 1;
  
  rtimer_reset();
  rt->time = 0;
  
  while(1) {
      
  #if LWB_CONF_USE_LF_FOR_WAKEUP
    t_start_lf = rt->time; 
    rt->time = rtimer_now_hf();
  #endif /* LWB_CONF_USE_LF_FOR_WAKEUP */
    
  #if LWB_CONF_T_PREPROCESS
    if(t_preprocess) {
      if(pre_proc) {
        pre_proc();
      }
      /* use the HF osc. here to schedule the next wakeup! */
      LWB_WAIT_UNTIL(rt->time + 
                     (LWB_CONF_T_PREPROCESS * RTIMER_SECOND_HF / 1000));
    #if LWB_CONF_USE_LF_FOR_WAKEUP 
      t_start_lf += t_preprocess;
    #endif /* LWB_CONF_USE_LF_FOR_WAKEUP */
      t_preprocess = 0; /* reset value */
    }    
  #endif /* LWB_CONF_T_PREPROCESS */
    
    t_start = rt->time;
        
    /* --- COMMUNICATION ROUND STARTS --- */
    
    global_time = schedule.time;
    rx_timestamp = t_start;
    
    LWB_SEND_SCHED();
   
    glossy_rssi = glossy_get_rssi(0);
    stats.relay_cnt = glossy_get_relay_cnt_first_rx();
    slot_idx = 0;     /* reset the packet counter */    
        
    /* --- DATA SLOTS --- */
    
    if(LWB_SCHED_HAS_DATA_SLOT(&schedule)) {      
      /* adjust T_DATA (modification to original LWB) */
      if(LWB_SCHED_HAS_SACK_SLOT(&schedule)) {
        /* this is a data round */
        t_slot = LWB_CONF_T_DATA;
      } else {
        t_slot = LWB_CONF_T_CONT;
      }
      for(i = 0; i < LWB_SCHED_N_SLOTS(&schedule); i++, slot_idx++) {
        streams_to_update[i] = 0;
        /* is this our slot? Note: slots assigned to node ID 0 always belong 
         * to the host */
        if(schedule.slot[i] == 0 || schedule.slot[i] == node_id) {
          /* send a data packet (if there is any) */
          payload_len = lwb_out_buffer_get((uint8_t*)glossy_payload);
          if(payload_len) { 
            /* note: stream ID is irrelevant here */
            /* wait until the data slot starts */
            LWB_WAIT_UNTIL(t_start + LWB_T_SLOT_START(slot_idx));  
            LWB_SEND_PACKET();
            DEBUG_PRINT_VERBOSE("data packet sent (%ub)", payload_len);
          }
        } else {
          if(t_slot == LWB_CONF_T_CONT) {
            /* it's a request round */
            payload_len = LWB_CONF_SRQ_PKT_LEN;
          } else {
            payload_len = GLOSSY_UNKNOWN_PAYLOAD_LEN;
          }
          /* wait until the data slot starts */
          LWB_WAIT_UNTIL(t_start + LWB_T_SLOT_START(slot_idx) - t_guard);
          LWB_RCV_PACKET();  /* receive a data packet */
          payload_len = glossy_get_payload_len();
          if(LWB_DATA_RCVD) {
            if(t_slot == LWB_CONF_T_CONT) {
              /* stream request! */
              uint16_t srq[2] = { schedule.slot[i], glossy_payload[0] };
              lwb_sched_proc_srq((lwb_stream_req_t*)srq);
            } else {
              /* measure the time it takes to process the received message */
              RTIMER_CAPTURE;
              streams_to_update[i] = 1;
              DEBUG_PRINT_VERBOSE("data received from node %u (%ub)", 
                                  schedule.slot[i], payload_len);
              lwb_in_buffer_put((uint8_t*)glossy_payload, payload_len);
              /* update statistics */
              stats.rx_total += payload_len;
              stats.pck_cnt++;
              /* measure time (must always be smaller than LWB_CONF_T_GAP!) */
              stats.t_proc_max = MAX((uint16_t)RTIMER_ELAPSED, 
                                     stats.t_proc_max);
            }
          } else {
            DEBUG_PRINT_VERBOSE("no data received from node %u", 
                                schedule.slot[i]);
          }
        }
      }
    }
    
    /* --- CONTENTION SLOT --- */
    
    if(LWB_SCHED_HAS_CONT_SLOT(&schedule)) {
      t_slot = LWB_CONF_T_CONT;
      payload_len = LWB_CONF_SRQ_PKT_LEN;
      glossy_payload[0] = 0;
      /* wait until the slot starts, then receive the packet */
      LWB_WAIT_UNTIL(t_start + LWB_T_SLOT_START(slot_idx) - t_guard);
      LWB_RCV_PACKET();
      if(LWB_DATA_RCVD && glossy_payload[0] != 0) {
        /* process the request only if there is a valid node ID */
        DEBUG_PRINT_INFO("request received from node %u", glossy_payload[0]);
        lwb_sched_proc_srq((lwb_stream_req_t*)glossy_payload);
      }
      slot_idx++;   /* increment the packet counter */
      if(glossy_get_n_rx_started()) {
        /* set the period to the min. value to notify the scheduler that at 
         * least one nodes wants to request a stream (has data to send) */
        schedule.period = LWB_CONF_PERIOD_MIN;
        srq_cnt++;
      }
    }

    /* COMPUTE NEW SCHEDULE */
    
    RTIMER_CAPTURE;
    i = schedule.n_slots;
    /* store the current period (required to schedule the next wakeup) */
    curr_period = schedule.period;
    schedule_len = lwb_sched_compute(&schedule, streams_to_update, 0);
    stats.t_sched_max = MAX((uint16_t)RTIMER_ELAPSED, stats.t_sched_max);
    slot_idx++; /* increment to have more time for the schedule computation */
  
    /* --- 2ND SCHEDULE --- */
    
    if(i & 0x4000) { 
      /* send the 2nd schedule only if there was a contention slot */
      payload_len = 2;
      glossy_payload[0] = curr_period;
      LWB_WAIT_UNTIL(t_start + LWB_T_SLOT_START(slot_idx));
      LWB_SEND_PACKET();    /* send as normal packet! saves energy */
    }
    
    /* --- COMMUNICATION ROUND ENDS --- */
    /* time for other computations */
        
    /* poll the other processes to allow them to run after the LWB task was 
     * suspended (note: the polled processes will be executed in the inverse
     * order they were started/created) */
    if(curr_period > LWB_CONF_SCHED_PERIOD_IDLE * LWB_CONF_PERIOD_SCALE / 2) {
      /* print out some stats */
      DEBUG_PRINT_INFO("%lu T=%u n=%u|%u ts=%u srq=%u p=%u per=%d rssi=%ddBm", 
                      global_time,
                      schedule.period / LWB_CONF_PERIOD_SCALE,
                      schedule.n_slots & 0x3fff,
                      schedule.n_slots >> 14,
                      stats.t_sched_max, 
                      srq_cnt, 
                      stats.pck_cnt,
                      glossy_get_per(),
                      glossy_rssi);
    
      debug_print_poll();
      if(post_proc) {
        /* will be executed before the debug print task */
        process_poll(post_proc);    
      }
    #if LWB_CONF_T_PREPROCESS
     #if LWB_CONF_USE_LF_FOR_WAKEUP
      t_preprocess = (LWB_CONF_T_PREPROCESS * RTIMER_SECOND_LF / 1000);
     #else /* LWB_CONF_USE_LF_FOR_WAKEUP */
      t_preprocess = (LWB_CONF_T_PREPROCESS * RTIMER_SECOND_HF / 1000);      
     #endif /* LWB_CONF_USE_LF_FOR_WAKEUP */
    #endif /* LWB_CONF_T_PREPROCESS */
    }

    if(!running) {
      // yield LWB task indefinitely
      LWB_TASK_SUSPENDED;
      PT_YIELD(&lwb_pt);
      t_preprocess = 0;
      continue;
    }
    
  #if LWB_CONF_USE_LF_FOR_WAKEUP
    /* suspend this task and wait for the next round */
    LWB_LF_WAIT_UNTIL(t_start_lf + curr_period * RTIMER_SECOND_LF / 
                      LWB_CONF_PERIOD_SCALE - t_preprocess);
  #else /* LWB_CONF_USE_LF_FOR_WAKEUP */
    LWB_WAIT_UNTIL(t_start + (curr_period * RTIMER_SECOND_HF) /
                   LWB_CONF_PERIOD_SCALE - t_preprocess);      
  #endif /* LWB_CONF_USE_LF_FOR_WAKEUP */
  }
  
  PT_END(&lwb_pt);
}
/*---------------------------------------------------------------------------*/
/**
 * @brief declaration of the protothread (source node)
 */
PT_THREAD(lwb_thread_src(rtimer_t *rt)) 
{  
  /* all variables must be static */
  static lwb_schedule_t schedule;
  static rtimer_clock_t t_ref;
#if LWB_CONF_USE_LF_FOR_WAKEUP
  static rtimer_clock_t t_ref_lf;
  static rtimer_clock_t last_synced_lf;
#else
  static rtimer_clock_t t_ref_last;
#endif /* LWB_CONF_USE_LF_FOR_WAKEUP */
  /* packet buffer: use uint16_t to force word boundary alignment */
  static uint16_t glossy_payload[(LWB_CONF_MAX_PKT_LEN + 1) / 2];
  static uint32_t t_guard;                  /* 32-bit is enough for t_guard! */
  static uint32_t t_slot = LWB_CONF_T_DATA;
  static uint32_t t_preprocess = 0;
  static uint8_t slot_idx;
  static uint8_t payload_len;
  static const void* callback_func = lwb_thread_src;
  
  PT_BEGIN(&lwb_pt);
  
  memset(&schedule, 0, sizeof(schedule)); 
  
  sync_state = BOOTSTRAP;
  running = 1;
  
  while(1) {
    
  #if LWB_CONF_USE_LF_FOR_WAKEUP
    rt->time = rtimer_now_hf();        /* overwrite LF with HF timestamp */
  #endif /* LWB_CONF_USE_LF_FOR_WAKEUP */
    
  #if LWB_CONF_T_PREPROCESS
    if(t_preprocess) {
      if(pre_proc) {
        pre_proc();
      }
      /* use the HF osc. to schedule the next wakeup! */
      LWB_WAIT_UNTIL(rt->time + 
                     (LWB_CONF_T_PREPROCESS * RTIMER_SECOND_HF / 1000));
      t_preprocess = 0;
    }
  #endif /* LWB_CONF_T_PREPROCESS */
    
    /* --- COMMUNICATION ROUND STARTS --- */
    
    if(sync_state == BOOTSTRAP) {
      DEBUG_PRINT_MSG_NOW("BOOTSTRAP ");
      stats.bootstrap_cnt++;
      //node_registered = 0;
      /* synchronize first! wait for the first schedule... */
      payload_len = LWB_SCHED_PKT_HEADER_LEN;   /* empty schedule */
      do {
        LWB_RCV_SCHED();
        if((rtimer_now_hf() - t_ref) > LWB_CONF_T_SILENT) {
          DEBUG_PRINT_MSG_NOW("communication timeout, going to sleep...");
          running = 0;
          LWB_BEFORE_DEEPSLEEP();
          LWB_LF_WAIT_UNTIL(rtimer_now_lf() + LWB_CONF_T_DEEPSLEEP);
          t_ref = rtimer_now_hf();
          running = 1;
          DEBUG_PRINT_MSG_NOW("BOOTSTRAP ");
          /* alternative: implement a host failover policy */
        }
      } while(!glossy_is_t_ref_updated());
      /* schedule received! */
      putchar('\r');
      putchar('\n');
    } else {
      /* tell Glossy how many bytes we expect */
      payload_len = GLOSSY_UNKNOWN_PAYLOAD_LEN;
      if(schedule.period > 
         LWB_CONF_SCHED_PERIOD_IDLE * LWB_CONF_PERIOD_SCALE / 2) {
        payload_len = LWB_SCHED_PKT_HEADER_LEN;   /* empty schedule */
      }
      LWB_RCV_SCHED();  
    }
    stats.glossy_snr = glossy_get_snr();
                   
    /* update the sync state machine (compute new sync state and update 
     * t_guard) */
    LWB_UPDATE_SYNC_STATE;  
     
    if(glossy_is_t_ref_updated()) {
      /* HF timestamp of first RX; subtract a constant offset */
      t_ref = glossy_get_t_ref() - LWB_CONF_T_REF_OFS;
      
    #if LWB_CONF_USE_LF_FOR_WAKEUP
      /* estimate t_ref_lf by subtracting the elapsed time since t_ref: */
      rtimer_clock_t hf_now;
      rtimer_now(&hf_now, &t_ref_lf);
      t_ref_lf -= (uint32_t)(hf_now - t_ref) / (uint32_t)RTIMER_HF_LF_RATIO;
    #endif /* LWB_CONF_USE_LF_FOR_WAKEUP */
      if(schedule.period == 
         LWB_CONF_SCHED_PERIOD_IDLE * LWB_CONF_PERIOD_SCALE) {
        /* only update the timestamp during the idle period */
        global_time = schedule.time;
    #if LWB_CONF_USE_LF_FOR_WAKEUP
        last_synced_lf = t_ref_lf;    
     #endif /* LWB_CONF_USE_LF_FOR_WAKEUP */
        rx_timestamp = t_ref;
      }
    } else {
      DEBUG_PRINT_WARNING("schedule missed");
      /* we can only estimate t_ref and t_ref_lf */
      if(sync_state == UNSYNCED) {
    #if LWB_CONF_USE_LF_FOR_WAKEUP
        t_ref_lf = last_synced_lf;      /* restore the last known sync point */
    #else /* LWB_CONF_USE_LF_FOR_WAKEUP */
        t_ref = rx_timestamp;
    #endif  /* LWB_CONF_USE_LF_FOR_WAKEUP */
        if(schedule.period > 
           LWB_CONF_SCHED_PERIOD_IDLE * LWB_CONF_PERIOD_SCALE / 2) {
    #if LWB_CONF_USE_LF_FOR_WAKEUP
          t_ref_lf += LWB_CONF_SCHED_PERIOD_IDLE * RTIMER_SECOND_LF;
    #else /* LWB_CONF_USE_LF_FOR_WAKEUP */
          t_ref += LWB_CONF_SCHED_PERIOD_IDLE * RTIMER_SECOND_HF;
    #endif /* LWB_CONF_USE_LF_FOR_WAKEUP */
        }
        schedule.period = LWB_CONF_SCHED_PERIOD_IDLE * LWB_CONF_PERIOD_SCALE;
      } else {
        /* since HF clock was off, we need a new timestamp; subtract a const.
         * processing offset to adjust (if needed) */
    #if LWB_CONF_USE_LF_FOR_WAKEUP
        t_ref_lf += schedule.period * (RTIMER_SECOND_LF) /
                    LWB_CONF_PERIOD_SCALE;
    #else /* LWB_CONF_USE_LF_FOR_WAKEUP */
        t_ref += schedule.period * (RTIMER_SECOND_HF) /
                    LWB_CONF_PERIOD_SCALE;
    #endif /* LWB_CONF_USE_LF_FOR_WAKEUP */
      }
    }
        
    /* permission to participate in this round? */
    if(sync_state == SYNCED) {
      
      static uint8_t i;
      slot_idx = 0;   /* reset the packet counter */
      stats.relay_cnt = glossy_get_relay_cnt_first_rx();  
    
      /* --- DATA SLOTS --- */
      
      if(LWB_SCHED_HAS_DATA_SLOT(&schedule)) {
        /* set the slot duration */
        if(LWB_SCHED_HAS_SACK_SLOT(&schedule)) {
          /* this is a data round */
          t_slot = LWB_CONF_T_DATA;
        } else {
          /* it's a request round */
          t_slot = LWB_CONF_T_CONT;
          //node_registered = 0;
        }
        for(i = 0; i < LWB_SCHED_N_SLOTS(&schedule); i++, slot_idx++) {
          if(schedule.slot[i] == node_id) {
            //node_registered = 1;
            stats.t_slot_last = schedule.time;
            /* this is our data slot, send a data packet */
            if(!FIFO_EMPTY(&out_buffer)) {
              if(LWB_SCHED_HAS_SACK_SLOT(&schedule)) {
                /* it's a data round */
                payload_len = lwb_out_buffer_get((uint8_t*)glossy_payload);
              } else {
                payload_len = 1;
                /* request as many data slots as there are pkts in the queue*/
                *(uint8_t*)glossy_payload = lwb_get_send_buffer_state();
              }
              LWB_WAIT_UNTIL(t_ref + LWB_T_SLOT_START(slot_idx));
              LWB_SEND_PACKET();
              DEBUG_PRINT_VERBOSE("packet sent (%ub)", payload_len);
            } else {
              DEBUG_PRINT_VERBOSE("no message to send (data slot ignored)");
            }
          } else
          {
            payload_len = GLOSSY_UNKNOWN_PAYLOAD_LEN;
            if(!LWB_SCHED_HAS_SACK_SLOT(&schedule)) {
              /* the payload length is known in the request round */
              payload_len = LWB_CONF_SRQ_PKT_LEN;
            }
            /* receive a data packet */
            LWB_WAIT_UNTIL(t_ref + LWB_T_SLOT_START(slot_idx) - t_guard);
            LWB_RCV_PACKET();
            stats.rx_total += glossy_get_payload_len();
            stats.pck_cnt++;
          }
        }
      }
      
      /* --- CONTENTION SLOT --- */
      
      /* is there a contention slot in this round? */
      if(LWB_SCHED_HAS_CONT_SLOT(&schedule)) {
        t_slot = LWB_CONF_T_CONT;
        if(!FIFO_EMPTY(&out_buffer)) {
          /* if there is data in the output buffer, then request a slot */
          /* a slot request packet always looks the same (1 byte) */
          payload_len = LWB_CONF_SRQ_PKT_LEN;
          /* include the node ID in case this is the first request */
          glossy_payload[0] = 0;
          //if(!node_registered) {
          //  payload_len = 2;
          //  glossy_payload[0] = node_id;
          //}
          /* wait until the contention slot starts */
          LWB_WAIT_UNTIL(t_ref + LWB_T_SLOT_START(slot_idx));
          LWB_SEND_PACKET();
          DEBUG_PRINT_VERBOSE("request sent");
        } else {
          /* no request pending -> just receive / relay packets */
          payload_len = LWB_CONF_SRQ_PKT_LEN;
          LWB_WAIT_UNTIL(t_ref + LWB_T_SLOT_START(slot_idx) - t_guard);
          LWB_RCV_PACKET();
        }
        slot_idx++;   /* increment the packet counter */
      }
       
      /* increment to give the host time for the schedule computation */
      slot_idx++; 
            
      /* --- 2ND SCHEDULE --- */
      
      if(LWB_SCHED_HAS_CONT_SLOT(&schedule)) {
        /* only rcv the 2nd schedule if there was a contention slot */
        payload_len = 2;  /* we expect exactly 2 bytes */
        LWB_WAIT_UNTIL(t_ref + LWB_T_SLOT_START(slot_idx) - t_guard);
        LWB_RCV_PACKET();
        if(LWB_DATA_RCVD) {
          schedule.period = glossy_payload[0]; /* extract the updated period */
        } else {
          DEBUG_PRINT_VERBOSE("2nd schedule missed");
        }
      }
      
      /* in priciple, drift compensation is not needed due to the short round
       * periods */    
    #if !LWB_CONF_USE_LF_FOR_WAKEUP
      /* do some basic drift estimation, but only during idle periods and if 
       * state == SYNCED */
      if(schedule.period == 
         (LWB_CONF_SCHED_PERIOD_IDLE * LWB_CONF_PERIOD_SCALE)) {
        int32_t drift = ((int32_t)((t_ref - t_ref_last) - 
                   (int32_t)(LWB_CONF_SCHED_PERIOD_IDLE * RTIMER_SECOND_HF))) / 
                   (int32_t)LWB_CONF_SCHED_PERIOD_IDLE;
        t_ref_last = t_ref; 
        if(drift < 50 && drift > -50) { /* no more than 50 ticks per second */
          stats.drift = (stats.drift + drift) / 2;
        }
      }
    #endif /* LWB_CONF_USE_LF_FOR_WAKEUP */
    }
    
    /* --- COMMUNICATION ROUND ENDS --- */    
    /* time for other computations */
              
    /* poll the other processes to allow them to run after the LWB task was
     * suspended (note: the polled processes will be executed in the inverse
     * order they were started/created) */
    if(schedule.period > 
       LWB_CONF_SCHED_PERIOD_IDLE * LWB_CONF_PERIOD_SCALE / 2) {
      /* print out some stats (note: takes approx. 2ms to compose this string) */
      DEBUG_PRINT_INFO("%s %lu T=%u n=%u tp=%u p=%u r=%u b=%u "
                      "u=%u per=%d snr=%ddbm", 
                      lwb_sync_state_to_string[sync_state], 
                      //node_registered,
                      schedule.time, 
                      schedule.period / LWB_CONF_PERIOD_SCALE, 
                      LWB_SCHED_N_SLOTS(&schedule), 
                      stats.t_proc_max,
                      stats.pck_cnt,
                      stats.relay_cnt, 
                      stats.bootstrap_cnt, 
                      stats.unsynced_cnt,
                      glossy_get_per(),
                      stats.glossy_snr);
      
      debug_print_poll();
      if(post_proc) {
        process_poll(post_proc);
      }      
    #if LWB_CONF_T_PREPROCESS
     #if LWB_CONF_USE_LF_FOR_WAKEUP
      t_preprocess = (LWB_CONF_T_PREPROCESS * RTIMER_SECOND_LF / 1000);
     #else /* LWB_CONF_USE_LF_FOR_WAKEUP */
      t_preprocess = (LWB_CONF_T_PREPROCESS * RTIMER_SECOND_HF / 1000);      
     #endif /* LWB_CONF_USE_LF_FOR_WAKEUP */
    #endif /* LWB_CONF_T_PREPROCESS */
    }
    /* erase the schedule (slot allocations only) */
    memset(&schedule.slot, 0, sizeof(schedule.slot));
    
    if(!running) {
      // yield LWB task indefinitely
      LWB_TASK_SUSPENDED;
      PT_YIELD(&lwb_pt);
      t_preprocess = 0;
      sync_state = BOOTSTRAP;   // reset state machine
      continue;
    }

  #if LWB_CONF_USE_LF_FOR_WAKEUP
    LWB_LF_WAIT_UNTIL(t_ref_lf + (schedule.period * RTIMER_SECOND_LF) /
                      LWB_CONF_PERIOD_SCALE - 
                      (t_guard / RTIMER_HF_LF_RATIO) - t_preprocess);
  #else /* LWB_CONF_USE_LF_FOR_WAKEUP */
    LWB_WAIT_UNTIL(t_ref + (schedule.period * RTIMER_SECOND_HF) /
                   LWB_CONF_PERIOD_SCALE - t_guard - t_preprocess);    
  #endif /* LWB_CONF_USE_LF_FOR_WAKEUP */
  }

  PT_END(&lwb_pt);
}
/*---------------------------------------------------------------------------*/
void
lwb_pause(void)
{
  running = 0;
}
/*---------------------------------------------------------------------------*/
void
lwb_resume(void)
{
  if(!running) {
    running = 1;
    /* start in 5ms */
    if((node_id == HOST_ID)) {
      /* note: must add at least some clock ticks! */
      rtimer_schedule(LWB_CONF_RTIMER_ID, rtimer_now_hf() +
                      RTIMER_SECOND_HF / 500, 0, lwb_thread_host);
    } else {
      rtimer_schedule(LWB_CONF_RTIMER_ID, rtimer_now_hf() +
                      RTIMER_SECOND_HF / 500, 0, lwb_thread_src);
    }
  }
}
/*---------------------------------------------------------------------------*/
/* define the process control block */
PROCESS(lwb_process, "Communication Task (LWB)");
/*---------------------------------------------------------------------------*/
/* define the body (protothread) of a process */
PROCESS_THREAD(lwb_process, ev, data) 
{
  PROCESS_BEGIN();
  
  /* pass the start addresses of the memory blocks holding the queues */
  fifo_init(&in_buffer, (uint16_t)in_buffer_mem);
  fifo_init(&out_buffer, (uint16_t)out_buffer_mem); 

#ifdef LWB_CONF_TASK_ACT_PIN
  PIN_CFG_OUT(LWB_CONF_TASK_ACT_PIN);
  PIN_CLR(LWB_CONF_TASK_ACT_PIN);
#endif /* LWB_CONF_TASK_ACT_PIN */
  
  PT_INIT(&lwb_pt); /* initialize the protothread */

  lwb_resume();

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
void
lwb_start(void (*pre_lwb_func)(void), void *post_lwb_proc)
{
  pre_proc = pre_lwb_func;
  post_proc = (struct process*)post_lwb_proc;
  printf("Starting '%s'\r\n", lwb_process.name);
    
  printf("t_sched=%ums, t_data=%ums, t_cont=%ums, t_round=%ums, "
         "data=%ub, slots=%u, tx=%u, hop=%u\r\n", 
         (uint16_t)RTIMER_HF_TO_MS(LWB_CONF_T_SCHED),
         (uint16_t)RTIMER_HF_TO_MS(LWB_CONF_T_DATA),
         (uint16_t)RTIMER_HF_TO_MS(LWB_CONF_T_CONT),
         (uint16_t)RTIMER_HF_TO_MS(LWB_T_ROUND_MAX),
         LWB_CONF_MAX_DATA_PKT_LEN, 
         LWB_CONF_MAX_DATA_SLOTS, 
         LWB_CONF_TX_CNT_DATA, 
         LWB_CONF_MAX_HOPS);
  process_start(&lwb_process, NULL);
}
/*---------------------------------------------------------------------------*/

#endif /* LWB_VERSION */
