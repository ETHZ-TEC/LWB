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
 * 
 * Version: 2.0
 */

/**
 * @file
 *
 * a modified implementation of the Low-Power Wireless Bus called e-LWB
 * (event-based/triggered LWB)
 * it is a many-to-one protocol for fast data dissemination under rapidly
 * chanding loads
 * 
 * header length is 0, neither recipient node ID nor stream ID are required 
 * since all the data flows to the sinks
 */
 
#include "contiki.h"


#if LWB_VERSION == 0

/* parameter checks */
#if !defined(LWB_SCHED_ELWB)
#error "eLWB only supports the eLWB scheduler!"
#endif

/*---------------------------------------------------------------------------*/
#define LWB_PERIOD_SCALE                100   /* also change in sched-elwb.c */

/* also defined in sched-elwb.c! */
#define LWB_PERIOD_T_REQ_MIN    ((LWB_CONF_T_SCHED + \
                                 2 * LWB_CONF_T_CONT + 2 * LWB_CONF_T_GAP + \
                                 (RTIMER_SECOND_HF / 100)) / \
                                 (RTIMER_SECOND_HF / LWB_PERIOD_SCALE))
#define LWB_PERIOD_IDLE         (uint16_t)( \
                                 (uint32_t)LWB_CONF_SCHED_PERIOD_IDLE_MS * \
                                 LWB_PERIOD_SCALE / 1000)

#ifndef LWB_CONF_SCHED_PERIOD_IDLE_MS
#define LWB_CONF_SCHED_PERIOD_IDLE_MS   (LWB_CONF_SCHED_PERIOD_IDLE * 1000)
#endif /* LWB_CONF_SCHED_PERIOD_IDLE_MS */

#if LWB_CONF_HEADER_LEN != 0
#error "LWB_CONF_HEADER_LEN must be set to 0!"
#endif /* LWB_CONF_HEADER_LEN */

#if LWB_CONF_SCHED_PERIOD_IDLE_MS < 50
#error "eLWB idle period is too small!"
#endif /* LWB_CONF_SCHED_PERIOD_IDLE_MS */

/* expected packet length of a slot request */         
#ifndef LWB_CONF_SRQ_PKT_LEN
#define LWB_CONF_SRQ_PKT_LEN        1 
#endif /* LWB_CONF_SRQ_PKT_LEN */

#if LWB_CONF_SRQ_PKT_LEN != 1
#warning "LWB_CONF_SRQ_PKT_LEN should be set to 1!"
#endif /* LWB_CONF_SRQ_PKT_LEN */

/* use a bit to indicate whether it is a contention or a data round */
#define IS_DATA_ROUND(s)            LWB_SCHED_HAS_SACK_SLOT(s)
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
typedef enum {
  LWB_SCHED_STATE_IDLE = 0,
  LWB_SCHED_STATE_CONT_DET,        /* contention detected */
  LWB_SCHED_STATE_REQ,
  LWB_SCHED_STATE_DATA,
  LWB_SCHED_STATE_SUSPENDED,
  NUM_SCHED_STATES
} lwb_sched_state_t;
/*---------------------------------------------------------------------------*/
typedef struct {
  uint8_t   op;          /* pending operation: 0 = none, 1 = read, 2 = write */
  uint8_t   len;                            /* for write op: number of bytes */
  uint8_t*  notify;  /* pointer to notification byte, length will be written */
  uint8_t*  sram_ptr;                       /* local buffer (16-bit address) */
  uint32_t  xmem_addr;              /* 32-bit address in the external memory */
  /* note: 'op' and 'notify' are RW for the xmem task, whereas the other 
   *       fields are read-only! */
} xmem_task_t;
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
static const char* lwb_sync_state_to_string[NUM_OF_SYNC_STATES] = {
  "BOOTSTRAP", "SYN", "USYN", "USYN2"
};
static const uint32_t guard_time[NUM_OF_SYNC_STATES] = {
/*BOOTSTRAP,        SYNCED,           UNSYNCED,           UNSYNCED2 */
  LWB_CONF_T_GUARD, LWB_CONF_T_GUARD, LWB_CONF_T_GUARD_1, LWB_CONF_T_GUARD_2
};
static const lwb_sched_state_t update_sched_state[NUM_SCHED_STATES] = {
  LWB_SCHED_STATE_IDLE, LWB_SCHED_STATE_REQ, LWB_SCHED_STATE_DATA,
  LWB_SCHED_STATE_IDLE, LWB_SCHED_STATE_SUSPENDED
};
/*---------------------------------------------------------------------------*/
#ifdef LWB_CONF_TASK_ACT_PIN
  #define LWB_TASK_RESUMED        PIN_CLR(LWB_CONF_TASK_ACT_PIN); \
                                  PIN_SET(LWB_CONF_TASK_ACT_PIN)
  #define LWB_TASK_SUSPENDED      PIN_CLR(LWB_CONF_TASK_ACT_PIN)
#else
  #define LWB_TASK_RESUMED     
  #define LWB_TASK_SUSPENDED  
#endif
/*---------------------------------------------------------------------------*/
#define LWB_DATA_RCVD             (glossy_get_n_rx() > 0) 
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
static struct process*  pre_proc;
static lwb_sync_state_t sync_state;
static rtimer_clock_t   rx_timestamp;
static uint32_t         global_time;
static lwb_schedule_t   schedule;
static uint8_t          schedule_len;
static uint16_t         glossy_payload[(LWB_CONF_MAX_PKT_LEN + 1) / 2];
static uint8_t          payload_len;
static uint32_t         t_preprocess;
static uint32_t         t_slot;
static uint32_t         t_slot_ofs;
static uint32_t         t_guard;
static const void*      callback_func;
static lwb_statistics_t stats = { 0 };
static uint8_t          running = 0;
#if !LWB_CONF_USE_XMEM
/* allocate memory in the SRAM (+1 to store the message length) */
static uint8_t          in_buffer_mem[LWB_CONF_IN_BUFFER_SIZE * 
                                      (LWB_CONF_MAX_DATA_PKT_LEN + 1)];  
static uint8_t          out_buffer_mem[LWB_CONF_OUT_BUFFER_SIZE * 
                                       (LWB_CONF_MAX_DATA_PKT_LEN + 1)]; 
#else /* LWB_CONF_USE_XMEM */
static uint8_t          xmem_buffer[LWB_CONF_MAX_DATA_PKT_LEN + 1];
/* shared memory for process-to-protothread communication (xmem access) */
static xmem_task_t      xmem_task = { 0 };
#endif /* LWB_CONF_USE_XMEM */
FIFO(in_buffer, LWB_CONF_MAX_DATA_PKT_LEN + 1, LWB_CONF_IN_BUFFER_SIZE);
FIFO(out_buffer, LWB_CONF_MAX_DATA_PKT_LEN + 1, LWB_CONF_OUT_BUFFER_SIZE);
/*---------------------------------------------------------------------------*/
PROCESS(lwb_process, "Communication Task (eLWB)");/* process ctrl block def. */
/*---------------------------------------------------------------------------*/
/* store a received message in the incoming queue, returns 1 if successful, 
 * 0 otherwise */
uint8_t 
lwb_in_buffer_put(uint8_t* data, uint8_t len)
{  
  if(!len || len > LWB_CONF_MAX_DATA_PKT_LEN) {
    DEBUG_PRINT_WARNING("lwb: invalid packet received");
    return 0;
  }
  /* received messages will have the max. length LWB_CONF_MAX_DATA_PKT_LEN */
  uint32_t pkt_addr = fifo_put(&in_buffer);
  if(FIFO_ERROR != pkt_addr) {
#if !LWB_CONF_USE_XMEM
    /* copy the data into the queue */
    uint8_t* next_msg = (uint8_t*)((uint16_t)pkt_addr);
    memcpy(next_msg, data, len);
    /* last byte holds the payload length */
    *(next_msg + LWB_CONF_MAX_DATA_PKT_LEN) = len;    
#else /* LWB_CONF_USE_XMEM */
    if(xmem_task.op) {
      DEBUG_PRINT_ERROR("xmem task busy, operation skipped");
      return 0;
    }
    /* schedule a write operation from the external memory */
    xmem_task.op        = 2;
    xmem_task.len       = len;
    xmem_task.xmem_addr = pkt_addr;
    xmem_task.sram_ptr  = data;
    process_poll(&lwb_process);    
#endif /* LWB_CONF_USE_XMEM */
    return 1;
  }
  DEBUG_PRINT_WARNING("lwb rx queue full");
  return 0;
}
/*---------------------------------------------------------------------------*/
/* fetch the next 'ready-to-send' message from the outgoing queue
 * returns 1 if successful, 0 otherwise */
uint8_t 
lwb_out_buffer_get(uint8_t* out_data, uint8_t* out_len)
{   
  /* messages have the max. length LWB_CONF_MAX_DATA_PKT_LEN and are already
   * formatted */
  uint32_t pkt_addr = fifo_get(&out_buffer);
  if(FIFO_ERROR != pkt_addr) {
#if !LWB_CONF_USE_XMEM
    /* assume pointers are always 16-bit */
    uint8_t* next_msg = (uint8_t*)((uint16_t)pkt_addr);  
    memcpy(out_data, next_msg, *(next_msg + LWB_CONF_MAX_DATA_PKT_LEN));
    *out_len = *(next_msg + LWB_CONF_MAX_DATA_PKT_LEN);
#else /* LWB_CONF_USE_XMEM */
    if(xmem_task.op) {
      DEBUG_PRINT_ERROR("xmem task busy, operation skipped");
      return 0;
    }
    /* schedule a read operation from the external memory */
    xmem_task.op        = 1;
    xmem_task.notify    = out_len;
    xmem_task.xmem_addr = pkt_addr;
    xmem_task.sram_ptr  = out_data;
    process_poll(&lwb_process);
#endif /* LWB_CONF_USE_XMEM */
    return 1;
  }
  DEBUG_PRINT_VERBOSE("lwb tx queue empty");
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
#if !LWB_CONF_USE_XMEM
    /* assume pointers are 16-bit */
    uint8_t* next_msg = (uint8_t*)((uint16_t)pkt_addr);
    *(next_msg + LWB_CONF_MAX_DATA_PKT_LEN) = len;
    memcpy(next_msg, data, len);
#else /* LWB_CONF_USE_XMEM */
    memcpy(xmem_buffer, data, len);
    *(xmem_buffer + LWB_CONF_MAX_DATA_PKT_LEN) = len;
    xmem_wait_until_ready();
    xmem_write(pkt_addr, LWB_CONF_MAX_DATA_PKT_LEN + 1, xmem_buffer);
#endif /* LWB_CONF_USE_XMEM */
    DEBUG_PRINT_VERBOSE("msg added to lwb tx queue");
    return 1;
  }
  DEBUG_PRINT_VERBOSE("lwb tx queue full");
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
#if !LWB_CONF_USE_XMEM
    /* assume pointers are 16-bit */
    uint8_t* next_msg = (uint8_t*)((uint16_t)pkt_addr); 
    memcpy(out_data, next_msg, *(next_msg + LWB_CONF_MAX_DATA_PKT_LEN));
    return *(next_msg + LWB_CONF_MAX_DATA_PKT_LEN);
#else /* LWB_CONF_USE_XMEM */
    if(!xmem_read(pkt_addr, LWB_CONF_MAX_DATA_PKT_LEN + 1, xmem_buffer)) {
      DEBUG_PRINT_ERROR("xmem_read() failed");
      return 0;
    }
    xmem_wait_until_ready(); /* wait for the data transfer to complete */
    uint8_t msg_len = *(xmem_buffer + LWB_CONF_MAX_DATA_PKT_LEN);
    memcpy(out_data, xmem_buffer, msg_len);
    return msg_len;
#endif /* LWB_CONF_USE_XMEM */
  }
  DEBUG_PRINT_VERBOSE("lwb rx queue empty");
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
/*---------------------------------------------------------------------------*/
lwb_conn_state_t
lwb_get_state(void)
{
  if(!running) { return LWB_STATE_SUSPENDED; }
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
  /* variables specific to the host (all must be static) */
  static rtimer_clock_t t_start;
#if LWB_CONF_USE_LF_FOR_WAKEUP
  static rtimer_clock_t t_start_lf;
#endif /* LWB_CONF_USE_LF_FOR_WAKEUP */
  static uint16_t curr_period = 0;
  static uint16_t srq_cnt = 0;
  static int8_t glossy_rssi = 0;
  static lwb_sched_state_t sched_state;
  
  PT_BEGIN(&lwb_pt);

  sched_state   = LWB_SCHED_STATE_IDLE;
  callback_func = lwb_thread_host;
  t_guard       = LWB_CONF_T_GUARD;      /* constant guard time for the host */
 
  //rtimer_reset();
  //rt->time = 0;
  
  while(1) {
  
  #if LWB_CONF_T_PREPROCESS
    if(t_preprocess) {
      if(pre_proc) {
        process_poll(pre_proc);
      }
      
      /* update the schedule in case there is data to send!
      * note: this will delay the sending of the schedule by some time! */
      if(LWB_SCHED_HAS_CONT_SLOT(&schedule) && !FIFO_EMPTY(&out_buffer)) {
        schedule_len = lwb_sched_compute(&schedule, (uint8_t*)&sched_state, 
                                        lwb_get_send_buffer_state());
        DEBUG_PRINT_VERBOSE("schedule recomputed");
      }
    #if LWB_CONF_USE_LF_FOR_WAKEUP
      LWB_LF_WAIT_UNTIL(rt->time + LWB_CONF_T_PREPROCESS);
    #else      
      LWB_WAIT_UNTIL(rt->time + LWB_CONF_T_PREPROCESS);
    #endif /* LWB_CONF_USE_LF_FOR_WAKEUP */
      t_preprocess = 0; /* reset value */
    }    
  #endif /* LWB_CONF_T_PREPROCESS */
      
    /* --- COMMUNICATION ROUND STARTS --- */
        
  #if LWB_CONF_USE_LF_FOR_WAKEUP
    t_start_lf = rt->time; 
    rt->time = rtimer_now_hf();
  #endif /* LWB_CONF_USE_LF_FOR_WAKEUP */
    
    t_start = rt->time;

    /* --- SEND SCHEDULE --- */    
    LWB_SEND_SCHED();
   
    glossy_rssi     = glossy_get_rssi();
    stats.relay_cnt = glossy_get_relay_cnt_first_rx();
    t_slot_ofs      = (LWB_CONF_T_SCHED + LWB_CONF_T_GAP);
    global_time     = schedule.time;
    rx_timestamp    = t_start;
        
  #if LWB_CONF_USE_XMEM
    /* put the external memory back into active mode (takes ~500us) */
    xmem_wakeup();    
  #endif /* LWB_CONF_USE_XMEM */

    /* --- DATA SLOTS --- */
    
    if(LWB_SCHED_HAS_DATA_SLOT(&schedule)) {

    #if LWB_CONF_SCHED_COMPRESS
      lwb_sched_uncompress((uint8_t*)schedule.slot, 
                           LWB_SCHED_N_SLOTS(&schedule));
    #endif /* LWB_CONF_SCHED_COMPRESS */

      /* adjust T_DATA (modification to original LWB) */
      if(IS_DATA_ROUND(&schedule)) {
        /* this is a data round */
        t_slot = LWB_CONF_T_DATA;
      } else {
        t_slot = LWB_CONF_T_CONT;
      }
      static uint16_t i;
      for(i = 0; i < LWB_SCHED_N_SLOTS(&schedule); i++) {
        /* is this our slot? Note: slots assigned to node ID 0 always belong 
         * to the host */
        if(schedule.slot[i] == 0 || schedule.slot[i] == node_id) {
          /* send a data packet (if there is any) */
          lwb_out_buffer_get((uint8_t*)glossy_payload, &payload_len);
          if(payload_len) { 
            /* note: stream ID is irrelevant here */
            /* wait until the data slot starts */
            LWB_WAIT_UNTIL(t_start + t_slot_ofs);  
            LWB_SEND_PACKET();
            DEBUG_PRINT_VERBOSE("data packet sent (%ub)", payload_len);
          }
        } else {
          if(IS_DATA_ROUND(&schedule)) {
            payload_len = GLOSSY_UNKNOWN_PAYLOAD_LEN;
          } else {
            /* it's a request round */
            payload_len = LWB_CONF_SRQ_PKT_LEN;
          }
          /* wait until the data slot starts */
          LWB_WAIT_UNTIL(t_start + t_slot_ofs - t_guard);
          LWB_RCV_PACKET();  /* receive a data packet */
          payload_len = glossy_get_payload_len();
          if(LWB_DATA_RCVD) {
            if(!IS_DATA_ROUND(&schedule)) {
              /* stream request! */
              uint16_t srq[2] = { schedule.slot[i], glossy_payload[0] };
              lwb_sched_proc_srq((lwb_stream_req_t*)srq);
            } else {
              DEBUG_PRINT_VERBOSE("data received from node %u (%ub)", 
                                  schedule.slot[i], payload_len);
              lwb_in_buffer_put((uint8_t*)glossy_payload, payload_len);
              /* update statistics */
              stats.rx_total += payload_len;
              stats.pck_cnt++;
            }
          } else {
            DEBUG_PRINT_VERBOSE("no data received from node %u", 
                                schedule.slot[i]);
          }
        }
        t_slot_ofs += (t_slot + LWB_CONF_T_GAP);
      }
    }
    
    /* --- CONTENTION SLOT --- */
    
    if(LWB_SCHED_HAS_CONT_SLOT(&schedule)) {
      t_slot = LWB_CONF_T_CONT;
      payload_len = LWB_CONF_SRQ_PKT_LEN;
      glossy_payload[0] = 0;
      /* wait until the slot starts, then receive the packet */
      LWB_WAIT_UNTIL(t_start + t_slot_ofs - t_guard);
      LWB_RCV_PACKET();
      //if(LWB_DATA_RCVD && glossy_payload[0] != 0) {
      //  /* process the request only if there is a valid node ID */
      //  DEBUG_PRINT_INFO("request received from node %u", glossy_payload[0]);
      //  lwb_sched_proc_srq((lwb_stream_req_t*)glossy_payload);
      //}
      if(glossy_get_n_rx_started()) {
        /* set the period to the min. value to notify the scheduler that at 
         * least one nodes wants to request a stream (has data to send) */
        sched_state     = LWB_SCHED_STATE_CONT_DET;
        schedule.period = LWB_PERIOD_T_REQ_MIN + 
                          LWB_SCHED_N_SLOTS(&schedule) * 
                          (LWB_CONF_T_DATA + LWB_CONF_T_GAP) /
                          (RTIMER_SECOND_HF / LWB_PERIOD_SCALE);
        srq_cnt++;
      }
      t_slot_ofs += LWB_CONF_T_CONT + LWB_CONF_T_GAP;
  
      /* --- SEND 2ND SCHEDULE --- */
      
      /* (just a 2-byte packet to indicate a change in the round period) */    
      /* send the 2nd schedule only if there was a contention slot */
      payload_len = 2;
      glossy_payload[0] = schedule.period;
      LWB_WAIT_UNTIL(t_start + t_slot_ofs);
      LWB_SEND_PACKET();    /* send as normal packet! saves energy */
    }
    
    /* --- COMMUNICATION ROUND ENDS --- */
    /* time for other computations */
        
    /* poll the other processes to allow them to run after the LWB task was 
     * suspended (note: the polled processes will be executed in the inverse
     * order they were started/created) */
    if(sched_state == LWB_SCHED_STATE_DATA ||
       sched_state == LWB_SCHED_STATE_IDLE) {
      /* print out some stats */
      DEBUG_PRINT_INFO("t=%lu T=%u n=%u srq=%u p=%u per=%d "
                       "rssi=%ddBm", 
                       schedule.time,
                       schedule.period * (1000 / LWB_PERIOD_SCALE),
                       schedule.n_slots & 0x3fff,
                       srq_cnt, 
                       stats.pck_cnt,
                       glossy_get_per(),
                       glossy_rssi);
      
    #if LWB_CONF_USE_XMEM
      /* make sure the xmem task has a chance to run, yield for T_GAP */
      rtimer_schedule(LWB_CONF_RTIMER_ID, rtimer_now_hf() + LWB_CONF_T_GAP,
                      0, callback_func);
      LWB_TASK_SUSPENDED;
      PT_YIELD(&lwb_pt);
      LWB_TASK_RESUMED;
    #endif /* LWB_CONF_USE_XMEM */
    
      debug_print_poll();
      if(post_proc) {
        process_poll(post_proc);    
      }
    #if LWB_CONF_T_PREPROCESS
      t_preprocess = LWB_CONF_T_PREPROCESS;     
    #endif /* LWB_CONF_T_PREPROCESS */
    }
    
    /* --- COMPUTE NEW SCHEDULE (for the next round) --- */
    curr_period  = schedule.period; /* required to schedule the next wake-up */
    schedule_len = lwb_sched_compute(&schedule, (uint8_t*)&sched_state, 
                                     lwb_get_send_buffer_state());
    sched_state  = update_sched_state[sched_state];
    if(!schedule_len) {
      DEBUG_PRINT_ERROR("invalid schedule (0 bytes)");
    }

    if(!running) {
      /* suspend LWB task indefinitely */
      LWB_TASK_SUSPENDED;
      PT_YIELD(&lwb_pt);
      LWB_TASK_RESUMED;
      t_preprocess = 0;
      sched_state = LWB_SCHED_STATE_IDLE;
      continue;
    }
        
  #if LWB_CONF_USE_LF_FOR_WAKEUP
    /* suspend this task and wait for the next round */
    LWB_LF_WAIT_UNTIL(t_start_lf + curr_period * RTIMER_SECOND_LF / 
                      LWB_PERIOD_SCALE - t_preprocess);
  #else /* LWB_CONF_USE_LF_FOR_WAKEUP */
    LWB_WAIT_UNTIL(t_start + (curr_period * RTIMER_SECOND_HF) /
                   LWB_PERIOD_SCALE - t_preprocess);      
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
  /* variables specific to the source node (all must be static) */
  static rtimer_clock_t t_ref;
#if LWB_CONF_USE_LF_FOR_WAKEUP
  static rtimer_clock_t t_ref_lf;
  static rtimer_clock_t last_synced_lf;
#else
  static rtimer_clock_t t_ref_last;
#endif /* LWB_CONF_USE_LF_FOR_WAKEUP */
  
  PT_BEGIN(&lwb_pt);
  
  memset(&schedule, 0, sizeof(schedule));
  sync_state    = BOOTSTRAP;
  callback_func = lwb_thread_src;
  
  while(1) {
        
  #if LWB_CONF_T_PREPROCESS
    if(t_preprocess) {
      if(pre_proc) {
        process_poll(pre_proc);
      }
    #if LWB_CONF_USE_LF_FOR_WAKEUP
      LWB_LF_WAIT_UNTIL(rt->time + LWB_CONF_T_PREPROCESS);
    #else      
      LWB_WAIT_UNTIL(rt->time + LWB_CONF_T_PREPROCESS);
    #endif /* LWB_CONF_USE_LF_FOR_WAKEUP */
      t_preprocess = 0;
    }
  #endif /* LWB_CONF_T_PREPROCESS */
    
    /* --- COMMUNICATION ROUND STARTS --- */
    
  #if LWB_CONF_USE_LF_FOR_WAKEUP
    rt->time = rtimer_now_hf();        /* overwrite LF with HF timestamp */
  #endif /* LWB_CONF_USE_LF_FOR_WAKEUP */
            
    /* --- RECEIVE SCHEDULE --- */
    
    payload_len = GLOSSY_UNKNOWN_PAYLOAD_LEN;
    if(sync_state == BOOTSTRAP) {
      schedule.n_slots = 0;   /* reset */
      DEBUG_PRINT_MSG_NOW("BOOTSTRAP");
      stats.bootstrap_cnt++;
      /* synchronize first! wait for the first schedule... */
      do {
        /* poll the preprocess or application task */
        if(post_proc) {
          process_poll(post_proc);
        }
        /* turn radio on */
        glossy_start(GLOSSY_UNKNOWN_INITIATOR, (uint8_t *)&schedule,
                     payload_len, LWB_CONF_TX_CNT_SCHED, GLOSSY_WITH_SYNC,
                     GLOSSY_WITH_RF_CAL);
        LWB_WAIT_UNTIL(rt->time + LWB_CONF_T_SCHED);
        glossy_stop();
      } while(!glossy_is_t_ref_updated() && running);
      /* schedule received! */
    } else {
      LWB_RCV_SCHED();  
    }
    stats.glossy_snr = glossy_get_snr();
                   
  #if LWB_CONF_USE_XMEM
    /* put the external memory back into active mode (takes ~500us) */
    xmem_wakeup();    
  #endif /* LWB_CONF_USE_XMEM */

    /* --- SYNC --- */
    
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
      if(schedule.period == LWB_PERIOD_IDLE) {
        /* only update the timestamp during the idle period */
        global_time = schedule.time;
    #if LWB_CONF_USE_LF_FOR_WAKEUP
        last_synced_lf = t_ref_lf;    
     #endif /* LWB_CONF_USE_LF_FOR_WAKEUP */
        rx_timestamp = t_ref;
      }
      stats.relay_cnt = glossy_get_relay_cnt_first_rx();
      
    } else {
      DEBUG_PRINT_WARNING("schedule missed");
      /* we can only estimate t_ref and t_ref_lf */
      if(schedule.period < LWB_PERIOD_IDLE) {
        /* missed schedule was during a contention/data round -> reset t_ref */
    #if LWB_CONF_USE_LF_FOR_WAKEUP
        t_ref_lf = last_synced_lf;      /* restore the last known sync point */
    #else /* LWB_CONF_USE_LF_FOR_WAKEUP */
        t_ref = rx_timestamp;
    #endif  /* LWB_CONF_USE_LF_FOR_WAKEUP */
        if(IS_DATA_ROUND(&schedule)) {
          /* last round was a data round? -> add one period */
      #if LWB_CONF_USE_LF_FOR_WAKEUP
          t_ref_lf += LWB_CONF_SCHED_PERIOD_IDLE_MS * RTIMER_SECOND_LF / 1000;
      #else /* LWB_CONF_USE_LF_FOR_WAKEUP */
          t_ref += LWB_CONF_SCHED_PERIOD_IDLE_MS * RTIMER_SECOND_HF / 1000;
      #endif /* LWB_CONF_USE_LF_FOR_WAKEUP */
        }
        schedule.period = LWB_PERIOD_IDLE;
      } else {
    #if LWB_CONF_USE_LF_FOR_WAKEUP
        t_ref_lf += LWB_CONF_SCHED_PERIOD_IDLE_MS * RTIMER_SECOND_LF / 1000;
    #else /* LWB_CONF_USE_LF_FOR_WAKEUP */
        t_ref += LWB_CONF_SCHED_PERIOD_IDLE_MS * RTIMER_SECOND_HF / 1000;
    #endif /* LWB_CONF_USE_LF_FOR_WAKEUP */
      }
    }
        
    /* permission to participate in this round? */
    if(sync_state == SYNCED) {
      
    #if LWB_CONF_SCHED_COMPRESS
      lwb_sched_uncompress((uint8_t*)schedule.slot, 
                          LWB_SCHED_N_SLOTS(&schedule));
    #endif /* LWB_CONF_SCHED_COMPRESS */
      
      static uint16_t i;
      t_slot_ofs = (LWB_CONF_T_SCHED + LWB_CONF_T_GAP); 
    
      /* --- DATA SLOTS --- */
      
      if(LWB_SCHED_HAS_DATA_SLOT(&schedule)) {
        /* set the slot duration */
        if(IS_DATA_ROUND(&schedule)) {
          /* this is a data round */
          t_slot = LWB_CONF_T_DATA;
        } else {
          /* it's a request round */
          t_slot = LWB_CONF_T_CONT;
          //node_registered = 0;
        }
        for(i = 0; i < LWB_SCHED_N_SLOTS(&schedule); i++) {
          if(schedule.slot[i] == node_id) {
            //node_registered = 1;
            stats.t_slot_last = schedule.time;
            /* this is our data slot, send a data packet */
            if(!FIFO_EMPTY(&out_buffer)) {
              if(IS_DATA_ROUND(&schedule)) {
                lwb_out_buffer_get((uint8_t*)glossy_payload, &payload_len);
              } else {
                payload_len = 1;
                /* request as many data slots as there are pkts in the queue*/
                *(uint8_t*)glossy_payload = lwb_get_send_buffer_state();
              }
              LWB_WAIT_UNTIL(t_ref + t_slot_ofs);
              LWB_SEND_PACKET();
              DEBUG_PRINT_VERBOSE("packet sent (%ub)", payload_len);
            } else {
              DEBUG_PRINT_VERBOSE("no message to send (data slot ignored)");
            }
          } else
          {
            payload_len = GLOSSY_UNKNOWN_PAYLOAD_LEN;
            if(!IS_DATA_ROUND(&schedule)) {
              /* the payload length is known in the request round */
              payload_len = LWB_CONF_SRQ_PKT_LEN;
            }
            /* receive a data packet */
            LWB_WAIT_UNTIL(t_ref + t_slot_ofs - t_guard);
            LWB_RCV_PACKET();
            payload_len = glossy_get_payload_len();
            if(schedule.slot[i] == 0 || schedule.slot[i] == HOST_ID) {
              /* forward the packet to the application task */
              lwb_in_buffer_put((uint8_t*)glossy_payload, payload_len);
            }
            stats.rx_total += payload_len;
            stats.pck_cnt++;
          }
          t_slot_ofs += (t_slot + LWB_CONF_T_GAP);
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
          LWB_WAIT_UNTIL(t_ref + t_slot_ofs);
          LWB_SEND_PACKET();
        } else {
          /* no request pending -> just receive / relay packets */
          payload_len = LWB_CONF_SRQ_PKT_LEN;
          LWB_WAIT_UNTIL(t_ref + t_slot_ofs - t_guard);
          LWB_RCV_PACKET();
        }
        t_slot_ofs += LWB_CONF_T_CONT + LWB_CONF_T_GAP;
                   
        /* --- RECEIVE 2ND SCHEDULE --- */
      
        /* only rcv the 2nd schedule if there was a contention slot */
        payload_len = 2;  /* we expect exactly 2 bytes */
        LWB_WAIT_UNTIL(t_ref + t_slot_ofs - t_guard);
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
      if(schedule.period == LWB_PERIOD_IDLE) {
        int32_t drift = ((int32_t)((t_ref - t_ref_last) - 
           (int32_t)(LWB_CONF_SCHED_PERIOD_IDLE_MS * RTIMER_SECOND_HF / 1000)))
           * 1000 / (int32_t)LWB_CONF_SCHED_PERIOD_IDLE_MS;
        t_ref_last = t_ref; 
        if(drift < 50 && drift > -50) { /* no more than 50 ticks per second */
          stats.drift = (stats.drift + drift) / 2;
        }
      }
    #endif /* LWB_CONF_USE_LF_FOR_WAKEUP */
    }
    
    /* --- COMMUNICATION ROUND ENDS --- */    
    /* time for other computations */
    
    if(schedule.period > LWB_PERIOD_IDLE / 2) {
      /* print out some stats (note: takes approx. 2ms to compose this string) */
      DEBUG_PRINT_INFO("%s %lu T=%u n=%u tp=%u p=%u r=%u b=%u "
                       "u=%u per=%d snr=%ddbm", 
                       lwb_sync_state_to_string[sync_state],
                       schedule.time, 
                       schedule.period * (1000 / LWB_PERIOD_SCALE), 
                       LWB_SCHED_N_SLOTS(&schedule), 
                       stats.t_proc_max,
                       stats.pck_cnt,
                       stats.relay_cnt, 
                       stats.bootstrap_cnt, 
                       stats.unsynced_cnt,
                       glossy_get_per(),
                       stats.glossy_snr);
        
      /* poll the other processes */
      debug_print_poll();
      if(post_proc) {
        process_poll(post_proc);
      }      
    #if LWB_CONF_T_PREPROCESS
      t_preprocess = LWB_CONF_T_PREPROCESS;
    #endif /* LWB_CONF_T_PREPROCESS */
    }
    /* erase the schedule (slot allocations only) */
    memset(&schedule.slot, 0, sizeof(schedule.slot));
    
    if(!running) {
      debug_print_poll();
      if(post_proc) {
        process_poll(post_proc);
      } 
      /* suspend LWB task indefinitely */
      LWB_TASK_SUSPENDED;
      PT_YIELD(&lwb_pt);
      LWB_TASK_RESUMED;
      /* no preprocessing, can be done by the task which resumes the LWB */
      t_preprocess = 0;
      sync_state = BOOTSTRAP;   /* reset state machine */
      continue;
    }
    
  #if LWB_CONF_USE_LF_FOR_WAKEUP
    LWB_LF_WAIT_UNTIL(t_ref_lf + (schedule.period * RTIMER_SECOND_LF) /
                      LWB_PERIOD_SCALE - 
                      (t_guard / RTIMER_HF_LF_RATIO) - t_preprocess);
  #else /* LWB_CONF_USE_LF_FOR_WAKEUP */
    LWB_WAIT_UNTIL(t_ref + (schedule.period * RTIMER_SECOND_HF) /
                   LWB_PERIOD_SCALE - t_guard - t_preprocess);    
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
  /* start in 10ms */
#if LWB_CONF_USE_LF_FOR_WAKEUP
  rtimer_clock_t t_wakeup = rtimer_now_lf() + RTIMER_SECOND_LF / 100;
  rtimer_id_t    rt_id    = LWB_CONF_LF_RTIMER_ID;
#else /* LWB_CONF_USE_LF_FOR_WAKEUP */
  rtimer_clock_t t_wakeup = rtimer_now_hf() + RTIMER_SECOND_HF / 100;
  rtimer_id_t    rt_id    = LWB_CONF_RTIMER_ID;
#endif /* LWB_CONF_USE_LF_FOR_WAKEUP */
    
  if(!running) {
    running = 1;
    if(node_id == HOST_ID) {
    #if LWB_CONF_USE_LF_FOR_WAKEUP && RTIMER_CONF_LF_UPDATE_INT
      /* update the global time and wait for the next full second */
      schedule.time = ((t_wakeup + RTIMER_SECOND_LF) / RTIMER_SECOND_LF);
      lwb_sched_set_time(schedule.time);
      t_wakeup = (rtimer_clock_t)schedule.time * RTIMER_SECOND_LF;
    #endif /* LWB_CONF_USE_LF_FOR_WAKEUP && RTIMER_CONF_LF_UPDATE_INT */
      rtimer_schedule(rt_id, t_wakeup, 0, lwb_thread_host);
    } else {
      rtimer_schedule(rt_id, t_wakeup, 0, lwb_thread_src);
    }
  }
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(lwb_process, ev, data) 
{
  PROCESS_BEGIN();
  
#if !LWB_CONF_USE_XMEM
  /* pass the start addresses of the memory blocks holding the queues */
  fifo_init(&in_buffer, (uint16_t)in_buffer_mem);
  fifo_init(&out_buffer, (uint16_t)out_buffer_mem); 
#else  /* LWB_CONF_USE_XMEM */
  /* allocate memory for the message buffering (in ext. memory) */
  fifo_init(&in_buffer, xmem_alloc(LWB_CONF_IN_BUFFER_SIZE * 
                                   (LWB_CONF_MAX_DATA_PKT_LEN + 1)));
  fifo_init(&out_buffer, xmem_alloc(LWB_CONF_OUT_BUFFER_SIZE * 
                                    (LWB_CONF_MAX_DATA_PKT_LEN + 1)));   
#endif /* LWB_CONF_USE_XMEM */

#ifdef LWB_CONF_TASK_ACT_PIN
  PIN_CFG_OUT(LWB_CONF_TASK_ACT_PIN);
  PIN_CLR(LWB_CONF_TASK_ACT_PIN);
#endif /* LWB_CONF_TASK_ACT_PIN */
  
  PT_INIT(&lwb_pt); /* initialize the protothread */

  if(node_id == HOST_ID) {
    schedule_len = lwb_sched_init(&schedule);
  }
  lwb_resume();

  /* instead of terminating the process here, use it for other tasks such as
   * external memory access */  
#if LWB_CONF_USE_XMEM
  while(1) {
    LWB_TASK_SUSPENDED;
    PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_POLL);
    LWB_TASK_RESUMED;
    /* is there anything to do? */
    if(xmem_task.op == 1) {           /* read operation */
      if(xmem_read(xmem_task.xmem_addr, LWB_CONF_MAX_DATA_PKT_LEN + 1, 
                   xmem_buffer)) {
        xmem_wait_until_ready();   /* wait for the data transfer to complete */
        /* trust the data in the memory, no need to check the length field */
        uint8_t len = *(xmem_buffer + LWB_CONF_MAX_DATA_PKT_LEN);
        memcpy(xmem_task.sram_ptr, xmem_buffer, len);
        if(xmem_task.notify) {
          *xmem_task.notify = len;
        }
      }
    } else if(xmem_task.op == 2) {    /* write operation */
      memcpy(xmem_buffer, xmem_task.sram_ptr, xmem_task.len);
      *(xmem_buffer + LWB_CONF_MAX_DATA_PKT_LEN) = xmem_task.len;
      xmem_wait_until_ready();      /* wait for ongoing transfer to complete */
      xmem_write(xmem_task.xmem_addr, LWB_CONF_MAX_DATA_PKT_LEN + 1,
                 xmem_buffer);
    } // else: no operation pending
    xmem_task.op = 0;
  }  
#endif /* LWB_CONF_USE_XMEM */

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
void
lwb_start(struct process *pre_lwb_proc, struct process *post_lwb_proc)
{
  pre_proc = pre_lwb_proc;
  post_proc = (struct process*)post_lwb_proc;
  printf("Starting '%s'\r\n", lwb_process.name);    
  printf(" data=%ub n_slots=%u n_tx_data=%u n_tx_sched=%u n_hops=%u\r\n", 
         LWB_CONF_MAX_DATA_PKT_LEN, 
         LWB_CONF_MAX_DATA_SLOTS, 
         LWB_CONF_TX_CNT_DATA, 
         LWB_CONF_TX_CNT_SCHED,
         LWB_CONF_MAX_HOPS);
  /* ceil the values (therefore + RTIMER_SECOND_HF / 1000 - 1) */
  printf(" slot times [ms]: sched=%u data=%u cont=%u\r\n",
         (uint16_t)RTIMER_HF_TO_MS(LWB_CONF_T_SCHED + (RTIMER_SECOND_HF / 1000 - 1)),
         (uint16_t)RTIMER_HF_TO_MS(LWB_CONF_T_DATA + (RTIMER_SECOND_HF / 1000 - 1)),
         (uint16_t)RTIMER_HF_TO_MS(LWB_CONF_T_CONT + (RTIMER_SECOND_HF / 1000 - 1)));
  process_start(&lwb_process, NULL);
}
/*---------------------------------------------------------------------------*/

#endif /* LWB_VERSION */
