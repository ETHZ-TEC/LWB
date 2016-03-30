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
 *          Federico Ferrari
 *          Marco Zimmerling
 */

/**
 * @file
 *
 * a modified implementation of the Low-Power Wireless Bus
 */
 
#include "contiki.h"

#if LWB_VERSION == 2

/* this version of the LWB only supports the 'burst' scheduler */
#if !defined(LWB_SCHED_AE)
#error "LWB_MOD only support the 'burst' and 'AE' scheduler"
#endif

/*---------------------------------------------------------------------------*/
#define LWB_CONF_PERIOD_SCALE       100         /* also change in sched-ae.c */
#define LWB_CONF_PERIOD_MIN         4

/* expected packet length of a slot request */         
#ifndef LWB_CONF_SRQ_PKT_LEN
#define LWB_CONF_SRQ_PKT_LEN        1 
#endif /* LWB_CONF_SRQ_PKT_LEN */

/* how many LF clock ticks to wake up earlier when using the LF oscillator
 * btw. the rounds */
#ifndef LWB_CONF_LF_WAKEUP_OFS
#define LWB_CONF_LF_WAKEUP_OFS      5
#endif /* LWB_CONF_LF_WAKEUP_OFS */

#ifndef LWB_CONF_SACK_SLOT
#define LWB_CONF_SACK_SLOT          0
#endif /* LWB_CONF_SACK_SLOT */

/* indicates when this node is about to send a request */
#ifdef LWB_REQ_IND_PIN
  #define LWB_REQ_IND               { PIN_SET(LWB_REQ_IND_PIN); \
                                      PIN_CLR(LWB_REQ_IND_PIN); }
#else /* LWB_CONF_REQ_SENT_PIN */
  #define LWB_REQ_IND
#endif /* LWB_CONF_REQ_SENT_PIN */

/* code that is executed upon detection of a contention */
#ifndef LWB_REQ_DETECTED
#define LWB_REQ_DETECTED
#endif /* LWB_REQ_DETECTED */

/* indicates when this node is about to send a data packet */ 
#ifdef LWB_DATA_IND_PIN
  #define LWB_DATA_IND              { PIN_SET(LWB_DATA_IND_PIN); \
                                      PIN_CLR(LWB_DATA_IND_PIN); }
#else /* LWB_CONF_DATA_IND_PIN */
  #define LWB_DATA_IND
#endif /* LWB_CONF_DATA_IND_PIN */

/* is executed before each data slot */
#ifndef LWB_DATA_SLOT_STARTS
#define LWB_DATA_SLOT_STARTS
#endif /* LWB_DATA_SLOT_STARTS */
/*---------------------------------------------------------------------------*/
/* internal sync state of the LWB on the source node */
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
  LWB_BEFORE_DEEPSLEEP();\
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
static struct process*  pre_proc;
static struct process*  post_proc;
static lwb_sync_state_t sync_state;
static rtimer_clock_t   reception_timestamp;
static uint32_t         global_time;
static lwb_statistics_t stats = { 0 };
/* no buffers needed if this is only a relay node */
#if !LWB_CONF_RELAY_ONLY
#if !LWB_CONF_USE_XMEM
/* allocate memory in the SRAM (+1 to store the message length) */
static uint8_t          in_buffer_mem[LWB_CONF_IN_BUFFER_SIZE * 
                                      (LWB_CONF_MAX_DATA_PKT_LEN + 1)];  
static uint8_t          out_buffer_mem[LWB_CONF_OUT_BUFFER_SIZE * 
                                       (LWB_CONF_MAX_DATA_PKT_LEN + 1)]; 
#else /* LWB_CONF_USE_XMEM */
static uint8_t          data_buffer[LWB_CONF_MAX_DATA_PKT_LEN + 1];
static uint32_t         stats_addr = 0;
#endif /* LWB_CONF_USE_XMEM */
FIFO(in_buffer, LWB_CONF_MAX_DATA_PKT_LEN + 1, LWB_CONF_IN_BUFFER_SIZE);
FIFO(out_buffer, LWB_CONF_MAX_DATA_PKT_LEN + 1, LWB_CONF_OUT_BUFFER_SIZE);
#endif /* LWB_CONF_RELAY_ONLY */
/*---------------------------------------------------------------------------*/
#if LWB_CONF_USE_XMEM   /* to get rid of a compiler warning */
static uint16_t 
calc_crc16(const uint8_t* data, uint8_t num_bytes) 
{
  uint16_t crc  = 0,
           mask = 0xa001;
  while(num_bytes) {
    uint8_t ch = *data;
    int8_t bit = 0;
    while(bit < 8) {
      if((crc & 1) ^ (ch & 1)) {
        crc = (crc >> 1) ^ mask;
      } else {
        crc >>= 1;
      }
      ch >>= 1; 
      bit += 1;
    }
    data++;
    num_bytes--;
  }
  return crc;
}
#endif /* LWB_CONF_USE_XMEM */
/*---------------------------------------------------------------------------*/
uint8_t
lwb_stats_load(void) 
{
#if LWB_CONF_USE_XMEM
  uint16_t crc;
  if(!xmem_init()) { /* make sure external memory is accessible */
    return 0;
  }
  stats_addr = xmem_alloc(sizeof(lwb_statistics_t));
  if(XMEM_ALLOC_ERROR == stats_addr || 
     !xmem_read(stats_addr, sizeof(lwb_statistics_t), (uint8_t*)&stats)) {
    DEBUG_PRINT_MSG_NOW("WARNING: failed to load stats");
  }
  crc = stats.crc;
  stats.crc = 0;
  if(calc_crc16((uint8_t*)&stats, sizeof(lwb_statistics_t)) != crc) {
    DEBUG_PRINT_MSG_NOW("WARNING: stats corrupted, values reset");
    memset(&stats, 0, sizeof(lwb_statistics_t));
  }
  stats.reset_cnt++;
  DEBUG_PRINT_MSG_NOW("stats loaded, reset count: %d", stats.reset_cnt);
  return 1;
#else
  return 0;
#endif /* LWB_CONF_USE_XMEM */
}
/*---------------------------------------------------------------------------*/
void 
lwb_stats_save(void) 
{
#if LWB_CONF_USE_XMEM
  stats.crc = 0;    /* necessary */
  stats.crc = calc_crc16((uint8_t*)&stats, sizeof(lwb_statistics_t));
  if(!xmem_write(stats_addr, sizeof(lwb_statistics_t), (uint8_t*)&stats)) {
    DEBUG_PRINT_WARNING("failed to write stats");
  }
#endif /* LWB_CONF_USE_XMEM */
}
/*---------------------------------------------------------------------------*/
void 
lwb_stats_reset(void) 
{
#if LWB_CONF_USE_XMEM
  memset(&stats, 0, sizeof(lwb_statistics_t));
  lwb_stats_save();
#endif /* LWB_CONF_USE_XMEM */
}
/*---------------------------------------------------------------------------*/
#if !LWB_CONF_RELAY_ONLY
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
#if !LWB_CONF_USE_XMEM
    /* copy the data into the queue */
    memcpy((uint8_t*)((uint16_t)pkt_addr), data, len);
    /* last byte holds the payload length */
    *(uint8_t*)((uint16_t)pkt_addr + LWB_CONF_MAX_DATA_PKT_LEN) = len;    
#else /* LWB_CONF_USE_XMEM */
    /* write the data into the queue in the external memory */
    xmem_write(pkt_addr, len, data);
    xmem_write(pkt_addr + LWB_CONF_MAX_DATA_PKT_LEN, 1, &len);
#endif /* LWB_CONF_USE_XMEM */
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
#if !LWB_CONF_USE_XMEM
    /* assume pointers are always 16-bit */
    uint8_t* next_msg = (uint8_t*)((uint16_t)pkt_addr);  
    memcpy(out_data, next_msg, *(next_msg + LWB_CONF_MAX_DATA_PKT_LEN));
    return *(next_msg + LWB_CONF_MAX_DATA_PKT_LEN);
#else /* LWB_CONF_USE_XMEM */
    xmem_read(pkt_addr, LWB_CONF_MAX_DATA_PKT_LEN + 1, data_buffer);
    memcpy(out_data, data_buffer, *(out_data + LWB_CONF_MAX_DATA_PKT_LEN));
    return *(out_data + LWB_CONF_MAX_DATA_PKT_LEN);
#endif /* LWB_CONF_USE_XMEM */
  }
  DEBUG_PRINT_VERBOSE("out queue empty");
  return 0;
}
/*---------------------------------------------------------------------------*/
/* puts a message into the outgoing queue, returns 1 if successful, 
 * 0 otherwise */
uint8_t
lwb_put_data(const uint8_t * const data, 
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
    *(data_buffer + LWB_CONF_MAX_DATA_PKT_LEN) = len;
    memcpy(data_buffer, data, len);
    /* always read the max length since we don't know how long the packet is */
    xmem_write(pkt_addr, LWB_CONF_MAX_DATA_PKT_LEN + 1, data_buffer);
#endif /* LWB_CONF_USE_XMEM */
    return 1;
  }
  DEBUG_PRINT_VERBOSE("out queue full");
  return 0;
}
/*---------------------------------------------------------------------------*/
/* copies the oldest received message in the queue into out_data and returns 
 * the message size (in bytes) */
uint8_t
lwb_get_data(uint8_t* out_data)
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
    memcpy(out_data, next_msg, 
           *(next_msg + LWB_CONF_MAX_DATA_PKT_LEN));
    return *(next_msg + LWB_CONF_MAX_DATA_PKT_LEN);
#else /* LWB_CONF_USE_XMEM */
    xmem_read(pkt_addr, LWB_CONF_MAX_DATA_PKT_LEN + 1, data_buffer);
    memcpy(out_data, data_buffer, 
           *(data_buffer + LWB_CONF_MAX_DATA_PKT_LEN));
    return *(data_buffer + LWB_CONF_MAX_DATA_PKT_LEN);
#endif /* LWB_CONF_USE_XMEM */
  }
  DEBUG_PRINT_VERBOSE("in queue empty");
  return 0;
}
/*---------------------------------------------------------------------------*/
uint8_t
lwb_get_rcv_buffer_state(void)
{
  return !FIFO_EMPTY(&in_buffer);
}
/*---------------------------------------------------------------------------*/
uint8_t
lwb_get_send_buffer_state(void)
{
  return !FIFO_EMPTY(&out_buffer);
}
#endif /* LWB_CONF_RELAY_ONLY */
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
  if(sync_state < SYNCED) { return LWB_STATE_INIT; }
  else if(sync_state < UNSYNCED) { return LWB_STATE_CONNECTED; }
  return LWB_STATE_CONN_LOST;
}
/*---------------------------------------------------------------------------*/
uint32_t 
lwb_get_time(rtimer_clock_t* reception_time)
{
  if(reception_time) {
    *reception_time = reception_timestamp;
  }
  return global_time;
}
/*---------------------------------------------------------------------------*/
#if !LWB_CONF_RELAY_ONLY
/**
 * @brief thread of the host node
 */
PT_THREAD(lwb_thread_host(rtimer_t *rt)) 
{  
  /* all variables must be static */
  static lwb_schedule_t schedule;
  static rtimer_clock_t t_start; 
  static rtimer_clock_t t_now;
  static rtimer_clock_t t_start_lf;
  /* packet buffer: use uint16_t to force word boundary alignment */
  static uint16_t glossy_payload[(LWB_CONF_MAX_PKT_LEN + 1) / 2];
  /* constant guard time for the host */
  static const uint32_t t_guard = LWB_CONF_T_GUARD; 
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

  /* note: all statements above PT_BEGIN() will be executed each time the 
   * protothread is scheduled */
  
  PT_BEGIN(&lwb_pt);   /* declare variables before this statement! */
    
  memset(&schedule, 0, sizeof(schedule));
  
  /* initialization specific to the host node */
  schedule_len = lwb_sched_init(&schedule);
  sync_state = SYNCED;  /* the host is always 'synced' */
  
  rt->time = rtimer_now_lf();
  
  while(1) {
    
    t_start_lf = rt->time; 
    rt->time = rtimer_now_hf();
    t_start = rt->time;
        
    /* --- COMMUNICATION ROUND STARTS --- */
    
    global_time = schedule.time;
    reception_timestamp = t_start;
    
    LWB_SEND_SCHED();
   
    glossy_rssi = glossy_get_rssi();
    stats.relay_cnt = glossy_get_relay_cnt_first_rx();
    slot_idx = 0;     /* reset the packet counter */
    
#if LWB_CONF_USE_XMEM
    /* put the external memory back into active mode (takes ~500us) */
    xmem_wakeup();    
#endif /* LWB_CONF_USE_XMEM */
        
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
            LWB_DATA_SLOT_STARTS;
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
              stats.data_tot += payload_len;
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
        LWB_REQ_DETECTED;
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

    /* --- S-ACK SLOT --- */
  #if LWB_CONF_SACK_SLOT
    if(i &  & 0x8000) {
      payload_len = lwb_sched_prepare_sack((uint8_t*)glossy_payload);
      /* wait for the slot to start */
      LWB_WAIT_UNTIL(t_start + LWB_T_SLOT_START(slot_idx));
      LWB_SEND_PACKET();   /* transmit s-ack */
      DEBUG_PRINT_VERBOSE("S-ACK sent (%ub)", payload_len);
      slot_idx++;
    }
  #endif /* LWB_CONF_SACK_SLOT */
  
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
    
    /* print out some stats */
    DEBUG_PRINT_INFO("%lu T=%u n=%u|%u ts=%u srq=%u p=%u per=%d%% rssi=%ddBm", 
                     global_time,
                     schedule.period / LWB_CONF_PERIOD_SCALE,
                     schedule.n_slots & 0x3fff,
                     schedule.n_slots >> 14,
                     stats.t_sched_max, 
                     srq_cnt, 
                     stats.pck_cnt,
                     glossy_get_per(),
                     glossy_rssi);
        
#if LWB_CONF_STATS_NVMEM
    lwb_stats_save();
#endif /* LWB_CONF_STATS_NVMEM */
    /* poll the other processes to allow them to run after the LWB task was 
     * suspended (note: the polled processes will be executed in the inverse
     * order they were started/created) */
    if(curr_period > LWB_CONF_SCHED_PERIOD_IDLE * LWB_CONF_PERIOD_SCALE /2) {
      debug_print_poll();
      if(post_proc) {
        /* will be executed before the debug print task */
        process_poll(post_proc);    
      }
    }
    
    /* suspend this task and wait for the next round */
    LWB_LF_WAIT_UNTIL(t_start_lf + curr_period * RTIMER_SECOND_LF / 
                      LWB_CONF_PERIOD_SCALE);
  }
  
  PT_END(&lwb_pt);
}
#endif /* LWB_CONF_RELAY_ONLY */
/*---------------------------------------------------------------------------*/
/**
 * @brief declaration of the protothread (source node)
 */
PT_THREAD(lwb_thread_src(rtimer_t *rt)) 
{  
  /* all variables must be static */
  static lwb_schedule_t schedule;
  static rtimer_clock_t t_ref;
  static rtimer_clock_t t_ref_lf;
  /* packet buffer: use uint16_t to force word boundary alignment */
  static uint16_t glossy_payload[(LWB_CONF_MAX_PKT_LEN + 1) / 2];
  static uint32_t t_guard;                  /* 32-bit is enough for t_guard! */
  static uint32_t t_slot = LWB_CONF_T_DATA;
  static uint8_t slot_idx;
#if !LWB_CONF_RELAY_ONLY
  static uint8_t payload_len;
#endif /* LWB_CONF_RELAY_ONLY */
  static int8_t  glossy_snr = 0;
  static uint8_t node_registered = 0;   /* host knows about this node? */
  static const void* callback_func = lwb_thread_src;
  
  PT_BEGIN(&lwb_pt);   /* declare variables before this statement! */
  
  memset(&schedule, 0, sizeof(schedule)); 
  
  sync_state = BOOTSTRAP;
  
  while(1) {
          
    /* --- COMMUNICATION ROUND STARTS --- */
        
    rt->time = rtimer_now_hf();        /* overwrite LF with HF timestamp */
    
    if(sync_state == BOOTSTRAP) {
      DEBUG_PRINT_MSG_NOW("BOOTSTRAP ");
      stats.bootstrap_cnt++;
      node_registered = 0;
      /* synchronize first! wait for the first schedule... */
      payload_len = LWB_SCHED_PKT_HEADER_LEN;   /* empty schedule */
      do {
        LWB_RCV_SCHED();
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
    glossy_snr = glossy_get_snr();
                   
    /* update the sync state machine (compute new sync state and update 
     * t_guard) */
    LWB_UPDATE_SYNC_STATE;  
     
    if(glossy_is_t_ref_updated()) {
      /* HF timestamp of first RX; subtract a constant offset */
      t_ref = glossy_get_t_ref() - LWB_CONF_T_REF_OFS;
      /* estimate t_ref_lf by subtracting the elapsed time since t_ref: */
      rtimer_clock_t hf_now;
      rtimer_now(&hf_now, &t_ref_lf);
      t_ref_lf -= (uint32_t)(hf_now - t_ref) / (uint32_t)RTIMER_HF_LF_RATIO;
      if(schedule.period == 
         LWB_CONF_SCHED_PERIOD_IDLE * LWB_CONF_PERIOD_SCALE) {
        /* only update the timestamp during the idle period */
        global_time = schedule.time;
        reception_timestamp = t_ref_lf;
      }
    } else {
      DEBUG_PRINT_WARNING("schedule missed");
      /* we can only estimate t_ref and t_ref_lf */
      if(sync_state == UNSYNCED) {
        t_ref_lf = reception_timestamp;
        if(schedule.period > 
           LWB_CONF_SCHED_PERIOD_IDLE * LWB_CONF_PERIOD_SCALE / 2) {
          t_ref_lf += LWB_CONF_SCHED_PERIOD_IDLE * RTIMER_SECOND_LF;
        }
        schedule.period = LWB_CONF_SCHED_PERIOD_IDLE * LWB_CONF_PERIOD_SCALE;
      } else {
        /* since HF clock was off, we need a new timestamp; subtract a const.
         * processing offset to adjust (if needed) */
        t_ref_lf += schedule.period * (RTIMER_SECOND_LF) /
                    LWB_CONF_PERIOD_SCALE;
      }
    }
        
  #if LWB_CONF_USE_XMEM
    /* put the external memory back into active mode (takes ~500us) */
    xmem_wakeup();    
  #endif /* LWB_CONF_USE_XMEM */
        
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
          node_registered = 0;
        }
        for(i = 0; i < LWB_SCHED_N_SLOTS(&schedule); i++, slot_idx++) {
  #if !LWB_CONF_RELAY_ONLY
          if(schedule.slot[i] == node_id) {
            node_registered = 1;
            stats.t_slot_last = schedule.time;
            /* this is our data slot, send a data packet */
            if(!FIFO_EMPTY(&out_buffer)) {
              if(LWB_SCHED_HAS_SACK_SLOT(&schedule)) {
                /* it's a data round */
                payload_len = lwb_out_buffer_get((uint8_t*)glossy_payload);
                //if(payload_len == 0) { DEBUG_PRINT_WARNING("pkt_len = 0b"); }
                LWB_DATA_IND;
              } else {
                payload_len = 1;
                /* request LWB_CONF_OUT_BUFFER_SIZE data slots */
                *(uint8_t*)glossy_payload = (uint8_t)LWB_CONF_OUT_BUFFER_SIZE;
              }
              LWB_WAIT_UNTIL(t_ref + LWB_T_SLOT_START(slot_idx));
              LWB_SEND_PACKET();
              DEBUG_PRINT_VERBOSE("packet sent (%ub)", payload_len);
            } else {
              DEBUG_PRINT_VERBOSE("no message to send (data slot ignored)");
            }
          } else
  #endif /* LWB_CONF_RELAY_ONLY */
          {
            payload_len = GLOSSY_UNKNOWN_PAYLOAD_LEN;
            if(!LWB_SCHED_HAS_SACK_SLOT(&schedule)) {
              /* the payload length is known in the request round */
              payload_len = LWB_CONF_SRQ_PKT_LEN;
            }
            /* receive a data packet */
            LWB_WAIT_UNTIL(t_ref + LWB_T_SLOT_START(slot_idx) - t_guard);
            LWB_RCV_PACKET();
            stats.data_tot += glossy_get_payload_len(); 
            stats.pck_cnt++;
          }
        }
      }
      
      /* --- CONTENTION SLOT --- */
      
      /* is there a contention slot in this round? */
      if(LWB_SCHED_HAS_CONT_SLOT(&schedule)) {
        t_slot = LWB_CONF_T_CONT;
  #if !LWB_CONF_RELAY_ONLY
        if(!FIFO_EMPTY(&out_buffer)) {
          /* if there is data in the output buffer, then request a slot */
          /* a slot request packet always looks the same (1 byte) */
          payload_len = 1;
          /* include the node ID in case this is the first request */
          glossy_payload[0] = 0;
          if(!node_registered) {
            *(uint8_t*)glossy_payload = (uint8_t)node_id; /* truncate */
          }
          /* wait until the contention slot starts */
          LWB_REQ_IND;
          LWB_WAIT_UNTIL(t_ref + LWB_T_SLOT_START(slot_idx));
          LWB_SEND_PACKET();
          DEBUG_PRINT_VERBOSE("request sent");
        } else {          
  #endif /* LWB_CONF_RELAY_ONLY */     
          /* no request pending -> just receive / relay packets */
          payload_len = LWB_CONF_SRQ_PKT_LEN;
          LWB_WAIT_UNTIL(t_ref + LWB_T_SLOT_START(slot_idx) - t_guard);
          LWB_RCV_PACKET();      
  #if !LWB_CONF_RELAY_ONLY
        }
  #endif /* LWB_CONF_RELAY_ONLY */
        slot_idx++;   /* increment the packet counter */
      }
       
      /* increment to give the host time for the schedule computation */
      slot_idx++; 
      
      /* --- S-ACK SLOT --- */
  #if LWB_CONF_SACK_SLOT
      if(LWB_SCHED_HAS_SACK_SLOT(&schedule)) {
        payload_len = GLOSSY_UNKNOWN_PAYLOAD_LEN;
        /* wait for the slot to start */
        LWB_WAIT_UNTIL(t_ref + LWB_T_SLOT_START(slot_idx) - t_guard);     
        LWB_RCV_PACKET();                 /* receive s-ack */
        payload_len = glossy_get_payload_len() / 2;
    #if !LWB_CONF_RELAY_ONLY
        if(LWB_DATA_RCVD) {
          i = 0;
          while(i < payload_len) {
            if(*(glossy_payload + i) == node_id) {
              /* notify the application */
              lwb_in_buffer_put((uint8_t*)(glossy_payload + i), 2);
              DEBUG_PRINT_VERBOSE("S-ACK received");
            }
            i++;
          }
        } else {
          DEBUG_PRINT_VERBOSE("no data received in SACK slot");
        }
    #endif /* LWB_CONF_RELAY_ONLY */
        slot_idx++;
      }
  #endif /* LWB_CONF_SACK_SLOT */
      
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
    }
    
    /* --- COMMUNICATION ROUND ENDS --- */    
    /* time for other computations */

    /* DRIFT compensation is not needed (see lwb-mod-burst.c if necessary) */
      
    /* print out some stats (note: takes approx. 2ms to compose this string) */
    DEBUG_PRINT_INFO("%s %u %lu T=%u n=%u tp=%u p=%u r=%u b=%u "
                     "u=%u per=%d%% snr=%ddbm", 
                     lwb_sync_state_to_string[sync_state], 
                     node_registered,
                     schedule.time, 
                     schedule.period / LWB_CONF_PERIOD_SCALE, 
                     LWB_SCHED_N_SLOTS(&schedule), 
                     stats.t_proc_max,
                     stats.pck_cnt,
                     stats.relay_cnt, 
                     stats.bootstrap_cnt, 
                     stats.unsynced_cnt,
                     glossy_get_per(),
                     glossy_snr);
      
  #if LWB_CONF_STATS_NVMEM
    lwb_stats_save();
  #endif /* LWB_CONF_STATS_NVMEM */
    /* erase the schedule (slot allocations only) */
    memset(&schedule.slot, 0, sizeof(schedule.slot));

    /* poll the other processes to allow them to run after the LWB task was
     * suspended (note: the polled processes will be executed in the inverse
     * order they were started/created) */
    if(schedule.period > 
       LWB_CONF_SCHED_PERIOD_IDLE * LWB_CONF_PERIOD_SCALE / 2) {
      debug_print_poll();
      if(post_proc) {
        process_poll(post_proc);
      }
    }
    
    LWB_LF_WAIT_UNTIL(t_ref_lf + schedule.period * RTIMER_SECOND_LF /
                      LWB_CONF_PERIOD_SCALE - 
                      (t_guard / RTIMER_HF_LF_RATIO + LWB_CONF_LF_WAKEUP_OFS));
  }

  PT_END(&lwb_pt);
}
/*---------------------------------------------------------------------------*/
/* define the process control block */
PROCESS(lwb_process, "Communication Task (LWB)");
/*---------------------------------------------------------------------------*/
/* define the body (protothread) of a process */
PROCESS_THREAD(lwb_process, ev, data) 
{
  PROCESS_BEGIN();
  
#if LWB_CONF_STATS_NVMEM
  lwb_stats_load();   /* load the stats from the external memory */
#endif /* LWB_CONF_STATS_NVMEM */
  
#if !LWB_CONF_RELAY_ONLY
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
#endif /* LWB_CONF_RELAY_ONLY */
  
#ifdef LWB_CONF_TASK_ACT_PIN
  PIN_CFG_OUT(LWB_CONF_TASK_ACT_PIN);
  PIN_CLR(LWB_CONF_TASK_ACT_PIN);
#endif /* LWB_CONF_TASK_ACT_PIN */
        
  PT_INIT(&lwb_pt); /* initialize the protothread */
  
#ifdef NODE_ID
  #if (NODE_ID == HOST_ID) && !LWB_CONF_RELAY_ONLY
    /* note: must add at least some clock ticks! */
    rtimer_schedule(LWB_CONF_RTIMER_ID, 
                    rtimer_now_hf() + RTIMER_SECOND_HF / 10,
                    0, lwb_thread_host);
  #else
    rtimer_schedule(LWB_CONF_RTIMER_ID, 
                    rtimer_now_hf() + RTIMER_SECOND_HF / 10,
                    0, lwb_thread_src);    
  #endif
#else /* NODE_ID */
  if((node_id == HOST_ID) && !LWB_CONF_RELAY_ONLY) {
  #if !LWB_CONF_RELAY_ONLY
    /* note: must add at least some clock ticks! */
    rtimer_schedule(LWB_CONF_RTIMER_ID, 
                    rtimer_now_hf() + RTIMER_SECOND_HF / 10,
                    0, lwb_thread_host);
  #endif /* LWB_CONF_RELAY_ONLY */
  } else {
    rtimer_schedule(LWB_CONF_RTIMER_ID, 
                    rtimer_now_hf() + RTIMER_SECOND_HF / 10,
                    0, lwb_thread_src);
  }
#endif /* NODE_ID */

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
void
lwb_start(void *pre_lwb_proc, void *post_lwb_proc)
{
  pre_proc = (struct process*)pre_lwb_proc;
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