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
 *          Federico Ferrari
 *          Marco Zimmerling
 */

/**
 * @file
 *
 * an implementation of the Low-Power Wireless Bus
 */
 
#include "contiki.h"

#if LWB_VERSION == 1
/*---------------------------------------------------------------------------*/
#if LWB_CONF_HEADER_LEN != 3
#error "LWB_CONF_HEADER_LEN must be 3!"
#endif
#define LWB_DATA_PKT_PAYLOAD_LEN    (LWB_CONF_MAX_DATA_PKT_LEN - \
                                     LWB_CONF_HEADER_LEN)
#define STREAM_REQ_PKT_SIZE         5

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
  QUASI_SYNCED,
  SYNCED,
  SYNCED_2,
  MISSED,
  UNSYNCED,
  UNSYNCED2,
  NUM_OF_SYNC_STATES
} lwb_sync_state_t;
/*---------------------------------------------------------------------------*/
typedef enum {
  EVT_1ST_SCHED_RCVD = 0,
  EVT_2ND_SCHED_RCVD,
  EVT_SCHED_MISSED,
  NUM_OF_SYNC_EVENTS
} sync_event_t; 
/*---------------------------------------------------------------------------*/
typedef struct {
  uint16_t recipient;     /* target node ID */
  uint8_t  stream_id;     /* message type and connection ID (used as stream ID
                             in LWB); first bit is msg type */
  uint8_t  payload[LWB_DATA_PKT_PAYLOAD_LEN];       
} lwb_data_pkt_t;
/*---------------------------------------------------------------------------*/
typedef struct {  
  /* be aware of structure alignment (8-bit are aligned to 8-bit, 16 to 16 etc)
   * the first 3 bytes of a glossy packet are always node_id and stream_id */
  union {
    struct {
      lwb_data_pkt_t data_pkt;
      uint8_t reserved1[LWB_CONF_MAX_PKT_LEN - LWB_CONF_MAX_DATA_PKT_LEN];
    };
    struct {
      lwb_stream_req_t srq_pkt;
      uint8_t reserved2[LWB_CONF_MAX_PKT_LEN - LWB_STREAM_REQ_PKT_LEN];
    };
    lwb_stream_ack_t sack_pkt;
    uint8_t raw_data[LWB_CONF_MAX_PKT_LEN];
  };
} glossy_payload_t;
/*---------------------------------------------------------------------------*/
/**
 * @brief the finite state machine for the time synchronization on a source 
 * node the next state can be retrieved from the current state (column) and
 * the latest event (row)
 * @note  undefined transitions force the SM to go back into bootstrap
 */
#if LWB_CONF_SKIP_QUASI_SYNCED
#define AFTER_BOOT      SYNCED
#else
#define AFTER_BOOT      QUASI_SYNCED
#endif
static const 
lwb_sync_state_t next_state[NUM_OF_SYNC_EVENTS][NUM_OF_SYNC_STATES] = 
{/* STATES:                                                                               EVENTS:           */
 /* BOOTSTRAP,  QUASISYNCED,  SYNCED,    SYNCED2,   MISSED,    UNSYNCED,  UNSYNCED2                         */
  { AFTER_BOOT, SYNCED,       BOOTSTRAP, SYNCED,    SYNCED,    BOOTSTRAP, SYNCED    }, /* 1st schedule rcvd */
  { BOOTSTRAP,  QUASI_SYNCED, SYNCED_2,  BOOTSTRAP, BOOTSTRAP, SYNCED_2,  BOOTSTRAP }, /* 2nd schedule rcvd */
  { BOOTSTRAP,  BOOTSTRAP,    MISSED,    UNSYNCED,  UNSYNCED,  UNSYNCED2, BOOTSTRAP }  /* schedule missed   */
};
/* note: syn2 = already synced */
static const char* lwb_sync_state_to_string[NUM_OF_SYNC_STATES] = 
{ "BOOTSTRAP", "QSYN", "SYN", "SYN2", "MISS", "USYN", "USYN2" };
static const uint32_t guard_time[NUM_OF_SYNC_STATES] = {
/* STATE:      BOOTSTRAP,        QUASI_SYNCED,      SYNCED,           SYNCED_2,         MISSED,             UNSYNCED,           UNSYNCED2 */
/* T_GUARD: */ LWB_CONF_T_GUARD, LWB_CONF_T_GUARD,  LWB_CONF_T_GUARD, LWB_CONF_T_GUARD, LWB_CONF_T_GUARD_1, LWB_CONF_T_GUARD_2, LWB_CONF_T_GUARD_3
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
                                   (LWB_CONF_T_DATA + LWB_CONF_T_GAP) * i)
#define LWB_DATA_RCVD             (glossy_get_n_rx() > 0)
#define RTIMER_CAPTURE            (t_now = rtimer_now_hf())
#define RTIMER_ELAPSED            ((rtimer_now_hf() - t_now) * 1000 / 3250)    
#define GET_EVENT                 (glossy_is_t_ref_updated() ? \
                                   (LWB_SCHED_IS_1ST(&schedule) ? \
                                    EVT_1ST_SCHED_RCVD : EVT_2ND_SCHED_RCVD)\
                                   : EVT_SCHED_MISSED)
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
  glossy_start(GLOSSY_UNKNOWN_INITIATOR, (uint8_t *)&schedule, \
               GLOSSY_UNKNOWN_PAYLOAD_LEN, \
               LWB_CONF_TX_CNT_SCHED, GLOSSY_WITH_SYNC, GLOSSY_WITH_RF_CAL);\
  LWB_WAIT_UNTIL(rt->time + LWB_CONF_T_SCHED + t_guard);\
  glossy_stop();\
}   
#define LWB_SEND_PACKET() \
{\
  glossy_start(node_id, (uint8_t*)&glossy_payload, payload_len, \
               LWB_CONF_TX_CNT_DATA, GLOSSY_WITHOUT_SYNC, \
               GLOSSY_WITHOUT_RF_CAL);\
  LWB_WAIT_UNTIL(rt->time + LWB_CONF_T_DATA);\
  glossy_stop();\
}
#define LWB_RCV_PACKET() \
{\
  glossy_start(GLOSSY_UNKNOWN_INITIATOR, (uint8_t*)&glossy_payload, \
               GLOSSY_UNKNOWN_PAYLOAD_LEN, \
               LWB_CONF_TX_CNT_DATA, GLOSSY_WITHOUT_SYNC, \
               GLOSSY_WITHOUT_RF_CAL);\
  LWB_WAIT_UNTIL(rt->time + LWB_CONF_T_DATA + t_guard);\
  glossy_stop();\
}
#define LWB_SEND_SRQ() \
{\
  glossy_start(node_id, (uint8_t*)&glossy_payload, payload_len, \
               LWB_CONF_TX_CNT_DATA, GLOSSY_WITHOUT_SYNC, \
               GLOSSY_WITHOUT_RF_CAL);\
  LWB_WAIT_UNTIL(rt->time + LWB_CONF_T_CONT);\
  glossy_stop();\
}
#define LWB_RCV_SRQ() \
{\
  glossy_start(GLOSSY_UNKNOWN_INITIATOR, (uint8_t*)&glossy_payload, \
               GLOSSY_UNKNOWN_PAYLOAD_LEN, \
               LWB_CONF_TX_CNT_DATA, GLOSSY_WITHOUT_SYNC, \
               GLOSSY_WITHOUT_RF_CAL);\
  LWB_WAIT_UNTIL(rt->time + LWB_CONF_T_CONT + t_guard);\
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
  LWB_TASK_RESUMED;\
  LWB_AFTER_DEEPSLEEP();\
}
#define LWB_UPDATE_SYNC_STATE \
{\
  /* get the new state based on the event */\
  sync_state = next_state[GET_EVENT][sync_state];\
  t_guard = guard_time[sync_state];         /* adjust the guard time */\
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
static rtimer_clock_t   rx_timestamp;
static rtimer_clock_t   rx_timestamp_lf;
static uint32_t         global_time;
static lwb_statistics_t stats = { 0 };
static uint8_t          urgent_stream_req = 0;
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
uint8_t
lwb_stats_load(void) 
{
#if LWB_CONF_USE_XMEM
  if(!xmem_init()) { /* make sure external memory is accessible */
    return 0;
  }
  stats_addr = xmem_alloc(sizeof(lwb_statistics_t));
  if(XMEM_ALLOC_ERROR == stats_addr || 
     !xmem_read(stats_addr, sizeof(lwb_statistics_t), (uint8_t*)&stats)) {
    DEBUG_PRINT_MSG_NOW("WARNING: failed to load stats");
  }
  if(crc16((uint8_t*)&stats, sizeof(lwb_statistics_t) - 2, 0) != stats.crc) {
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
  stats.crc = crc16((uint8_t*)&stats, sizeof(lwb_statistics_t) - 2, 0);
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
  if(!len) {
    return 0;
  }
  if(len > LWB_CONF_MAX_DATA_PKT_LEN) {
    len = LWB_CONF_MAX_DATA_PKT_LEN;
    DEBUG_PRINT_WARNING("received data packet is too big"); 
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
    /* write the data into the queue in the external memory */
    memcpy(data_buffer, data, len);
    *(data_buffer + LWB_CONF_MAX_DATA_PKT_LEN) = len;
    xmem_write(pkt_addr, LWB_CONF_MAX_DATA_PKT_LEN + 1, data_buffer);
#endif /* LWB_CONF_USE_XMEM */
    return 1;
  }
  stats.rxbuf_drop++;
  DEBUG_PRINT_WARNING("lwb rx queue full");
  return 0;
}
/*---------------------------------------------------------------------------*/
/* fetch the next 'ready-to-send' message from the outgoing queue
 * returns the message length in bytes */
uint8_t 
lwb_out_buffer_get(uint8_t* out_data)
{   
  /* messages have the max. length LWB_CONF_MAX_DATA_PKT_LEN and are already
   * formatted according to glossy_payload_t */
  uint32_t pkt_addr = fifo_get(&out_buffer);
  if(FIFO_ERROR != pkt_addr) {
#if !LWB_CONF_USE_XMEM
    /* assume pointers are always 16-bit */
    uint8_t* next_msg = (uint8_t*)((uint16_t)pkt_addr);
    /* check the length */
    uint8_t len = *(next_msg + LWB_CONF_MAX_DATA_PKT_LEN);
    if(len > LWB_CONF_MAX_DATA_PKT_LEN) {
      DEBUG_PRINT_WARNING("invalid message length detected");
      len = LWB_CONF_MAX_DATA_PKT_LEN;  /* truncate */
    }
    memcpy(out_data, next_msg, len);
#else /* LWB_CONF_USE_XMEM */
    if(!xmem_read(pkt_addr, LWB_CONF_MAX_DATA_PKT_LEN + 1, data_buffer)) {
      return 0;
    }
    /* check the length */
    uint8_t len = *(data_buffer + LWB_CONF_MAX_DATA_PKT_LEN);
    memcpy(out_data, data_buffer, len);
#endif /* LWB_CONF_USE_XMEM */
    return len;
  }
  DEBUG_PRINT_VERBOSE("lwb tx queue empty");
  return 0;
}
/*---------------------------------------------------------------------------*/
/* puts a message into the outgoing queue, returns 1 if successful, 
 * 0 otherwise */
uint8_t
lwb_send_pkt(uint16_t recipient,
             uint8_t stream_id, 
             const uint8_t * const data, 
             uint8_t len)
{
  /* data has the max. length LWB_DATA_PKT_PAYLOAD_LEN, lwb header needs 
   * to be added before the data is inserted into the queue */
  if(len > LWB_DATA_PKT_PAYLOAD_LEN || !data) {
    DEBUG_PRINT_WARNING("invalid payload length");
    return 0;
  }
  uint32_t pkt_addr = fifo_put(&out_buffer);
  if(FIFO_ERROR != pkt_addr) {
#if !LWB_CONF_USE_XMEM
    /* assume pointers are 16-bit */
    uint8_t* next_msg = (uint8_t*)(uint16_t)pkt_addr;  
    *(next_msg) = (uint8_t)recipient;   /* recipient L */  
    *(next_msg + 1) = recipient >> 8;   /* recipient H */  
    *(next_msg + 2) = stream_id; 
    *(next_msg + LWB_CONF_MAX_DATA_PKT_LEN) = len + LWB_CONF_HEADER_LEN;
    memcpy(next_msg + LWB_CONF_HEADER_LEN, data, len);
#else /* LWB_CONF_USE_XMEM */
    *(data_buffer) = (uint8_t)recipient;   /* recipient L */  
    *(data_buffer + 1) = recipient >> 8;   /* recipient H */
    *(data_buffer + 2) = stream_id;
    *(data_buffer + LWB_CONF_MAX_DATA_PKT_LEN) = len + LWB_CONF_HEADER_LEN;
    memcpy(data_buffer + LWB_CONF_HEADER_LEN, data, len);
    xmem_write(pkt_addr, LWB_CONF_MAX_DATA_PKT_LEN + 1, data_buffer);
#endif /* LWB_CONF_USE_XMEM */
    return 1;
  }
  stats.txbuf_drop++;
  DEBUG_PRINT_VERBOSE("lwb tx queue full");
  return 0;
}
/*---------------------------------------------------------------------------*/
/* copies the oldest received message in the queue into out_data and returns 
 * the message size (in bytes) */
uint8_t
lwb_rcv_pkt(uint8_t* out_data,
             uint16_t * const out_node_id, 
             uint8_t * const out_stream_id)
{ 
  if(!out_data) { return 0; }
  /* messages in the queue have the max. length LWB_CONF_MAX_DATA_PKT_LEN, 
   * lwb header needs to be stripped off; payload has max. length
   * LWB_DATA_PKT_PAYLOAD_LEN */
  uint32_t pkt_addr = fifo_get(&in_buffer);
  if(FIFO_ERROR != pkt_addr) {
#if !LWB_CONF_USE_XMEM
    /* assume pointers are 16-bit */
    uint8_t* next_msg = (uint8_t*)(uint16_t)pkt_addr; 
    uint8_t msg_len = *(next_msg + LWB_CONF_MAX_DATA_PKT_LEN) -
                      LWB_CONF_HEADER_LEN;
    if(msg_len > LWB_CONF_MAX_DATA_PKT_LEN) {
      msg_len = LWB_CONF_MAX_DATA_PKT_LEN;
    }
    memcpy(out_data, next_msg + LWB_CONF_HEADER_LEN, msg_len);
    if(out_node_id) {
      /* cant just treat next_msg as 16-bit value due to misalignment */
      *out_node_id = (uint16_t)next_msg[1] << 8 | next_msg[0];
    }
    if(out_stream_id) {
      *out_stream_id = next_msg[2];
    }
#else /* LWB_CONF_USE_XMEM */
    if(!xmem_read(pkt_addr, LWB_CONF_MAX_DATA_PKT_LEN + 1, data_buffer)) {
      return 0;
    }
    uint8_t msg_len = *(data_buffer + LWB_CONF_MAX_DATA_PKT_LEN) -
                      LWB_CONF_HEADER_LEN;
    memcpy(out_data, data_buffer + LWB_CONF_HEADER_LEN, msg_len);
    if(out_node_id) {
      memcpy(out_node_id, data_buffer, 2);
    }
    if(out_stream_id) {
      *out_stream_id = data_buffer[2];
    }
#endif /* LWB_CONF_USE_XMEM */
    return msg_len;
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
#endif /* LWB_CONF_RELAY_ONLY */
/*---------------------------------------------------------------------------*/
const lwb_statistics_t * const
lwb_get_stats(void)
{
  return &stats;
}
/*---------------------------------------------------------------------------*/
uint8_t
lwb_request_stream(lwb_stream_req_t* stream_request, uint8_t urgent)
{
  if(!stream_request) { return 0; }
  if(urgent) {
    urgent_stream_req = stream_request->stream_id;
  }
  return lwb_stream_add(stream_request);
}
/*---------------------------------------------------------------------------*/
lwb_conn_state_t
lwb_get_state(void)
{
  if(sync_state < SYNCED) { return LWB_STATE_INIT; }
  else if(sync_state < MISSED) { return LWB_STATE_CONNECTED; }
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
uint64_t
lwb_get_timestamp(void)
{
  /* convert to microseconds */
  uint64_t timestamp = (uint64_t)global_time * 1000000;
  if(sync_state <= SYNCED_2) {
    return timestamp + /* convert to microseconds */
           (rtimer_now_hf() - rx_timestamp) * 1000000 / RTIMER_SECOND_HF;
  }
  /* not synced! */
  return timestamp +
         (rtimer_now_lf() - rx_timestamp_lf) * 1000000 / RTIMER_SECOND_LF;
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
#if LWB_CONF_USE_LF_FOR_WAKEUP
  static rtimer_clock_t t_start_lf;
#endif /* LWB_CONF_USE_LF_FOR_WAKEUP */
  static glossy_payload_t glossy_payload;                   /* packet buffer */
  /* constant guard time for the host */
  static const uint32_t t_guard = LWB_CONF_T_GUARD; 
  static uint8_t slot_idx;
  static uint8_t streams_to_update[LWB_CONF_MAX_DATA_SLOTS];
  static uint8_t schedule_len, 
                 payload_len;
  static uint8_t rcvd_data_pkts;
  static int8_t  glossy_rssi;
  static const void* callback_func = lwb_thread_host;

  /* note: all statements above PT_BEGIN() will be executed each time the 
   * protothread is scheduled */
  
  PT_BEGIN(&lwb_pt);   /* declare variables before this statement! */
    
  memset(&schedule, 0, sizeof(schedule));
  
  /* initialization specific to the host node */
  schedule_len = lwb_sched_init(&schedule);
  sync_state = SYNCED;  /* the host is always 'synced' */
  
  rtimer_reset();
  rt->time = 0;
  
  while(1) {
#if LWB_CONF_T_PREPROCESS
    if(pre_proc) {
      process_poll(pre_proc);
    }
  #if LWB_CONF_USE_LF_FOR_WAKEUP
    LWB_LF_WAIT_UNTIL(rt->time + LWB_CONF_T_PREPROCESS *
                      RTIMER_SECOND_LF / 1000);
  #else /* LWB_CONF_USE_LF_FOR_WAKEUP */
    LWB_WAIT_UNTIL(rt->time + LWB_CONF_T_PREPROCESS *
                   RTIMER_SECOND_HF / 1000)
  #endif /* LWB_CONF_USE_LF_FOR_WAKEUP */
#endif /* LWB_CONF_T_PREPROCESS */

#if LWB_CONF_USE_LF_FOR_WAKEUP 
    t_start_lf = rt->time; 
    rt->time = rtimer_now_hf();
    t_start = rt->time;
#else /* LWB_CONF_USE_LF_FOR_WAKEUP */
    /* set the start time of the round to the expiration time of the last 
     * scheduled timeout */
    t_start = rt->time;
#endif  /* LWB_CONF_USE_LF_FOR_WAKEUP */
        
    /* --- COMMUNICATION ROUND STARTS --- */
    
    LWB_SCHED_SET_AS_1ST(&schedule);          /* mark this schedule as first */
    LWB_SEND_SCHED();            /* send the previously computed schedule */

    global_time  = schedule.time;
    rx_timestamp = glossy_get_t_ref();

    glossy_rssi = glossy_get_rssi(0);
    stats.relay_cnt = glossy_get_relay_cnt_first_rx();
    slot_idx = 0;     /* reset the packet counter */
    
#if LWB_CONF_USE_XMEM
    /* put the external memory back into active mode (takes ~500us) */
    xmem_wakeup();    
#endif /* LWB_CONF_USE_XMEM */

    /* uncompress the schedule */
#if LWB_CONF_SCHED_COMPRESS
    lwb_sched_uncompress((uint8_t*)schedule.slot, 
                         LWB_SCHED_N_SLOTS(&schedule));
#endif /* LWB_CONF_SCHED_COMPRESS */
    
    /* --- S-ACK SLOT --- */
    
    if(LWB_SCHED_HAS_SACK_SLOT(&schedule)) {
      payload_len = lwb_sched_prepare_sack(&glossy_payload.sack_pkt); 
      /* wait for the slot to start */
      LWB_WAIT_UNTIL(t_start + LWB_T_SLOT_START(0));            
      LWB_SEND_PACKET();   /* transmit s-ack */
      DEBUG_PRINT_VERBOSE("S-ACK sent");
      slot_idx++;   /* increment the packet counter */
    } else {
      DEBUG_PRINT_VERBOSE("no S-ACK slot");
    }
         
    /* --- DATA SLOTS --- */
    
    rcvd_data_pkts = 0;    /* number of received data packets in this round */
    if(LWB_SCHED_HAS_DATA_SLOT(&schedule)) {
      static uint8_t i = 0;
      for(i = 0; i < LWB_SCHED_N_SLOTS(&schedule); i++, slot_idx++) {
        streams_to_update[i] = LWB_INVALID_STREAM_ID;
        /* is this our slot? Note: slots assigned to node ID 0 always belong 
         * to the host */
        if(schedule.slot[i] == 0 || schedule.slot[i] == node_id) {
          /* send a data packet (if there is any) */
          payload_len = lwb_out_buffer_get(glossy_payload.raw_data);
          if(payload_len) { 
            /* note: stream ID is irrelevant here */
            /* wait until the data slot starts */
            LWB_WAIT_UNTIL(t_start + LWB_T_SLOT_START(slot_idx));  
            LWB_SEND_PACKET();
            DEBUG_PRINT_VERBOSE("data packet sent (%ub)", payload_len);
          }
        } else {        
          /* wait until the data slot starts */
          LWB_DATA_SLOT_STARTS;
          LWB_WAIT_UNTIL(t_start + LWB_T_SLOT_START(slot_idx) - t_guard); 
          LWB_RCV_PACKET();  /* receive a data packet */
          payload_len = glossy_get_payload_len();
          if(LWB_DATA_RCVD && payload_len) {
            /* measure the time it takes to process the received message */
            RTIMER_CAPTURE;   
            if(glossy_payload.data_pkt.recipient == node_id || 
               glossy_payload.data_pkt.recipient == LWB_RECIPIENT_SINK ||
               glossy_payload.data_pkt.recipient == LWB_RECIPIENT_BROADCAST) {
              /* is it a stream request? (piggyback on data packet) */
              if(LWB_INVALID_STREAM_ID == glossy_payload.data_pkt.stream_id) {
                DEBUG_PRINT_VERBOSE("piggyback stream request from node %u", 
                                 glossy_payload.srq_pkt.id);
                lwb_sched_proc_srq((lwb_stream_req_t*)
                                   &glossy_payload.raw_data[3]);
              } else 
              {
                streams_to_update[i] = glossy_payload.data_pkt.stream_id;
                DEBUG_PRINT_VERBOSE("data received (s=%u.%u l=%u)", 
                                    schedule.slot[i], 
                                    glossy_payload.data_pkt.stream_id, 
                                    payload_len);
                /* replace target node ID by sender node ID */
                glossy_payload.data_pkt.recipient = schedule.slot[i];
                lwb_in_buffer_put(glossy_payload.raw_data, 
                                  payload_len);
              }
            } else {
              DEBUG_PRINT_VERBOSE("packet dropped, target_id != node_id");
            }
            /* update statistics */
            stats.rx_total += payload_len;
            stats.pck_cnt++;
            rcvd_data_pkts++;
            /* measure time (must always be smaller than LWB_CONF_T_GAP!) */
            stats.t_proc_max = MAX((uint16_t)RTIMER_ELAPSED, stats.t_proc_max);
          } else {
            DEBUG_PRINT_VERBOSE("no data received from node %u", 
                                schedule.slot[i]);
          }
        }
      }
    }
    
    /* --- CONTENTION SLOT --- */
    
    if(LWB_SCHED_HAS_CONT_SLOT(&schedule)) {
      /* wait until the slot starts, then receive the packet */
      LWB_WAIT_UNTIL(t_start + LWB_T_SLOT_START(slot_idx) - t_guard);
      LWB_RCV_SRQ();
      if(LWB_DATA_RCVD) {
        LWB_REQ_DETECTED;
        /* check the request */
        /*DEBUG_PRINT_INFO("stream request from node %u (stream %u, IPI %u)", 
                         glossy_payload.srq_pkt.id, 
                         glossy_payload.srq_pkt.stream_id, 
                         glossy_payload.srq_pkt.ipi);*/
        lwb_sched_proc_srq(&glossy_payload.srq_pkt);
      }
    }

    /* compute the new schedule */
    RTIMER_CAPTURE;
    schedule_len = lwb_sched_compute(&schedule, 
                                     streams_to_update, 
                                     lwb_get_send_buffer_state());
    stats.t_sched_max = MAX((uint16_t)RTIMER_ELAPSED, stats.t_sched_max);

    LWB_WAIT_UNTIL(t_start + LWB_CONF_T_SCHED2_START);
    LWB_SEND_SCHED();    /* send the schedule for the next round */
    
    /* --- COMMUNICATION ROUND ENDS --- */
    /* time for other computations */
    
    /* print out some stats */
    DEBUG_PRINT_INFO("t=%lu ts=%u td=%u dp=%u p=%u per=%d rssi=%ddBm", 
                     global_time,
                     stats.t_sched_max, 
                     stats.t_proc_max, 
                     rcvd_data_pkts, 
                     stats.pck_cnt,
                     glossy_get_per(),
                     glossy_rssi);
        
#if LWB_CONF_STATS_NVMEM
    lwb_stats_save();
#endif /* LWB_CONF_STATS_NVMEM */
    /* poll the other processes to allow them to run after the LWB task was 
     * suspended (note: the polled processes will be executed in the inverse
     * order they were started/created) */
    debug_print_poll();
    if(post_proc) {
      /* will be executed before the debug print task */
      process_poll(post_proc);    
    }
    
    /* suspend this task and wait for the next round */
#if LWB_CONF_USE_LF_FOR_WAKEUP
    LWB_LF_WAIT_UNTIL(t_start_lf + 
                      (rtimer_clock_t)schedule.period * RTIMER_SECOND_LF / 
                      LWB_CONF_TIME_SCALE - LWB_CONF_T_PREPROCESS *
                      RTIMER_SECOND_LF / 1000);
#else /* LWB_CONF_USE_LF_FOR_WAKEUP */
    LWB_WAIT_UNTIL(t_start + 
                   (rtimer_clock_t)schedule.period * RTIMER_SECOND_HF /
                   LWB_CONF_TIME_SCALE - 
                   LWB_CONF_T_PREPROCESS * RTIMER_SECOND_HF / 1000);
#endif /* LWB_CONF_USE_LF_FOR_WAKEUP */
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
  static glossy_payload_t glossy_payload;                   /* packet buffer */
  static lwb_schedule_t schedule;
#if !LWB_CONF_RELAY_ONLY
  static rtimer_clock_t t_now; 
#endif /* LWB_CONF_RELAY_ONLY */
  static rtimer_clock_t t_ref;
#if LWB_CONF_USE_LF_FOR_WAKEUP
  static rtimer_clock_t t_ref_lf;
#endif /* LWB_CONF_USE_LF_FOR_WAKEUP */
#if LWB_CONF_TIME_SCALE == 1
  static rtimer_clock_t t_ref_last;
  static int32_t  drift = 0;
  static uint16_t period_last = LWB_CONF_SCHED_PERIOD_MIN;
#endif /* LWB_CONF_TIME_SCALE == 1 */
  static uint32_t t_guard;                  /* 32-bit is enough for t_guard! */
  static uint8_t  slot_idx;
  static uint8_t  relay_cnt_first_rx;
#if !LWB_CONF_RELAY_ONLY
  static uint8_t  payload_len;
  static uint8_t  rounds_to_wait;
#endif /* LWB_CONF_RELAY_ONLY */
  static int8_t   glossy_snr = 0;
  static const void* callback_func = lwb_thread_src;
  
  PT_BEGIN(&lwb_pt);   /* declare variables before this statement! */
  
  memset(&schedule, 0, sizeof(schedule)); 
  
  /* initialization specific to the source node */
  lwb_stream_init();
  sync_state = BOOTSTRAP;
  
  while(1) {
      
#if LWB_CONF_T_PREPROCESS
    if(pre_proc) {
      process_poll(pre_proc);
    }
  #if LWB_CONF_USE_LF_FOR_WAKEUP
    LWB_LF_WAIT_UNTIL(rt->time + 
                      LWB_CONF_T_PREPROCESS * RTIMER_SECOND_LF / 1000);
  #else /* LWB_CONF_USE_LF_FOR_WAKEUP */
    LWB_WAIT_UNTIL(rt->time + 
                   LWB_CONF_T_PREPROCESS * RTIMER_SECOND_HF / 1000)
  #endif /* LWB_CONF_USE_LF_FOR_WAKEUP */
#endif /* LWB_CONF_T_PREPROCESS */
    
    /* --- COMMUNICATION ROUND STARTS --- */
    
#if LWB_CONF_USE_LF_FOR_WAKEUP
    rt->time = rtimer_now_hf();        /* overwrite LF with HF timestamp */
    t_ref = rt->time + t_guard;        /* in case the schedule is missed */
#endif /* LWB_CONF_USE_LF_FOR_WAKEUP */
    
    if(sync_state == BOOTSTRAP) {
BOOTSTRAP_MODE:
      DEBUG_PRINT_MSG_NOW("BOOTSTRAP ");
      stats.bootstrap_cnt++;
      lwb_stream_rejoin();  /* rejoin all (active) streams */
      /* synchronize first! wait for the first schedule... */
      do {
        LWB_RCV_SCHED();
        if((rtimer_now_hf() - t_ref) > LWB_CONF_T_SILENT) {
          DEBUG_PRINT_MSG_NOW("communication timeout, going to sleep...");
          stats.sleep_cnt++;
          LWB_BEFORE_DEEPSLEEP();
          LWB_LF_WAIT_UNTIL(rtimer_now_lf() + LWB_CONF_T_DEEPSLEEP);
          t_ref = rtimer_now_hf();
          /* alternative: implement a host failover policy */
        }
      } while(!glossy_is_t_ref_updated() || !LWB_SCHED_IS_1ST(&schedule));
      /* schedule received! */
    } else {
      LWB_RCV_SCHED();  
    }
    glossy_snr = glossy_get_snr();

#if LWB_CONF_USE_XMEM
    /* put the external memory back into active mode (takes ~500us) */
    xmem_wakeup();    
#endif /* LWB_CONF_USE_XMEM */
           
    /* update the sync state machine (compute new sync state and update 
     * t_guard) */
    LWB_UPDATE_SYNC_STATE;  
    if(BOOTSTRAP == sync_state) {
      /* something went wrong */
      goto BOOTSTRAP_MODE;
    } 
    LWB_SCHED_SET_AS_2ND(&schedule);    /* clear the last bit of 'period' */
    if(glossy_is_t_ref_updated()) {
      /* HF timestamp of first RX; subtract a constant offset */
      t_ref = glossy_get_t_ref() - LWB_CONF_T_REF_OFS;           
  #if LWB_CONF_USE_LF_FOR_WAKEUP
      /* estimate t_ref_lf by subtracting the elapsed time since t_ref: */
      rtimer_clock_t hf_now;
      rtimer_now(&hf_now, &t_ref_lf);
      t_ref_lf -= (uint32_t)(hf_now - t_ref) / (uint32_t)RTIMER_HF_LF_RATIO;
  #endif /* LWB_CONF_USE_LF_FOR_WAKEUP */
      global_time = schedule.time;
      rx_timestamp = t_ref;
    } else {
      DEBUG_PRINT_WARNING("schedule missed");
      /* we can only estimate t_ref and t_ref_lf */
  #if LWB_CONF_USE_LF_FOR_WAKEUP
      /* since HF clock was off, we need a new timestamp; subtract a const.
       * processing offset to adjust (if needed) */
      t_ref_lf += ((rtimer_clock_t)schedule.period * RTIMER_SECOND_LF +
                  ((int32_t)schedule.period * stats.drift / 256)) /
                  LWB_CONF_TIME_SCALE;
      /* do not update t_ref here */
  #else  /* LWB_CONF_USE_LF_FOR_WAKEUP */
      t_ref += (rtimer_clock_t)schedule.period *
               (RTIMER_SECOND_HF + stats.drift / 16) / LWB_CONF_TIME_SCALE;
  #endif /* LWB_CONF_USE_LF_FOR_WAKEUP */
      /* don't update schedule.time here! */
    }

    /* permission to participate in this round? */
    if(sync_state == SYNCED || sync_state == UNSYNCED) {
        
      static uint8_t i;  /* must be static */      
      slot_idx = 0;   /* reset the packet counter */
      relay_cnt_first_rx = glossy_get_relay_cnt_first_rx();
#if LWB_CONF_SCHED_COMPRESS
      lwb_sched_uncompress((uint8_t*)schedule.slot, 
                           LWB_SCHED_N_SLOTS(&schedule));
#endif /* LWB_CONF_SCHED_COMPRESS */
      
      /* --- S-ACK SLOT --- */

      if(LWB_SCHED_HAS_SACK_SLOT(&schedule)) {   
        /* wait for the slot to start */
        LWB_WAIT_UNTIL(t_ref + LWB_T_SLOT_START(0) - t_guard);     
        LWB_RCV_PACKET();                 /* receive s-ack */
  #if !LWB_CONF_RELAY_ONLY
        if(LWB_DATA_RCVD) {
          static uint8_t i; /* must be static */
          i = 0;            /* must be a separate line of code */
          DEBUG_PRINT_VERBOSE("S-ACK packet received (%u stream acks)", 
                              glossy_payload.sack_pkt.n_extra + 1);
          do {
            /* note: potential cause of problems (but: structure glossy_payload
             * should be aligned to an even address!) */
            if(*(uint16_t*)(glossy_payload.raw_data + (i * 4)) == node_id) {     
              uint8_t stream_id = *(uint8_t*)(glossy_payload.raw_data + 
                                  (i * 4 + 2));
              stats.t_slot_last = schedule.time;
              rounds_to_wait = 0;
              if(lwb_stream_update_state(stream_id)) {
                DEBUG_PRINT_INFO("S-ACK received for stream %u (joined)", 
                                 stream_id);
              } else {
                DEBUG_PRINT_INFO("S-ACK received for stream %u (removed)", 
                                 stream_id);
              }
            } 
            i++;
          } while(i <= glossy_payload.sack_pkt.n_extra);
        } else {
          DEBUG_PRINT_VERBOSE("no data received in SACK SLOT");
        }
  #endif /* LWB_CONF_RELAY_ONLY */
        slot_idx++;   /* increment the packet counter */
      }
      
      /* --- DATA SLOTS --- */

      if(LWB_SCHED_HAS_DATA_SLOT(&schedule)) {
        for(i = 0; i < LWB_SCHED_N_SLOTS(&schedule); i++, slot_idx++) {
  #if !LWB_CONF_RELAY_ONLY
          if(schedule.slot[i] == node_id) {
            stats.t_slot_last = schedule.time;
            /* this is our data slot, send a data packet */
            payload_len = 0;
            /* is there an 'urgent' stream request? -> if so, piggyback it 
             * onto the data packet */
            if(urgent_stream_req) {
              glossy_payload.data_pkt.recipient = LWB_RECIPIENT_SINK;
              glossy_payload.data_pkt.stream_id = LWB_INVALID_STREAM_ID;
              if(lwb_stream_prepare_req((lwb_stream_req_t*)
                                        &glossy_payload.raw_data[3], 
                                        urgent_stream_req)) {
                payload_len = sizeof(lwb_stream_req_t) + 3;
              }
              DEBUG_PRINT_VERBOSE("piggyback stream request prepared");
            } else {
              /* fetch the next 'ready-to-send' packet */
              payload_len = lwb_out_buffer_get(glossy_payload.raw_data);
            }
            if(payload_len) {
              LWB_DATA_IND;
              LWB_WAIT_UNTIL(t_ref + LWB_T_SLOT_START(slot_idx));
              LWB_SEND_PACKET();
              DEBUG_PRINT_INFO("data packet sent (%ub)", payload_len);
            } else {              
              DEBUG_PRINT_VERBOSE("no message to send (data slot ignored)");
            }
          } else
  #endif /* LWB_CONF_RELAY_ONLY */
          {
            /* receive a data packet */
            LWB_WAIT_UNTIL(t_ref + LWB_T_SLOT_START(slot_idx) - 
                           t_guard);
            LWB_RCV_PACKET();
            payload_len = glossy_get_payload_len();
  #if !LWB_CONF_RELAY_ONLY
            /* process the received data */
            if(LWB_DATA_RCVD && payload_len) {
              /* measure the time it takes to process the received data */
              RTIMER_CAPTURE;     
              /* only forward packets that are destined for this node */
              if(glossy_payload.data_pkt.recipient == node_id || 
                glossy_payload.data_pkt.recipient == LWB_RECIPIENT_BROADCAST) {
                DEBUG_PRINT_VERBOSE("data received");
                /* replace target node ID by sender node ID */
                glossy_payload.data_pkt.recipient = schedule.slot[i];
                lwb_in_buffer_put(glossy_payload.raw_data, 
                                  payload_len);
              } else {
                DEBUG_PRINT_VERBOSE("received packet dropped");      
              }
              /* must always be smaller than LWB_CONF_T_GAP */
              stats.t_proc_max = MAX((uint16_t)RTIMER_ELAPSED, 
                                     stats.t_proc_max); 
            } /* else: no data received */   
  #endif /* LWB_CONF_RELAY_ONLY */
            stats.rx_total += payload_len;
            stats.pck_cnt++;
          }
        }
      }
      
      /* --- CONTENTION SLOT --- */

      /* is there a contention slot in this round? */
      if(LWB_SCHED_HAS_CONT_SLOT(&schedule)) {
  #if !LWB_CONF_RELAY_ONLY
        /* does this node have pending stream requests? */
        if(LWB_STREAM_REQ_PENDING) {
          if(!rounds_to_wait) {              /* allowed to send the request? */
            /* try to join, send the stream request */
            if(lwb_stream_prepare_req(&glossy_payload.srq_pkt, 
                                      LWB_INVALID_STREAM_ID)) {
      #if LWB_CONF_MAX_CONT_BACKOFF
              /* wait between 1 and LWB_CONF_MAX_CONT_BACKOFF rounds */
              rounds_to_wait = (random_rand() >> 1) % 
                               LWB_CONF_MAX_CONT_BACKOFF + 1;
      #endif /* LWB_CONF_MAX_CONT_BACKOFF */
              payload_len = sizeof(lwb_stream_req_t);
              /* wait until the contention slot starts */
              LWB_REQ_IND;
              LWB_WAIT_UNTIL(t_ref + LWB_T_SLOT_START(slot_idx));
              LWB_SEND_SRQ();  
              DEBUG_PRINT_INFO("request for stream %u sent", 
                               glossy_payload.srq_pkt.stream_id);
            } /* else {
              DEBUG_PRINT_ERROR("failed to prepare stream request packet");
            }*/
          } else {
            DEBUG_PRINT_VERBOSE("must wait %u rounds", rounds_to_wait);
            /* keep waiting and just relay incoming packets */
            rounds_to_wait--;       /* decrease the number of rounds to wait */
            /* wait until the contention slot starts */
            LWB_WAIT_UNTIL(t_ref + LWB_T_SLOT_START(slot_idx) - t_guard);  
            LWB_RCV_SRQ();
          }
          DEBUG_PRINT_VERBOSE("pending stream requests: 0x%x", 
                              LWB_STREAM_REQ_PENDING);
        } else {
  #endif /* LWB_CONF_RELAY_ONLY */
          /* no request pending -> just receive / relay packets */
          LWB_WAIT_UNTIL(t_ref + LWB_T_SLOT_START(slot_idx) - t_guard);
          LWB_RCV_SRQ();
                  
  #if !LWB_CONF_RELAY_ONLY
        }
  #endif /* LWB_CONF_RELAY_ONLY */ 
      }
    }  
    
    /* --- 2ND SCHEDULE --- */

    LWB_WAIT_UNTIL(t_ref + LWB_CONF_T_SCHED2_START - t_guard);
    LWB_RCV_SCHED();
  
    /* update the state machine and the guard time */
    LWB_UPDATE_SYNC_STATE;
    if(BOOTSTRAP == sync_state) {
      goto BOOTSTRAP_MODE;
    }
    
    /* --- COMMUNICATION ROUND ENDS --- */    
    /* time for other computations */

#if !LWB_CONF_RELAY_ONLY
    /* check joining state */
    if(LWB_STREAMS_ACTIVE && ((schedule.time - stats.t_slot_last) > 
                              (LWB_CONF_SCHED_PERIOD_MAX << 2))) {
      DEBUG_PRINT_WARNING("no-slot timeout, requesting new stream...");
      lwb_stream_rejoin();  /* re-join (all streams) */
      stats.t_slot_last = schedule.time;
    }
#endif /* !LWB_CONF_RELAY_ONLY */
    
    /* estimate the clock drift */
#if (LWB_CONF_TIME_SCALE == 1)  /* only calc drift if time scale is not used */
  #if LWB_CONF_USE_LF_FOR_WAKEUP
    /* t_ref can't be used in this case -> use t_ref_lf instead */
    drift = ((int32_t)((t_ref_lf - t_ref_last) - ((int32_t)period_last *
                       RTIMER_SECOND_LF)) * 256) / (int32_t)period_last;
    t_ref_last = t_ref_lf;     
  #else /* LWB_CONF_USE_LF_FOR_WAKEUP */
    drift = ((int32_t)((t_ref - t_ref_last) - ((int32_t)period_last *
                       RTIMER_SECOND_HF)) * 16) / period_last;
    t_ref_last = t_ref; 
  #endif /* LWB_CONF_USE_LF_FOR_WAKEUP */

    if(sync_state <= MISSED) {
  #if LWB_CONF_USE_LF_FOR_WAKEUP
      /* convert LWB_CONF_MAX_CLOCK_DEV into clock ticks per second */
      if((drift < (int16_t)(LWB_CONF_MAX_CLOCK_DEV * RTIMER_SECOND_LF * 256 /
                            1000000)) &&
         (drift > -(int16_t)(LWB_CONF_MAX_CLOCK_DEV * RTIMER_SECOND_LF * 256 /
                             1000000))) {
  #else /* LWB_CONF_USE_LF_FOR_WAKEUP */
      if((drift < (int16_t)(LWB_CONF_MAX_CLOCK_DEV * RTIMER_SECOND_HF * 16 /
                            1000000)) &&
         (drift > -(int16_t)(LWB_CONF_MAX_CLOCK_DEV * RTIMER_SECOND_HF * 16 /
                             1000000))) {
  #endif /* LWB_CONF_USE_LF_FOR_WAKEUP */
        stats.drift = (stats.drift + drift) / 2;  /* low-pass filter */
      }
    }
#endif /* LWB_CONF_TIME_SCALE */

    if(sync_state > SYNCED_2) {
      stats.unsynced_cnt++;
    }
    /* print out some stats (note: takes approx. 2ms to compose this string) */
    DEBUG_PRINT_INFO("%s %lu T=%u n=%u s=%u tp=%u p=%u r=%u b=%u "
                     "u=%u dr=%d per=%d snr=%d",
                     lwb_sync_state_to_string[sync_state], 
                     schedule.time, 
                     schedule.period, 
                     LWB_SCHED_N_SLOTS(&schedule), 
                     LWB_STREAMS_ACTIVE, 
                     stats.t_proc_max,
                     stats.pck_cnt,
                     relay_cnt_first_rx,
                     stats.bootstrap_cnt, 
                     stats.unsynced_cnt, 
                     stats.drift, /* in clock ticks per second (x16 or x256) */
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
    debug_print_poll();
    if(post_proc) {
      process_poll(post_proc);
    }
    
#if LWB_CONF_USE_LF_FOR_WAKEUP
    LWB_LF_WAIT_UNTIL(t_ref_lf + 
                      ((rtimer_clock_t)schedule.period * RTIMER_SECOND_LF + 
                       (((int32_t)schedule.period * stats.drift) / 256)) /
                      LWB_CONF_TIME_SCALE - 
                      t_guard / (uint32_t)RTIMER_HF_LF_RATIO - 
                      LWB_CONF_T_PREPROCESS * RTIMER_SECOND_LF / 1000);
#else /* LWB_CONF_USE_LF_FOR_WAKEUP */
    LWB_WAIT_UNTIL(t_ref + 
                   ((rtimer_clock_t)schedule.period * RTIMER_SECOND_HF +
                    (((int32_t)schedule.period * stats.drift) / 16)) /
                   LWB_CONF_TIME_SCALE - t_guard - 
                   LWB_CONF_T_PREPROCESS * RTIMER_SECOND_HF / 1000);
#endif /* LWB_CONF_USE_LF_FOR_WAKEUP */
  }

  PT_END(&lwb_pt);
}
/*---------------------------------------------------------------------------*/
void
lwb_pause(void)
{
  rtimer_stop(LWB_CONF_RTIMER_ID);    /* cancel the rtimer */
  rtimer_stop(LWB_CONF_LF_RTIMER_ID);
}
/*---------------------------------------------------------------------------*/
void
lwb_resume(void)
{
#if LWB_CONF_USE_LF_FOR_WAKEUP
  rtimer_clock_t start_time = rtimer_now_lf() + RTIMER_SECOND_LF / 10;
  rtimer_id_t timer_id = LWB_CONF_LF_RTIMER_ID;
#else /* LWB_CONF_USE_LF_FOR_WAKEUP */
  rtimer_clock_t start_time = rtimer_now_hf() + RTIMER_SECOND_HF / 10;
  rtimer_id_t timer_id = LWB_CONF_RTIMER_ID;
#endif /* LWB_CONF_USE_LF_FOR_WAKEUP */
    
  if(node_id == HOST_ID) {
    /* note: must add at least some clock ticks! */
    rtimer_schedule(timer_id, start_time, 0, lwb_thread_host);
  } else {
    rtimer_schedule(timer_id, start_time, 0, lwb_thread_src);
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

  lwb_resume();

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
void
lwb_start(struct process* pre_lwb_proc, struct process *post_lwb_proc)
{
  pre_proc  = (struct process*)pre_lwb_proc;
  post_proc = (struct process*)post_lwb_proc;
  
  uart_enable(1);
  printf("Starting '%s'\r\n", lwb_process.name);    
  printf("t_sched=%ums, t_data=%ums, t_cont=%ums, t_round=%ums, "
         "data=%ub, slots=%u, tx=%u, hop=%u, scale=%u, sched2=%ums\r\n", 
         (uint16_t)RTIMER_HF_TO_MS(LWB_CONF_T_SCHED),
         (uint16_t)RTIMER_HF_TO_MS(LWB_CONF_T_DATA),
         (uint16_t)RTIMER_HF_TO_MS(LWB_CONF_T_CONT),
         (uint16_t)RTIMER_HF_TO_MS(LWB_T_ROUND_MAX),
         LWB_CONF_MAX_DATA_PKT_LEN, 
         LWB_CONF_MAX_DATA_SLOTS, 
         LWB_CONF_TX_CNT_DATA, 
         LWB_CONF_MAX_HOPS,
         LWB_CONF_TIME_SCALE, 
         (uint16_t)RTIMER_HF_TO_MS(LWB_CONF_T_SCHED2_START));  
  if((LWB_CONF_T_SCHED2_START + LWB_CONF_T_SCHED) > 
     (RTIMER_SECOND_HF / LWB_CONF_TIME_SCALE)) {
    printf("WARNING: LWB_CONF_T_SCHED2_START > min round period\r\n");
  }
  if(LWB_CONF_T_SCHED2_START < (LWB_T_ROUND_MAX / LWB_CONF_TIME_SCALE)) {
    printf("WARNING: LWB_CONF_T_SCHED2_START < LWB_T_ROUND_MAX!");
  }
  process_start(&lwb_process, NULL);
}
/*---------------------------------------------------------------------------*/

#endif /* LWB_VERSION */
