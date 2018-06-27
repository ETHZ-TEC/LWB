/*
 * Copyright (c) 2018, Swiss Federal Institute of Technology (ETH Zurich).
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
 * Version: 2.2
 */

/**
 * @file
 *
 * a modified implementation of the Low-Power Wireless Bus called e-LWB
 * (event-based LWB)
 * 
 * eLWB does not have a specific header, therefore ELWB_CONF_MAX_DATA_PKT_LEN
 * = ELWB_CONF_MAX_PKT_LEN
 * packet filtering is possible on the source nodes with the macro
 * ELWB_CONF_SRC_PKT_FILTER
 */
 
#include "main.h"

/*---------------------------------------------------------------------------*/
/* internal sync state of the eLWB */
typedef enum {
  BOOTSTRAP = 0,
  SYNCED,
  UNSYNCED,
  UNSYNCED2,
  NUM_OF_SYNC_STATES
} elwb_syncstate_t;
/*---------------------------------------------------------------------------*/
typedef enum {
  EVT_SCHED_RCVD = 0,
  EVT_SCHED_MISSED,
  NUM_OF_SYNC_EVENTS
} sync_event_t;
/*---------------------------------------------------------------------------*/
#pragma pack(1)     /* force alignment to 1 byte */
typedef struct {
  uint8_t   len;
  uint8_t   data[ELWB_CONF_MAX_PKT_LEN];
} elwb_queue_elem_t;
#pragma pack()
/*---------------------------------------------------------------------------*/
static const 
elwb_syncstate_t next_state[NUM_OF_SYNC_EVENTS][NUM_OF_SYNC_STATES] = 
{/* STATES:                                         EVENTS:         */
 /* BOOTSTRAP, SYNCED,   UNSYNCED,  UNSYNCED2                       */
  { SYNCED,    SYNCED,   SYNCED,    SYNCED    }, /* schedule rcvd   */
  { BOOTSTRAP, UNSYNCED, UNSYNCED2, BOOTSTRAP }  /* schedule missed */
};
static const char* elwb_syncstate_to_string[NUM_OF_SYNC_STATES] = {
  "BOOTSTRAP", "SYN", "USYN", "USYN2"
};
/*---------------------------------------------------------------------------*/
#ifdef ELWB_CONF_TASK_ACT_PIN
  #define ELWB_TASK_RESUMED        PIN_CLR(ELWB_CONF_TASK_ACT_PIN); \
                                   PIN_SET(ELWB_CONF_TASK_ACT_PIN)
  #define ELWB_TASK_SUSPENDED      PIN_CLR(ELWB_CONF_TASK_ACT_PIN)
#else /* ELWB_CONF_TASK_ACT_PIN */
  #define ELWB_TASK_RESUMED
  #define ELWB_TASK_SUSPENDED
#endif /* ELWB_CONF_TASK_ACT_PIN */
#if ELWB_CONF_PREEMPTION
  #define ELWB_ENABLE_PREEMPTION   { __eint(); __nop(); }
#else /* ELWB_CONF_PREEMPTION */
  #define ELWB_ENABLE_PREEMPTION
#endif /* ELWB_CONF_PREEMPTION */
/*---------------------------------------------------------------------------*/
#define ELWB_DATA_RCVD             (glossy_get_n_rx() > 0)
/*---------------------------------------------------------------------------*/
#define ELWB_SEND_SCHED() \
{\
  glossy_start(node_id, (uint8_t *)&schedule, schedule_len, \
               ELWB_CONF_N_TX_SCHED, GLOSSY_WITH_SYNC, GLOSSY_WITH_RF_CAL);\
  ELWB_WAIT_UNTIL(rt->time + ELWB_CONF_T_SCHED);\
  glossy_stop();\
}
#define ELWB_RCV_SCHED() \
{\
  glossy_start(GLOSSY_UNKNOWN_INITIATOR, (uint8_t *)&schedule, payload_len, \
               ELWB_CONF_N_TX_SCHED, GLOSSY_WITH_SYNC, GLOSSY_WITH_RF_CAL);\
  ELWB_WAIT_UNTIL(rt->time + ELWB_CONF_T_SCHED + ELWB_CONF_T_GUARD);\
  glossy_stop();\
}
#define ELWB_SEND_PACKET() \
{\
  glossy_start(node_id, (uint8_t*)glossy_payload, payload_len, \
               ELWB_CONF_N_TX_DATA, GLOSSY_WITHOUT_SYNC, \
               GLOSSY_WITHOUT_RF_CAL);\
  ELWB_WAIT_UNTIL(rt->time + t_slot);\
  glossy_stop();\
}
#define ELWB_RCV_PACKET() \
{\
  glossy_start(GLOSSY_UNKNOWN_INITIATOR, (uint8_t*)glossy_payload, \
               payload_len, \
               ELWB_CONF_N_TX_DATA, GLOSSY_WITHOUT_SYNC, \
               GLOSSY_WITHOUT_RF_CAL);\
  ELWB_WAIT_UNTIL(rt->time + t_slot + ELWB_CONF_T_GUARD);\
  glossy_stop();\
}
/*---------------------------------------------------------------------------*/
/* suspend the eLWB proto-thread until the rtimer reaches the specified time */
#define ELWB_WAIT_UNTIL(time) \
{\
  rtimer_schedule(ELWB_CONF_RTIMER_ID, time, 0, elwb_cb);\
  ELWB_TASK_SUSPENDED;\
  PT_YIELD(&elwb_pt);\
  ELWB_TASK_RESUMED;\
  ELWB_ENABLE_PREEMPTION; \
}
/* same as ELWB_WAIT_UNTIL, but use the LF timer to schedule the wake-up */
#define ELWB_LF_WAIT_UNTIL(time) \
{\
  rtimer_schedule(ELWB_CONF_LF_RTIMER_ID, time, 0, elwb_cb);\
  ELWB_TASK_SUSPENDED;\
  PT_YIELD(&elwb_pt);\
  ELWB_TASK_RESUMED;\
  ELWB_ENABLE_PREEMPTION; \
}
#define ELWB_BEFORE_DEEPSLEEP     BEFORE_DEEPSLEEP
#define ELWB_AFTER_DEEPSLEEP      AFTER_DEEPSLEEP
/*---------------------------------------------------------------------------*/
static struct pt          elwb_pt;
static struct process*    post_proc;
static struct process*    pre_proc;
static elwb_schedule_t    schedule;      /* use standard LWB schedule struct */
static elwb_stats_t       stats = { 0 };
static rtimer_clock_t     last_synced_hf;
static rtimer_clock_t     last_synced_lf;
static uint32_t           global_time;
static uint32_t           t_preprocess;
static uint32_t           t_slot;
static uint32_t           t_slot_ofs;
static uint16_t           glossy_payload[(ELWB_CONF_MAX_PKT_LEN + 1) / 2];
static uint8_t            payload_len;
static const void*        elwb_cb;
#if !ELWB_CONF_USE_XMEM
static elwb_queue_elem_t  rx_queue_mem[ELWB_CONF_IN_BUFFER_SIZE];
static elwb_queue_elem_t  tx_queue_mem[ELWB_CONF_OUT_BUFFER_SIZE];
#else /* ELWB_CONF_USE_XMEM */
static elwb_queue_elem_t  xmem_buffer;
#endif /* ELWB_CONF_USE_XMEM */
FIFO(rx_queue, sizeof(elwb_queue_elem_t), ELWB_CONF_IN_BUFFER_SIZE);
FIFO(tx_queue, sizeof(elwb_queue_elem_t), ELWB_CONF_OUT_BUFFER_SIZE);
/*---------------------------------------------------------------------------*/
#if ELWB_CONF_DATA_ACK
void
elwb_requeue_pkt(uint32_t pkt_addr)
{
  /* re-insert the packet into the tx queue */
  /* find an empty spot in the queue */
  uint32_t new_pkt_addr = fifo_put(&tx_queue);
  if(new_pkt_addr != FIFO_ERROR) {
    if(new_pkt_addr == pkt_addr) {
      return; /* same address? -> nothing to do */
    }
#if !ELWB_CONF_USE_XMEM
    memcpy((uint8_t*)(uint16_t)new_pkt_addr, (uint8_t*)(uint16_t)pkt_addr, 
           sizeof(elwb_queue_elem_t));
#else /* ELWB_CONF_USE_XMEM */
    xmem_read(pkt_addr, sizeof(elwb_queue_elem_t), (uint8_t*)&xmem_buffer);
    xmem_write(new_pkt_addr, xmem_buffer.len + 1, (uint8_t*)&xmem_buffer);
#endif /* ELWB_CONF_USE_XMEM */
    DEBUG_PRINT_VERBOSE("packet requeued");
  } else {
    stats.txbuf_drop++;
    DEBUG_PRINT_ERROR("requeue failed, out queue full");
  }
}
#endif /* ELWB_CONF_DATA_ACK */
/*---------------------------------------------------------------------------*/
/* store a received message in the incoming queue, returns 1 if successful, 
 * 0 otherwise */
uint8_t 
elwb_in_buffer_put(uint8_t* data, uint8_t len)
{
  if(!len || len > ELWB_CONF_MAX_PKT_LEN) {
    DEBUG_PRINT_WARNING("lwb: invalid packet received");
    return 0;
  }
  /* received messages will have the max. length ELWB_CONF_MAX_PKT_LEN */
  uint32_t pkt_addr = fifo_put(&rx_queue);
  if(FIFO_ERROR != pkt_addr) {
#if !ELWB_CONF_USE_XMEM
    /* copy the data into the queue */
    elwb_queue_elem_t* next_msg = (elwb_queue_elem_t*)((uint16_t)pkt_addr);
    memcpy(next_msg->data, data, len);
    next_msg->len = len;
#else /* ELWB_CONF_USE_XMEM */
    memcpy(xmem_buffer.data, data, len);
    xmem_buffer.len = len;
    xmem_write(pkt_addr, len + 1, (uint8_t*)&xmem_buffer);
#endif /* ELWB_CONF_USE_XMEM */
    return 1;
  }
  stats.rxbuf_drop++;
  DEBUG_PRINT_WARNING("eLWB RX queue full");
  return 0;
}
/*---------------------------------------------------------------------------*/
/* fetch the next 'ready-to-send' message from the outgoing queue
 * returns 1 if successful, 0 otherwise */
uint8_t 
elwb_out_buffer_get(uint8_t* out_data, uint8_t* out_len)
{
  /* messages have the max. length ELWB_CONF_MAX_PKT_LEN and are already
   * formatted */
  uint32_t pkt_addr = fifo_get(&tx_queue);
  if(FIFO_ERROR != pkt_addr) {
#if !ELWB_CONF_USE_XMEM
    /* assume pointers are always 16-bit */
    elwb_queue_elem_t* next_msg = (elwb_queue_elem_t*)((uint16_t)pkt_addr);  
    memcpy(out_data, next_msg->data, next_msg->len);
    *out_len = next_msg->len;
#else /* ELWB_CONF_USE_XMEM */
    xmem_read(pkt_addr, sizeof(elwb_queue_elem_t), (uint8_t*)&xmem_buffer);
    memcpy(out_data, xmem_buffer.data, xmem_buffer.len);
    *out_len = xmem_buffer.len;
#endif /* ELWB_CONF_USE_XMEM */
    return 1;
  }
  DEBUG_PRINT_VERBOSE("eLWB TX queue empty");
  return 0;
}
/*---------------------------------------------------------------------------*/
/* puts a message into the outgoing queue, returns 1 if successful, 
 * 0 otherwise */
uint8_t
elwb_send_pkt(const uint8_t* const data, uint8_t len)
{
  /* data has the max. length ELWB_CONF_MAX_PKT_LEN */
  if(len > ELWB_CONF_MAX_PKT_LEN || !data) {
    return 0;
  }
  uint32_t pkt_addr = fifo_put(&tx_queue);
  if(FIFO_ERROR != pkt_addr) {
#if !ELWB_CONF_USE_XMEM
    /* assume pointers are 16-bit */
    elwb_queue_elem_t* next_msg = (elwb_queue_elem_t*)((uint16_t)pkt_addr);
    next_msg->len = len;
    memcpy(next_msg->data, data, len);
#else /* ELWB_CONF_USE_XMEM */
    memcpy(xmem_buffer.data, data, len);
    xmem_buffer.len = len;
    xmem_wait_until_ready();
    xmem_write(pkt_addr, len + 1, (uint8_t*)&xmem_buffer);
#endif /* ELWB_CONF_USE_XMEM */
    DEBUG_PRINT_VERBOSE("msg added to eLWB TX queue");
    return 1;
  }
  stats.txbuf_drop++;
  DEBUG_PRINT_VERBOSE("eLWB TX queue full");
  return 0;
}
/*---------------------------------------------------------------------------*/
/* copies the oldest received message in the queue into out_data and returns 
 * the message size (in bytes) */
uint8_t
elwb_rcv_pkt(uint8_t* out_data)
{ 
  if(!out_data) { return 0; }
  /* messages in the queue have the max. length ELWB_CONF_MAX_PKT_LEN */
  uint32_t pkt_addr = fifo_get(&rx_queue);
  if(FIFO_ERROR != pkt_addr) {
#if !ELWB_CONF_USE_XMEM
    /* assume pointers are 16-bit */
    elwb_queue_elem_t* next_msg = (elwb_queue_elem_t*)((uint16_t)pkt_addr); 
    memcpy(out_data, next_msg->data, next_msg->len);
    return next_msg->len;
#else /* ELWB_CONF_USE_XMEM */
    if(!xmem_read(pkt_addr, sizeof(elwb_queue_elem_t), 
                  (uint8_t*)&xmem_buffer)) {
      return 0;
    }
    xmem_wait_until_ready(); /* wait for the data transfer to complete */
    memcpy(out_data, xmem_buffer.data, xmem_buffer.len);
    return xmem_buffer.len;
#endif /* ELWB_CONF_USE_XMEM */
  }
  DEBUG_PRINT_VERBOSE("lwb rx queue empty");
  return 0;
}
/*---------------------------------------------------------------------------*/
uint8_t
elwb_get_rcv_buffer_state(void)
{
  return FIFO_CNT(&rx_queue);
}
/*---------------------------------------------------------------------------*/
uint8_t
elwb_get_send_buffer_state(void)
{
  return FIFO_CNT(&tx_queue);
}
/*---------------------------------------------------------------------------*/
const elwb_stats_t * const
elwb_get_stats(void)
{
  return &stats;
}
/*---------------------------------------------------------------------------*/
uint32_t 
elwb_get_time(rtimer_clock_t* reception_time)
{
  if(reception_time) {
    *reception_time = last_synced_hf;
  }
  return global_time;
}
/*---------------------------------------------------------------------------*/
/* based on the LF timer! */
uint64_t
elwb_get_timestamp(void)
{
  return (uint64_t)global_time * 1000000 +
         (rtimer_now_lf() - last_synced_lf) * 1000000 / RTIMER_SECOND_LF;
}
/*---------------------------------------------------------------------------*/
/**
 * @brief thread of the host node
 */
PT_THREAD(elwb_thread_host(rtimer_t *rt)) 
{  
  /* variables specific to the host (all must be static) */
  static rtimer_clock_t t_start;
  static rtimer_clock_t t_start_lf;
  static uint16_t curr_period = 0;
  static uint8_t  schedule_len;
#if ELWB_CONF_DATA_ACK
  static uint8_t  data_ack[(ELWB_CONF_MAX_DATA_SLOTS + 7) / 8] = { 0 };
#endif /* ELWB_CONF_DATA_ACK */
  
  PT_BEGIN(&elwb_pt);

  elwb_cb      = elwb_thread_host;
  schedule_len = elwb_sched_init(&schedule);
   
  while(1) {
  
  #if ELWB_CONF_T_PREPROCESS_LF
    /* make sure t_preprocess is 0 except when a new round starts! */
    if(t_preprocess) {
      if(pre_proc) {
        process_poll(pre_proc);
      }
      ELWB_LF_WAIT_UNTIL(rt->time + ELWB_CONF_T_PREPROCESS_LF);
      t_preprocess = 0; /* reset value */
    }
  #endif /* ELWB_CONF_T_PREPROCESS_LF */
      
    /* --- COMMUNICATION ROUND STARTS --- */
        
    t_start    = rtimer_now_hf();
    t_start_lf = rt->time;
    rt->time   = t_start;

    /* --- SEND SCHEDULE --- */
    ELWB_SEND_SCHED();
   
    if(ELWB_SCHED_IS_FIRST(&schedule)) {
      /* sync point */
      global_time    = schedule.time;
      last_synced_hf = t_start;
      last_synced_lf = t_start_lf;
      /* collect some stats */
      stats.glossy_snr     = glossy_get_rssi();   /* use RSSI instead of SNR */
      stats.relay_cnt      = glossy_get_relay_cnt_first_rx();
      stats.glossy_t_to_rx = glossy_get_t_to_first_rx() * 100 / 325;
      stats.glossy_t_flood = glossy_get_flood_duration() * 100 / 325;
      stats.glossy_n_rx    = glossy_get_n_rx();
      stats.glossy_n_tx    = glossy_get_n_tx();
    }
    t_slot_ofs = (ELWB_CONF_T_SCHED + ELWB_CONF_T_GAP);
    
  #if ELWB_CONF_USE_XMEM
    /* put the external memory back into active mode (takes ~500us) */
    xmem_wakeup();
  #endif /* ELWB_CONF_USE_XMEM */

    /* --- DATA SLOTS --- */
    
    if(ELWB_SCHED_HAS_SLOTS(&schedule)) {

    #if ELWB_CONF_SCHED_COMPRESS
      elwb_sched_uncompress((uint8_t*)schedule.slot, 
                            ELWB_SCHED_N_SLOTS(&schedule));
    #endif /* ELWB_CONF_SCHED_COMPRESS */

      if(ELWB_SCHED_HAS_DATA_SLOTS(&schedule)) {
        /* this is a data round */
        t_slot     = ELWB_CONF_T_DATA;
        /* calculate the load (moving average over 10 rounds) */
        stats.load = (stats.load * 9 + 
                      ELWB_SCHED_N_SLOTS(&schedule) * 100 /
                      ELWB_CONF_MAX_DATA_SLOTS) / 10;
      } else {
        t_slot = ELWB_CONF_T_CONT;
      }
      static uint16_t i;
      for(i = 0; i < ELWB_SCHED_N_SLOTS(&schedule); i++) {
        /* is this our slot? Note: slots assigned to node ID 0 always belong 
         * to the host */
        if(schedule.slot[i] == 0 || schedule.slot[i] == node_id) {
          /* send a data packet (if there is any) */
          elwb_out_buffer_get((uint8_t*)glossy_payload, &payload_len);
          if(payload_len) {
            /* wait until the data slot starts */
            ELWB_WAIT_UNTIL(t_start + t_slot_ofs);  
            ELWB_SEND_PACKET();
            DEBUG_PRINT_VERBOSE("data packet sent (%ub)", payload_len);
            stats.pkt_snd++;
          }
        } else {
          if(ELWB_SCHED_HAS_DATA_SLOTS(&schedule)) {
            payload_len = GLOSSY_UNKNOWN_PAYLOAD_LEN;
          } else {
            /* it's a request round */
            payload_len = ELWB_REQ_PKT_LEN;
          }
          /* wait until the data slot starts */
          ELWB_WAIT_UNTIL(t_start + t_slot_ofs - ELWB_CONF_T_GUARD);
          ELWB_RCV_PACKET();  /* receive a data packet */
          payload_len = glossy_get_payload_len();
          if(ELWB_DATA_RCVD) {
            if(!ELWB_SCHED_HAS_DATA_SLOTS(&schedule)) {
              elwb_sched_process_req(schedule.slot[i], glossy_payload[0]);
            } else {
              DEBUG_PRINT_VERBOSE("data received from node %u (%ub)", 
                                  schedule.slot[i], payload_len);
              uint16_t res;
  #if ELWB_CONF_WRITE_TO_BOLT
              res = bolt_write((uint8_t*)glossy_payload, payload_len);
  #else /* ELWB_CONF_WRITE_TO_BOLT */
              res = elwb_in_buffer_put((uint8_t*)glossy_payload, payload_len);
  #endif /* ELWB_CONF_WRITE_TO_BOLT */
              if(res) {
                stats.pkt_fwd++;
  #if ELWB_CONF_DATA_ACK
                /* set the corresponding bit in the data ack packet */
                data_ack[i >> 3] |= (1 << (i & 0x07));
  #endif /* ELWB_CONF_DATA_ACK */
              }
              /* update statistics */
              stats.pkt_rcv++;
            }
          } else {
            DEBUG_PRINT_VERBOSE("no data received from node %u", 
                                schedule.slot[i]);
          }
        }
        t_slot_ofs += (t_slot + ELWB_CONF_T_GAP);
      }
    }
    
#if ELWB_CONF_DATA_ACK
    /* --- D-ACK SLOT --- */
    
    if(ELWB_SCHED_HAS_DATA_SLOTS(&schedule) &&
       !ELWB_SCHED_HAS_CONT_SLOT(&schedule)) {
      /* acknowledge each received packet of the last round */
      payload_len = (ELWB_SCHED_N_SLOTS(&schedule) + 7) / 8;
      if(payload_len) {
        memcpy((uint8_t*)glossy_payload, data_ack, payload_len);
        t_slot = ELWB_CONF_T_DACK;
        ELWB_WAIT_UNTIL(t_start + t_slot_ofs);
        ELWB_SEND_PACKET();
        DEBUG_PRINT_VERBOSE("D-ACK sent (%u bytes)", payload_len);
        t_slot_ofs += (ELWB_CONF_T_DACK + ELWB_CONF_T_GAP);
      }
      memset(data_ack, 0, (ELWB_CONF_MAX_DATA_SLOTS + 7) / 8);
    }
#endif /* ELWB_CONF_DATA_ACK */
    
    /* --- CONTENTION SLOT --- */
    
    if(ELWB_SCHED_HAS_CONT_SLOT(&schedule)) {
      t_slot = ELWB_CONF_T_CONT;
      payload_len = ELWB_REQ_PKT_LEN;
      glossy_payload[0] = 0;
      /* wait until the slot starts, then receive the packet */
      ELWB_WAIT_UNTIL(t_start + t_slot_ofs - ELWB_CONF_T_GUARD);
      ELWB_RCV_PACKET();
      if(ELWB_DATA_RCVD && glossy_payload[0] != 0) {
        /* process the request only if there is a valid node ID */
        elwb_sched_process_req(glossy_payload[0], 0);
      }
      if(glossy_get_n_rx_started()) {
        /* set the period to 0 to notify the scheduler that at 
         * least one nodes has data to send */
        schedule.period = 0;
        
        /* compute 2nd schedule */
        elwb_sched_compute(&schedule, 0);  /* do not allocate slots for host */
        glossy_payload[0] = schedule.period;
      } else {
        /* else: no update to schedule needed; set period to 0 to indicate to 
         * source nodes that there is no change in period (and no request round
         * following) */
        glossy_payload[0] = 0;
      }
      t_slot_ofs += ELWB_CONF_T_CONT + ELWB_CONF_T_GAP;
  
      /* --- SEND 2ND SCHEDULE --- */
      
      /* (just a 2-byte packet to indicate a change in the round period) */    
      /* send the 2nd schedule only if there was a contention slot */
      payload_len = 2;
      ELWB_WAIT_UNTIL(t_start + t_slot_ofs);
      ELWB_SEND_PACKET();    /* send as normal packet! saves energy */
    }
    
    /* --- COMMUNICATION ROUND ENDS --- */
    /* time for other computations */
    
    /* poll the other processes to allow them to run after the eLWB task was 
     * suspended (note: the polled processes will be executed in the inverse
     * order they were started/created) */
    if(ELWB_SCHED_IS_STATE_IDLE(&schedule)) {
      /* print out some stats */
      DEBUG_PRINT_INFO("%lu T=%us n=%u rcv=%u fwd=%u snd=%u per=%d "
                       "rssi=%ddBm", 
                       global_time,
                       elwb_sched_get_period(),
                       ELWB_SCHED_N_SLOTS(&schedule),
                       stats.pkt_rcv,
                       stats.pkt_fwd,
                       stats.pkt_snd,
                       glossy_get_per(),
                       stats.glossy_snr);
    
      if(post_proc) {
        process_poll(post_proc);
      }
    #if ELWB_CONF_T_PREPROCESS_LF
      t_preprocess = ELWB_CONF_T_PREPROCESS_LF;
    #endif /* ELWB_CONF_T_PREPROCESS_LF */
    }
    
    /* --- COMPUTE NEW SCHEDULE (for the next round) --- */
    curr_period  = schedule.period; /* required to schedule the next wake-up */
    schedule_len = elwb_sched_compute(&schedule, elwb_get_send_buffer_state());
    
    /* suspend this task and wait for the next round */
    ELWB_LF_WAIT_UNTIL(t_start_lf + curr_period * RTIMER_SECOND_LF / 
                       ELWB_PERIOD_SCALE - t_preprocess);
  }
  
  PT_END(&elwb_pt);
}
/*---------------------------------------------------------------------------*/
/**
 * @brief declaration of the protothread (source node)
 */
PT_THREAD(elwb_thread_src(rtimer_t *rt)) 
{  
  /* variables specific to the source node (all must be static) */
  static rtimer_clock_t t_ref;
  static rtimer_clock_t t_ref_lf;
  static elwb_syncstate_t sync_state;
  static uint8_t  node_registered;
  static uint8_t  rf_channel;
  static uint16_t period_idle;        /* last base period */
#if ELWB_CONF_DATA_ACK
  static uint8_t first_slot = 0xff,
                 num_slots  = 0;
#endif /* ELWB_CONF_DATA_ACK */
  
  PT_BEGIN(&elwb_pt);
  
  sync_state      = BOOTSTRAP;
  node_registered = 0;
  rf_channel      = RF_CONF_TX_CH;
  elwb_cb         = elwb_thread_src;
  
  while(1) {
        
  #if ELWB_CONF_T_PREPROCESS_LF
    if(t_preprocess) {
      if(pre_proc) {
        process_poll(pre_proc);
      }
      ELWB_LF_WAIT_UNTIL(rt->time + ELWB_CONF_T_PREPROCESS_LF);
      t_preprocess = 0;
    }
  #endif /* ELWB_CONF_T_PREPROCESS_LF */
    
    /* --- COMMUNICATION ROUND STARTS --- */
    
    rt->time = rtimer_now_hf();            /* overwrite LF with HF timestamp */
    
  #if ELWB_CONF_USE_XMEM
    /* xmem_wakeup() -> instead of waiting 500us, just pull CTRL line low */
    PIN_CLR(FRAM_CONF_CTRL_PIN);
    PIN_SET(FRAM_CONF_CTRL_PIN);
  #endif /* ELWB_CONF_USE_XMEM */
    
    /* --- RECEIVE SCHEDULE --- */
    
    payload_len = GLOSSY_UNKNOWN_PAYLOAD_LEN;
    if(sync_state == BOOTSTRAP) {
      while (1) {
        schedule.n_slots = 0;   /* reset */
        stats.bootstrap_cnt++;
        static rtimer_clock_t bootstrap_started;
        bootstrap_started = rtimer_now_hf();
        DEBUG_PRINT_MSG_NOW(elwb_syncstate_to_string[BOOTSTRAP]);
        /* synchronize first! wait for the first schedule... */
        do {
  #if WATCHDOG_CONF_ON && !WATCHDOG_CONF_RESET_ON_TA1IFG
          watchdog_reset();
  #endif /* WATCHDOG_CONF_ON */
          ELWB_RCV_SCHED();
        } while(!glossy_is_t_ref_updated() && ((rtimer_now_hf() -
                bootstrap_started) < ELWB_CONF_T_SILENT));
        if(glossy_is_t_ref_updated()) {
          break;  /* schedule received, exit bootstrap state */
        }
        /* go to sleep for ELWB_CONF_T_DEEPSLEEP_LF ticks */
        stats.sleep_cnt++;
        DEBUG_PRINT_MSG_NOW("TIMEOUT");
        ELWB_BEFORE_DEEPSLEEP();
        ELWB_LF_WAIT_UNTIL(rtimer_now_lf() + ELWB_CONF_T_DEEPSLEEP_LF);
        ELWB_AFTER_DEEPSLEEP();
        rt->time = rtimer_now_hf();
        /* switch frequency and try again */
        if(rf_channel == ELWB_CONF_RF_CH_PRIMARY) {
          rf_channel = ELWB_CONF_RF_CH_SECONDARY;
        } else {
          rf_channel = ELWB_CONF_RF_CH_PRIMARY;
        }
        rf1a_set_channel(rf_channel);
        DEBUG_PRINT_MSG_NOW("RF channel switched to %u", rf_channel);
      }
    } else {
      ELWB_RCV_SCHED();
    }
    
    /* schedule received? */
    if(glossy_is_t_ref_updated()) {
  #if ELWB_CONF_SCHED_CRC
      /* check the CRC */
      payload_len = glossy_get_payload_len();
      uint16_t calc_crc = crc16((uint8_t*)&schedule, payload_len - 2, 0);
      uint16_t pkt_crc;
      memcpy(&pkt_crc, (uint8_t*)&schedule + payload_len - 2, 2);
      if(calc_crc != pkt_crc) {
        /* not supposed to happend, all we can do now is go back to bootstrap
         * since the previous (valid schedule has been overwritten with a
         * corrupted one */
        EVENT_ERROR(EVENT_CC430_RADIO_ERROR, 0);
        DEBUG_PRINT_MSG_NOW("ERROR invalid eLWB schedule CRC!");
        sync_state = BOOTSTRAP;
        continue;
      }
  #endif /* ELWB_CONF_SCHED_CRC */
      /* update the sync state machine */
      sync_state = next_state[EVT_SCHED_RCVD][sync_state];
      /* HF timestamp of 1st RX, subtract const offset to align src and host */
      t_ref = glossy_get_t_ref() - ELWB_T_REF_OFS;
      /* calculate t_ref_lf by subtracting the elapsed time since t_ref: */
      rtimer_clock_t hf_now;
      rtimer_now(&hf_now, &t_ref_lf);
      t_ref_lf -= (uint32_t)(hf_now - t_ref) / (uint32_t)RTIMER_HF_LF_RATIO;
      if(ELWB_SCHED_IS_FIRST(&schedule)) {
        /* only update the timestamp during the idle period */
        period_idle = schedule.period;
        global_time = schedule.time;
        last_synced_lf = t_ref_lf;
        last_synced_hf = t_ref;
        
        /* collect some stats of the schedule flood */
        stats.relay_cnt      = glossy_get_relay_cnt_first_rx();
        stats.glossy_snr     = glossy_get_snr();
        stats.glossy_t_to_rx = glossy_get_t_to_first_rx() * 100 / 325;
        stats.glossy_t_flood = glossy_get_flood_duration() * 100 / 325;
        stats.glossy_n_rx    = glossy_get_n_rx();
        stats.glossy_n_tx    = glossy_get_n_tx();
        /* do some basic drift estimation: measured elapsed time minus
         * effective elapsed time (given by host) */
        uint16_t elapsed_time = (uint16_t)(schedule.time - global_time);
        int16_t drift = (int16_t)((int32_t)(t_ref_lf - last_synced_lf) - 
                                  (int32_t)(elapsed_time * RTIMER_SECOND_LF));
        /* now scale the difference in LF ticks to ppm */
        drift = drift * (int16_t)(1000000 / RTIMER_SECOND_LF) / elapsed_time;
        if(drift < 100 && drift > -100) {
          stats.drift = (stats.drift + drift) / 2;
        }
      }
    } else {
      /* update the sync state machine */
      sync_state = next_state[EVT_SCHED_MISSED][sync_state];
      if(sync_state == BOOTSTRAP) {
        t_preprocess = 0;
        continue;
      }
      stats.unsynced_cnt++;
      DEBUG_PRINT_WARNING("schedule missed");
      /* we can only estimate t_ref and t_ref_lf */
      if(!ELWB_SCHED_IS_STATE_IDLE(&schedule)) {
        /* missed schedule was during a contention/data round -> reset t_ref */
        t_ref_lf = last_synced_lf;
        /* mark as 'idle state' such that other processes can run */
        ELWB_SCHED_SET_STATE_IDLE(&schedule);
      } else {
        /* missed schedule is at beginning of a round: add last period */
        t_ref_lf += schedule.period * RTIMER_SECOND_LF / ELWB_PERIOD_SCALE;
      }
      schedule.period = period_idle;  /* reset period to idle period */
    }
    
    /* permission to participate in this round? */
    if(sync_state == SYNCED) {
      
    #if ELWB_CONF_SCHED_COMPRESS
      /* uncompress the schedule */
      elwb_sched_uncompress((uint8_t*)schedule.slot, 
                            ELWB_SCHED_N_SLOTS(&schedule));
    #endif /* ELWB_CONF_SCHED_COMPRESS */
      
      /* sanity check (mustn't exceed the compile-time fixed max. # slots!) */
      if(ELWB_SCHED_N_SLOTS(&schedule) > ELWB_SCHED_MAX_SLOTS) {
        DEBUG_PRINT_ERROR("n_slots exceeds limit!");
        EVENT_ERROR(EVENT_CC430_MEM_OVERFLOW, 1);
        ELWB_SCHED_CLR_SLOTS(&schedule);
        schedule.n_slots += ELWB_SCHED_MAX_SLOTS;
      }
      
      static uint16_t i;
      t_slot_ofs = (ELWB_CONF_T_SCHED + ELWB_CONF_T_GAP); 
    
      /* --- DATA SLOTS --- */
      
      if(ELWB_SCHED_HAS_SLOTS(&schedule)) {
        /* set the slot duration */
        if(ELWB_SCHED_HAS_DATA_SLOTS(&schedule)) {
          /* this is a data round */
          t_slot = ELWB_CONF_T_DATA;
        } else {
          /* it's a request round */
          t_slot = ELWB_CONF_T_CONT;
          node_registered = 0;
        }
        for(i = 0; i < ELWB_SCHED_N_SLOTS(&schedule); i++) {
          if(schedule.slot[i] == node_id) {
            node_registered = 1;
            /* this is our data slot, send a data packet (if there is any) */
            if(!FIFO_EMPTY(&tx_queue)) {
              if(ELWB_SCHED_HAS_DATA_SLOTS(&schedule)) {
  #if ELWB_CONF_DATA_ACK
                if(first_slot == 0xff) {
                  first_slot = i;
                }
                num_slots++;
  #endif /* ELWB_CONF_DATA_ACK */
                elwb_out_buffer_get((uint8_t*)glossy_payload, &payload_len);
                stats.pkt_snd++;
              } else {
                payload_len = ELWB_REQ_PKT_LEN;
                /* request as many data slots as there are pkts in the queue*/
                glossy_payload[0] = elwb_get_send_buffer_state();
                stats.load = (stats.load * 9 + 
                              (uint16_t)elwb_get_send_buffer_state() * 100 / 
                              ELWB_CONF_OUT_BUFFER_SIZE + 9) / 10;
              }
              ELWB_WAIT_UNTIL(t_ref + t_slot_ofs);
              ELWB_SEND_PACKET();
              DEBUG_PRINT_VERBOSE("packet sent (%ub)", payload_len);
            } else {
              DEBUG_PRINT_VERBOSE("no message to send (data slot ignored)");
            }
          } else
          {
            payload_len = GLOSSY_UNKNOWN_PAYLOAD_LEN;
            if(!ELWB_SCHED_HAS_DATA_SLOTS(&schedule)) {
              /* the payload length is known in the request round */
              payload_len = ELWB_REQ_PKT_LEN;
            }
            /* receive a data packet */
            ELWB_WAIT_UNTIL(t_ref + t_slot_ofs - ELWB_CONF_T_GUARD);
            ELWB_RCV_PACKET();
            payload_len = glossy_get_payload_len();
            if(ELWB_SCHED_HAS_DATA_SLOTS(&schedule) && ELWB_DATA_RCVD) {
             /* forward the packet to the application task if the initiator was
              * the host or sink or if the custom forward filter is 'true' */
              if(ELWB_CONF_SRC_PKT_FILTER(glossy_payload)) {
    #if ELWB_CONF_WRITE_TO_BOLT
                bolt_write((uint8_t*)glossy_payload, payload_len);
    #else /* ELWB_CONF_WRITE_TO_BOLT */
                elwb_in_buffer_put((uint8_t*)glossy_payload, payload_len);
    #endif /* ELWB_CONF_WRITE_TO_BOLT */
                stats.pkt_fwd++;
              }
              stats.pkt_rcv++;
            }
          }
          t_slot_ofs += (t_slot + ELWB_CONF_T_GAP);
        }
      }
      
  #if ELWB_CONF_DATA_ACK
      /* --- D-ACK SLOT --- */
      
      if(!(cfg.dbg_flags & 0x02) && 
         ELWB_SCHED_HAS_DATA_SLOTS(&schedule) &&
         !ELWB_SCHED_HAS_CONT_SLOT(&schedule)) {
        t_slot = ELWB_CONF_T_DACK;
        payload_len = GLOSSY_UNKNOWN_PAYLOAD_LEN;
        ELWB_WAIT_UNTIL(t_ref + t_slot_ofs - ELWB_CONF_T_GUARD);
        ELWB_RCV_PACKET();                 /* receive data ack */
        payload_len = glossy_get_payload_len();
        /* only look into the D-ACK packet if we actually sent some data in the
         * previous round */
        if(first_slot != 0xff) {
          if(ELWB_DATA_RCVD) {
            DEBUG_PRINT_VERBOSE("D-ACK rcvd (%ub)", payload_len);
            uint8_t* data_acks = (uint8_t*)glossy_payload;
            for(i = 0; i < num_slots; i++) {
              /* bit not set? => not acknowledged */
              if(!(data_acks[(first_slot + i) >> 3] &
                  (1 << ((first_slot + i) & 0x07)))) {
                /* resend the packet (re-insert it into the output FIFO) */
                uint32_t addr = fifo_elem_addr_rel(&tx_queue,
                                                   (int16_t)i - num_slots);
                elwb_requeue_pkt(addr);
              } else {
                stats.pkt_ack++;
              }
            }
          } else {
            /* requeue all */
            fifo_restore(&tx_queue, num_slots);
            DEBUG_PRINT_WARNING("D-ACK pkt missed, %u pkt requeued", num_slots);
          }
          first_slot  = 0xff;
          num_slots   = 0;
        }
        t_slot_ofs += (ELWB_CONF_T_DACK + ELWB_CONF_T_GAP);
      }
  #endif /* ELWB_CONF_DATA_ACK */
      
      /* --- CONTENTION SLOT --- */
      
      /* is there a contention slot in this round? */
      if(ELWB_SCHED_HAS_CONT_SLOT(&schedule)) {
        t_slot = ELWB_CONF_T_CONT;
        if(!FIFO_EMPTY(&tx_queue)) {
          /* if there is data in the output buffer, then request a slot */
          /* a slot request packet always looks the same */
          payload_len = ELWB_REQ_PKT_LEN;
          /* include the node ID in case this is the first request */
          glossy_payload[0] = 0;
          if(!node_registered) {
            glossy_payload[0] = node_id;
            DEBUG_PRINT_INFO("transmitting node ID");
          }
          /* wait until the contention slot starts */
          ELWB_WAIT_UNTIL(t_ref + t_slot_ofs);
          ELWB_SEND_PACKET();
        } else {
          /* no request pending -> just receive / relay packets */
          payload_len = ELWB_REQ_PKT_LEN;
          ELWB_WAIT_UNTIL(t_ref + t_slot_ofs - ELWB_CONF_T_GUARD);
          ELWB_RCV_PACKET();
        }
        t_slot_ofs += ELWB_CONF_T_CONT + ELWB_CONF_T_GAP;
        
        /* --- RECEIVE 2ND SCHEDULE --- */
      
        /* only rcv the 2nd schedule if there was a contention slot */
        payload_len = 2;  /* we expect exactly 2 bytes */
        ELWB_WAIT_UNTIL(t_ref + t_slot_ofs - ELWB_CONF_T_GUARD);
        ELWB_RCV_PACKET();
        if(ELWB_DATA_RCVD) {
          uint16_t new_period = glossy_payload[0];
          if(new_period != 0) {                      /* zero means no change */
            schedule.period = new_period;          /* extract updated period */
            schedule.n_slots = 0;                                  /* clear! */
          } /* else: all good, no need to change anything */
        } else {
          DEBUG_PRINT_WARNING("2nd schedule missed");
        }
      }
    }
    
    /* --- COMMUNICATION ROUND ENDS --- */    
    /* time for other computations */
    
    if(ELWB_SCHED_IS_STATE_IDLE(&schedule)) {
      /* print out some stats (note: takes ~2ms to compose this string!) */
      DEBUG_PRINT_INFO("%s %lu T=%u n=%u rcv=%u fwd=%u snd=%u ack=%u h=%u b=%u"
                       " u=%u per=%d snr=%d dr=%d", 
                       elwb_syncstate_to_string[sync_state],
                       schedule.time, 
                       schedule.period * (1000 / ELWB_PERIOD_SCALE), 
                       ELWB_SCHED_N_SLOTS(&schedule),
                       stats.pkt_rcv,
                       stats.pkt_fwd,
                       stats.pkt_snd,
                       stats.pkt_ack,
                       stats.relay_cnt, 
                       stats.bootstrap_cnt, 
                       stats.unsynced_cnt,
                       glossy_get_per(),
                       stats.glossy_snr,
                       stats.drift);

      /* poll the post process */
      if(post_proc) {
        process_poll(post_proc);
      }
    #if ELWB_CONF_T_PREPROCESS_LF
      t_preprocess = ELWB_CONF_T_PREPROCESS_LF;
    #endif /* ELWB_CONF_T_PREPROCESS_LF */
    }
    /* erase the schedule (slot allocations only) */
    memset(&schedule.slot, 0, sizeof(schedule.slot));
    ELWB_SCHED_CLR_SLOTS(&schedule);
    
    /* schedule the wakeup for the next round */
    ELWB_LF_WAIT_UNTIL(t_ref_lf + (schedule.period * RTIMER_SECOND_LF) /
                      ELWB_PERIOD_SCALE - ELWB_CONF_T_GUARD_LF - t_preprocess);
  }

  PT_END(&elwb_pt);
}
/*---------------------------------------------------------------------------*/
void
elwb_start(struct process *pre_elwb_proc, struct process *post_elwb_proc)
{
  pre_proc  = pre_elwb_proc;
  post_proc = (struct process*)post_elwb_proc;
  
  uart_enable(1);
  printf("Protothread 'eLWB' started\r\n");
  printf(" pkt_len=%u slots=%u n_tx_d=%u n_tx_s=%u hops=%u\r\n", 
         ELWB_CONF_MAX_PKT_LEN,
         ELWB_CONF_MAX_DATA_SLOTS,
         ELWB_CONF_N_TX_DATA, 
         ELWB_CONF_N_TX_SCHED,
         ELWB_CONF_N_HOPS);
  /* ceil the values (therefore + RTIMER_SECOND_HF / 1000 - 1) */
  printf(" slots [ms]: sched=%u data=%u cont=%u\r\n",
   (uint16_t)RTIMER_HF_TO_MS(ELWB_CONF_T_SCHED + (RTIMER_SECOND_HF / 1000 - 1)),
   (uint16_t)RTIMER_HF_TO_MS(ELWB_CONF_T_DATA + (RTIMER_SECOND_HF / 1000 - 1)),
   (uint16_t)RTIMER_HF_TO_MS(ELWB_CONF_T_CONT + (RTIMER_SECOND_HF / 1000 - 1)));
  
#if !ELWB_CONF_USE_XMEM
  /* pass the start addresses of the memory blocks holding the queues */
  fifo_init(&rx_queue, (uint16_t)rx_queue_mem);
  fifo_init(&tx_queue, (uint16_t)tx_queue_mem); 
#else  /* ELWB_CONF_USE_XMEM */
  /* allocate memory for the message buffering (in ext. memory) */
  fifo_init(&rx_queue, xmem_alloc(ELWB_CONF_IN_BUFFER_SIZE * 
                                  sizeof(elwb_queue_elem_t)));
  fifo_init(&tx_queue, xmem_alloc(ELWB_CONF_OUT_BUFFER_SIZE * 
                                  sizeof(elwb_queue_elem_t)));
#endif /* ELWB_CONF_USE_XMEM */

#ifdef ELWB_CONF_TASK_ACT_PIN
  PIN_CFG_OUT(ELWB_CONF_TASK_ACT_PIN);
  PIN_CLR(ELWB_CONF_TASK_ACT_PIN);
#endif /* ELWB_CONF_TASK_ACT_PIN */
  
  memset(&schedule, 0, sizeof(schedule));
  
  PT_INIT(&elwb_pt); /* initialize the protothread */

  /* start in 50ms (use LF timer), gives other task enough time to init */
  rtimer_clock_t t_wakeup = rtimer_now_lf() + RTIMER_SECOND_LF / 20;
#if IS_HOST
  /* update the global time and wait for the next full second */
  schedule.time = ((t_wakeup + RTIMER_SECOND_LF) / RTIMER_SECOND_LF);
  elwb_sched_set_time(schedule.time);
  t_wakeup = (rtimer_clock_t)schedule.time * RTIMER_SECOND_LF;
  rtimer_schedule(ELWB_CONF_LF_RTIMER_ID, t_wakeup, 0, elwb_thread_host);
#else /* IS_HOST */
  rtimer_schedule(ELWB_CONF_LF_RTIMER_ID, t_wakeup, 0, elwb_thread_src);
#endif /* IS_HOST */
}
/*---------------------------------------------------------------------------*/

