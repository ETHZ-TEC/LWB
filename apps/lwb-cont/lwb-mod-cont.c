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
 *
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
 *          Marco Zimmerling
 *          Federico Ferrari
 */

/**
 * @file
 *
 * This is a simplified version of the LWB, it's non-functional in terms of 
 * stream allocation and data transmission. 
 * The only purpose is to assess the effectiveness of the contention slot.
 * 
 * To minimize code size, all unnecessary parts were removed.
 */
 
#include "contiki.h"

/*---------------------------------------------------------------------------*/
/* length of the 'wakeup' packet during a contention slot (if the received
 * packet has a different size, it will be dropped (no retransmission) */
#define LWB_CONF_WAKEUP_PKT_LEN     1   /* set to 0 to disable */

/* max. random delay before sending the stream request, in clock ticks */
#ifndef LWB_CONF_RANDOM_DELAY_MAX
#define LWB_CONF_RANDOM_DELAY_MAX   0
#endif /* LWB_CONF_RANDOM_DELAY_MAX */ 

/* to perform an 'offset' test; this node will insert an artificial offset
 * before sending the stream request */
#ifndef LWB_CONF_OFFSET_NODE_ID
#define LWB_CONF_OFFSET_NODE_ID     0
#endif /* LWB_CONF_OFFSET_NODE_ID */
#define LWB_CONF_OFFSET_MAX         400
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
typedef struct {
  uint8_t raw_data[LWB_CONF_MAX_PACKET_LEN];
} glossy_payload_t;
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
  glossy_start(GLOSSY_UNKNOWN_INITIATOR, (uint8_t *)&schedule, \
               GLOSSY_UNKNOWN_PAYLOAD_LEN, \
               LWB_CONF_TX_CNT_SCHED, GLOSSY_WITH_SYNC, GLOSSY_WITH_RF_CAL);\
  LWB_WAIT_UNTIL(rt->time + LWB_CONF_T_SCHED + t_guard);\
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
               LWB_CONF_WAKEUP_PKT_LEN, \
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
#define LWB_UPDATE_SYNC_STATE \
{\
  /* get the new state based on the event */\
  sync_state = next_state[GET_EVENT][sync_state];\
  t_guard = guard_time[sync_state];         /* adjust the guard time */\
}
/*---------------------------------------------------------------------------*/
static struct pt        lwb_pt;
static struct process*  pre_proc;
static struct process*  post_proc;
static lwb_sync_state_t sync_state;
/*---------------------------------------------------------------------------*/
/**
 * @brief thread of the host node
 */
PT_THREAD(lwb_thread_host(rtimer_t *rt)) 
{  
  /* all variables must be static */
  static lwb_schedule_t schedule;
  static rtimer_clock_t t_start; 
  static glossy_payload_t glossy_payload;                   /* packet buffer */
  /* constant guard time for the host */
  static const uint32_t t_guard = LWB_CONF_T_GUARD; 
  static uint8_t schedule_len;
  static const void* callback_func = lwb_thread_host;
  static uint16_t req_missed = 0,
                  req_rcvd = 0;

  /* note: all statements above PT_BEGIN() will be executed each time the 
   * protothread is scheduled */
  
  PT_BEGIN(&lwb_pt);   /* declare variables before this statement! */
    
  /* schedule won't change, constant period of 1s + always a contention slot */
  memset(&schedule, 0, sizeof(schedule));
  schedule_len = LWB_SCHED_PKT_HEADER_LEN;
  schedule.period = 1;
  schedule.time   = 0;
  LWB_SCHED_SET_CONT_SLOT(&schedule);
    
  while(1) {
    
    /* set the start time of the round to the expiration time of the last 
     * scheduled timeout */
    t_start = rt->time;
        
    LWB_SEND_SCHED();
                
    /* --- CONTENTION SLOT --- */
    
    if(LWB_SCHED_HAS_CONT_SLOT(&schedule)) {
      memset(&glossy_payload.raw_data, 0, 10);
      /* wait until the slot starts, then receive the packet */
      LWB_WAIT_UNTIL(t_start + LWB_CONF_T_SCHED + LWB_CONF_T_GAP - t_guard);
      LWB_RCV_SRQ();
    }
    if(glossy_get_n_rx()) { 
      req_rcvd++;   /* at least one packet with CRC & header ok received */
    }
    if(glossy_get_n_rx_started() == 0) {
      req_missed++;   /* nothing received in the last flood */
    }
    
    /* print out some stats */
    DEBUG_PRINT_INFO("t=%lu rx_started=%u crc_ok=%u rx_fail=%u hdr_fail=%u "
                     "rcv=%u miss=%u", 
                     schedule.time,
                     glossy_get_n_rx_started(),
                     glossy_get_n_crc_ok(),
                     glossy_get_n_rx_fail(), 
                     glossy_get_n_header_fail(),
                     req_rcvd,
                     req_missed);
    
    /* poll the other processes to allow them to run after the LWB task was 
     * suspended (note: the polled processes will be executed in the inverse
     * order they were started/created) */
    debug_print_poll();
    if(post_proc) {
      /* will be executed before the debug print task */
      process_poll(post_proc);    
    }
    
    /* increase the time */
    schedule.time++;
    
    /* suspend this task and wait for the next round */
    LWB_WAIT_UNTIL(t_start + schedule.period * RTIMER_SECOND_HF /
                   LWB_CONF_TIME_SCALE);
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
  static glossy_payload_t glossy_payload;                   /* packet buffer */
  static uint32_t t_guard;                  /* 32-bit is enough for t_guard! */
  static uint8_t payload_len;
  static int8_t  glossy_snr = 0;
  static uint16_t requests_sent = 0;
  static uint16_t unsynced_cnt = 0;
  static int16_t t_rand = 0;   /* random offset */
  static const void* callback_func = lwb_thread_src;
  
  PT_BEGIN(&lwb_pt);   /* declare variables before this statement! */
  
  memset(&schedule, 0, sizeof(schedule)); 
  
  /* initialization specific to the source node */
  lwb_stream_init();
  sync_state        = BOOTSTRAP;
  
#if LWB_CONF_OFFSET_NODE_ID
  if(node_id == LWB_CONF_OFFSET_NODE_ID) {
    t_rand = -LWB_CONF_OFFSET_MAX;
  }
#endif /* LWB_CONF_OFFSET_NODE_ID */
  
  while(1) {
    
    if(sync_state == BOOTSTRAP) {
      DEBUG_PRINT_MSG_NOW("BOOTSTRAP ");
      /* synchronize first! wait for the first schedule... */
      do {
        LWB_RCV_SCHED();
      } while(!glossy_is_t_ref_updated());
      /* schedule received! */
      putchar('\r');
      putchar('\n');
    } else {        
      LWB_RCV_SCHED();       
    }
    glossy_snr = glossy_get_snr();
                   
    /* update the sync state machine (compute new sync state and update 
     * t_guard) */
    LWB_UPDATE_SYNC_STATE;  
    if(BOOTSTRAP == sync_state) {
      /* something went wrong */
      continue;
    } 
    if(glossy_is_t_ref_updated()) {
      /* HF timestamp of first RX; subtract a constant offset */
      t_ref = glossy_get_t_ref() - LWB_CONF_T_REF_OFS;           
    } else {
      DEBUG_PRINT_WARNING("schedule missed");
      /* we can only estimate t_ref and t_ref_lf */
      t_ref += schedule.period * (RTIMER_SECOND_HF) /
               LWB_CONF_TIME_SCALE;
      /* don't update the schedule.time here! */
    }
        
    /* --- CONTENTION SLOT --- */
      
    /* is there a contention slot in this round? */
    if(LWB_SCHED_HAS_CONT_SLOT(&schedule)) {
#if LWB_CONF_RANDOM_DELAY_MAX
      t_rand = random_rand() % LWB_CONF_RANDOM_DELAY_MAX;
#endif /* LWB_CONF_RANDOM_DELAY_MAX */
      
#if LWB_CONF_OFFSET_NODE_ID
      /* add controlled offset */
      if(node_id == LWB_CONF_OFFSET_NODE_ID) {        
        if(t_rand > LWB_CONF_OFFSET_MAX) {
          DEBUG_PRINT_INFO("STOP: max reached");
        } else {
          t_rand++;
        }
      }
#endif /* LWB_CONF_OFFSET_NODE_ID */
      
      /* send a 'wakeup' stream request */
      payload_len = 1;
      glossy_payload.raw_data[0] = 1;   /* repeat the length of the packet */
      LWB_WAIT_UNTIL(t_ref + LWB_CONF_T_SCHED + LWB_CONF_T_GAP + t_rand);
      LWB_SEND_SRQ();  
      requests_sent++;
    }

    if(sync_state == UNSYNCED) {
      unsynced_cnt++;
    }
    /* print out some stats (note: takes approx. 2ms to compose this string) */
    DEBUG_PRINT_INFO("%s %lu cont=%u usyn=%u snr=%ddbm", 
                     lwb_sync_state_to_string[sync_state], 
                     schedule.time,
                     requests_sent,
                     unsynced_cnt,
                     glossy_snr);

    /* poll the other processes to allow them to run after the LWB task was
     * suspended (note: the polled processes will be executed in the inverse
     * order they were started/created) */
    debug_print_poll();
    if(post_proc) {
      process_poll(post_proc);
    }
    
    LWB_WAIT_UNTIL(t_ref + 
                   schedule.period * (RTIMER_SECOND_HF) / 
                   LWB_CONF_TIME_SCALE - t_guard);
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
    
#ifdef LWB_CONF_TASK_ACT_PIN
  PIN_CFG_OUT(LWB_CONF_TASK_ACT_PIN);
  PIN_CLR(LWB_CONF_TASK_ACT_PIN);
#endif /* LWB_CONF_TASK_ACT_PIN */
        
  PT_INIT(&lwb_pt); /* initialize the protothread */
  
  if(node_id == HOST_ID) {
    /* note: must add at least some clock ticks! */
    rtimer_schedule(LWB_CONF_RTIMER_ID, 
                    rtimer_now_hf() + RTIMER_SECOND_HF / 5,
                    0, lwb_thread_host);
  } else {
    rtimer_schedule(LWB_CONF_RTIMER_ID, 
                    rtimer_now_hf() + RTIMER_SECOND_HF / 10,
                    0, lwb_thread_src);
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
void
lwb_start(void *pre_lwb_proc, void *post_lwb_proc)
{
  pre_proc = (struct process*)pre_lwb_proc;
  post_proc = (struct process*)post_lwb_proc;
  printf("Starting '%s' (contention test)\r\n", lwb_process.name);
    
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
  if((LWB_CONF_T_SCHED2_START > RTIMER_SECOND_HF / LWB_CONF_TIME_SCALE)) {
    printf("WARNING: LWB_CONF_T_SCHED2_START > 1s\r\n");
  }
  process_start(&lwb_process, NULL);
}
/*---------------------------------------------------------------------------*/
