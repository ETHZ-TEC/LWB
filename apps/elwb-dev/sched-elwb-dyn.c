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
 * @addtogroup  lwb-scheduler
 * @{
 *
 * @defgroup    sched-ae scheduler used for the acoustic emission demo app
 * @{
 *
 * @brief
 * a simple scheduler implementation for the LWB
 * 
 * The scheduler has a static base period of LWB_CONF_SCHED_PERIOD_IDLE sec. 
 * There is no IPI (inter-packet interval) required for stream requests, the
 * scheduler will assign 1 slot to each stream in the next round. If the source
 * node wants to transmit more data, it needs to request another stream, i.e.
 * for each data packet, a stream request is necessary.
 * There are no more stream acknowledgements, but the SACK slot can be used
 * to acknowledge the data reception.
 * Schedule compression is not used.
 */
 
#include "lwb.h"

#ifdef LWB_SCHED_ELWB_DYN

#define LWB_PERIOD_SCALE                100         /* also change in elwb.c */

#ifndef LWB_CONF_SCHED_PERIOD_IDLE_MS
#define LWB_CONF_SCHED_PERIOD_IDLE_MS   (LWB_CONF_SCHED_PERIOD_IDLE * 1000)
#endif /* LWB_CONF_SCHED_PERIOD_IDLE_MS */

#ifndef LWB_CONF_MAX_SLOTS_HOST
#define LWB_CONF_MAX_SLOTS_HOST         (LWB_CONF_MAX_DATA_SLOTS / 2)
#endif /* LWB_CONF_MAX_SLOTS_HOST */

/* parameter sanity check */

#if LWB_CONF_MAX_SLOTS_HOST > LWB_CONF_MAX_DATA_SLOTS
#error "LWB_CONF_MAX_SLOTS_HOST > LWB_CONF_MAX_DATA_SLOTS!"
#endif /* LWB_CONF_MAX_SLOTS_HOST */

#if !defined(LWB_CONF_STREAM_EXTRA_DATA_LEN) || \
    (LWB_CONF_STREAM_EXTRA_DATA_LEN != 0)
#error "LWB_CONF_STREAM_EXTRA_DATA_LEN not set to 0!"
#endif

#if LWB_CONF_MAX_N_STREAMS > LWB_CONF_MAX_DATA_SLOTS
#error "max. # of streams mustn't be bigger than max. # of data slots"
#endif

#define LWB_PERIOD_IDLE         (uint16_t)( \
                                 (uint32_t)LWB_CONF_SCHED_PERIOD_IDLE_MS * \
                                 LWB_PERIOD_SCALE / 1000)

/* earliest start (offset) of the request round, + 10ms slack (necessary due
 * to rounding issue) */
#define LWB_PERIOD_T_REQ_MIN    ((LWB_CONF_T_SCHED + \
                                 2 * LWB_CONF_T_CONT + 2 * LWB_CONF_T_GAP + \
                                 (RTIMER_SECOND_HF / 100)) * LWB_PERIOD_SCALE \
                                 / RTIMER_SECOND_HF)
    
/* duration of the 'request round' = start of the data round,
 * depends on the max. # nodes (= LWB_CONF_MAX_N_STREAMS), + add 10ms slack */
#define LWB_PERIOD_T_DATA       ((LWB_CONF_T_SCHED + LWB_CONF_T_GAP + \
                                  LWB_CONF_MAX_N_STREAMS * \
                                   (LWB_CONF_T_CONT + LWB_CONF_T_GAP) + \
                                  (RTIMER_SECOND_HF / 100)) / \
                                 (RTIMER_SECOND_HF / LWB_PERIOD_SCALE))

/* note: round up for the following values */
#define LWB_T_IDLE_ROUND_MS     (LWB_PERIOD_T_REQ_MIN * \
                                 (1000 / LWB_PERIOD_SCALE))
#define LWB_T_REQ_ROUND_MS      (LWB_PERIOD_T_DATA * (1000 / LWB_PERIOD_SCALE))
#define LWB_T_DATA_ROUND_MS     ((LWB_CONF_T_SCHED + LWB_CONF_T_GAP + \
                                  LWB_CONF_MAX_DATA_SLOTS * \
                                  (LWB_CONF_T_DATA + LWB_CONF_T_GAP)) * 1000 /\
                                 RTIMER_SECOND_HF)
#define LWB_T_ROUND_MAX_MS      (LWB_T_IDLE_ROUND_MS + LWB_T_REQ_ROUND_MS + \
                                 LWB_T_DATA_ROUND_MS + 10)     /* slack 10ms */
      
/* used to adjust the slot length in elwb.c */
#define SET_DATA_ROUND(s)       LWB_SCHED_SET_SACK_SLOT(s)

#define SET_STATE_IDLE(s)       LWB_SCHED_SET_DACK_SLOT(s)

/*---------------------------------------------------------------------------*/
uint16_t lwb_sched_compress(uint8_t* compressed_data, uint8_t n_slots);
/*---------------------------------------------------------------------------*/
/**
 * @brief struct to store information about active streams on the host
 */
typedef struct stream_info {
  struct stream_info *next;
  uint16_t            id;
  uint8_t             state;
  uint8_t             n_pkts;
} lwb_stream_list_t;
/*---------------------------------------------------------------------------*/
typedef enum {
  LWB_SCHED_STATE_IDLE = 0,
  LWB_SCHED_STATE_CONT_DET,
  LWB_SCHED_STATE_REQ,
  LWB_SCHED_STATE_DATA,
} lwb_sched_state_t;
/*---------------------------------------------------------------------------*/
static uint64_t          time;                                /* global time */
static uint16_t          period;            /* base (idle) period in seconds */
static uint16_t          n_streams;                      /* # active streams */
static lwb_sched_state_t sched_state;
LIST(streams_list);                    /* -> lists only work for data in RAM */
/* data structures to hold the stream info */
MEMB(streams_memb, lwb_stream_list_t, LWB_CONF_MAX_N_STREAMS);
/*---------------------------------------------------------------------------*/
void 
lwb_sched_proc_srq(const lwb_stream_req_t* req) 
{
  lwb_stream_list_t *s = 0;    
  /* check if stream already exists */
  for(s = list_head(streams_list); s != 0; s = s->next) {
    if(req->id == s->id) {
      if(!s->state) {
        n_streams++;
        s->state = 1;
      }
      s->n_pkts = req->reserved; /* # packets the node wants to send */
      return;
    }
  }
  if(n_streams >= LWB_CONF_MAX_N_STREAMS) {
    DEBUG_PRINT_WARNING("stream request from node %u dropped, max #streams "
                        "reached", req->id);
    return;
  } 
  /* does not exist: add the new stream */
  s = memb_alloc(&streams_memb);
  if(s == 0) {
    DEBUG_PRINT_ERROR("out of memory: stream request dropped");
    return;
  }
  s->id = req->id;
  s->n_pkts  = req->reserved;         /* # packets the node wants to send */
  s->state   = 1;
  /* insert the stream into the list, ordered by node id */
  lwb_stream_list_t *prev;
  for(prev = list_head(streams_list); prev != NULL; prev = prev->next) {
    if((req->id >= prev->id) && ((prev->next == NULL) || 
        (req->id < prev->next->id))) {
      break;
    }
  }
  list_insert(streams_list, prev, s);
  n_streams++;
  DEBUG_PRINT_INFO("stream of node %u added", req->id);
  /* no need to send a stream acknowledgement */
}
/*---------------------------------------------------------------------------*/
uint16_t 
lwb_sched_compute(lwb_schedule_t * const sched, 
                  const uint8_t * const unused, 
                  uint8_t reserve_slot_host) 
{ 
  static uint8_t slots_host;
  uint8_t n_slots_assigned = 0;
  
  /* increment the timestamp */
  time += sched->period;
  
  /*
   * note: the schedule is sent at the beginning of the next round,
   * i.e. it must include the next period
   */
  if(sched_state == LWB_SCHED_STATE_IDLE) {
    if(sched->period == 0) {
      /* request detected! prepare 2nd schedule (just update the period) */
      sched->n_slots = 0;
      sched->period = LWB_PERIOD_T_REQ_MIN + LWB_SCHED_N_SLOTS(sched) * 
                        (LWB_CONF_T_DATA + LWB_CONF_T_GAP) /
                        (RTIMER_SECOND_HF / LWB_PERIOD_SCALE);
      sched_state = LWB_SCHED_STATE_CONT_DET; /* change state */
      return 0;   /* return value doesn't matter here */
      
    } else {    
      /* regular idle round */
      slots_host = 0;
    #if LWB_CONF_MAX_SLOTS_HOST
      /* add slots for the host if requested */
      while(reserve_slot_host && n_slots_assigned < LWB_CONF_MAX_SLOTS_HOST) {
        sched->slot[n_slots_assigned++] = node_id;
        reserve_slot_host--;
      }
      slots_host = n_slots_assigned;
    #endif /* LWB_CONF_MAX_SLOTS_HOST */
      sched->n_slots = n_slots_assigned;
      sched->period  = period;
      if(n_slots_assigned) {
        SET_DATA_ROUND(sched);
      }
      LWB_SCHED_SET_CONT_SLOT(sched);
      SET_STATE_IDLE(sched);  /* will be used by source nodes */
      
      sched_state = LWB_SCHED_STATE_IDLE;   /* stay in idle state */
    }
  } else if(sched_state == LWB_SCHED_STATE_CONT_DET) {
    DEBUG_PRINT_VERBOSE("initiating a request round");
    /* clear the content of the schedule */
    memset(sched->slot, 0, sizeof(sched->slot)); 
    /* every node gets one slot (a chance to request a stream) */
    lwb_stream_list_t *curr_stream = list_head(streams_list);
    while(curr_stream != NULL) {
      sched->slot[n_slots_assigned++] = curr_stream->id;
      /* go to the next stream in the list */
      curr_stream = curr_stream->next;
    }
    if(n_slots_assigned == 0) {
      // if there are no registered nodes, then add a dummy slot
      sched->slot[n_slots_assigned++] = 0xffff;
    }
    sched->period  = LWB_PERIOD_T_DATA;
    sched->n_slots = n_slots_assigned;
    
    sched_state = LWB_SCHED_STATE_REQ;
    
  } else if(sched_state == LWB_SCHED_STATE_REQ) {
    /* calculate the schedule based on the received stream requests */
    /* clear the content of the schedule */
    memset(sched->slot, 0, sizeof(sched->slot)); 
    /* are there any streams? */
    if(n_streams) {  
      /* go through the list of streams */
      lwb_stream_list_t *curr_stream = list_head(streams_list);
      while(curr_stream != NULL && 
            (n_slots_assigned < LWB_CONF_MAX_DATA_SLOTS)) {
        if(curr_stream->state) {
          uint8_t n = curr_stream->n_pkts;
          /* assign as many slots as the node requested */
          while(n && (n_slots_assigned < LWB_CONF_MAX_DATA_SLOTS)) {
            sched->slot[n_slots_assigned++] = curr_stream->id;
            n--;
          }
        }
        /* go to the next stream in the list */
        curr_stream = curr_stream->next;
      }
    }
    sched->n_slots = n_slots_assigned;
    sched->period = (uint16_t)((uint32_t)period - 
                               LWB_PERIOD_T_DATA - LWB_PERIOD_T_REQ_MIN -
                              slots_host * (LWB_CONF_T_DATA + LWB_CONF_T_GAP) /
                               (RTIMER_SECOND_HF / LWB_PERIOD_SCALE));
    SET_DATA_ROUND(sched);            /* mark the next round as 'data round' */
    SET_STATE_IDLE(sched);            /* mark as 'idle' after this round */
    
    sched_state = LWB_SCHED_STATE_DATA;
    
  } else if(sched_state == LWB_SCHED_STATE_DATA) {    
    /* deactivate all streams (regardless of whether data was received) */
    lwb_stream_list_t* curr_stream = NULL;
    while(curr_stream != NULL) {
      curr_stream->state = 0;
      curr_stream = curr_stream->next;
    }
    /* reset values, back to idle state */
    slots_host     = 0;
    sched->n_slots = 0;
    sched->period  = period;
    LWB_SCHED_SET_CONT_SLOT(sched);
    
    sched_state = LWB_SCHED_STATE_IDLE;
    SET_STATE_IDLE(sched);  /* will be used by source nodes */
  }
  
  uint8_t compressed_size;
#if LWB_CONF_SCHED_COMPRESS
  compressed_size = lwb_sched_compress((uint8_t*)sched->slot, 
                                       n_slots_assigned);
  if((compressed_size + LWB_SCHED_PKT_HEADER_LEN) > LWB_CONF_MAX_PKT_LEN) {
    DEBUG_PRINT_ERROR("compressed schedule is too big!");
  }
#else
  compressed_size = n_slots_assigned * 2;
#endif /* LWB_CONF_SCHED_COMPRESS */
  
  sched->time = time / LWB_PERIOD_SCALE;
     
  /* log the parameters of the new schedule */
  DEBUG_PRINT_VERBOSE("schedule updated (s=%u T=%u n=%u|%u len=%u)", 
                      n_streams, sched->period * (1000 / LWB_PERIOD_SCALE), 
                      n_slots_assigned, sched->n_slots >> 13, 
                      compressed_size);
  
  return compressed_size + LWB_SCHED_PKT_HEADER_LEN;
}
/*---------------------------------------------------------------------------*/
uint16_t 
lwb_sched_init(lwb_schedule_t* sched) 
{
  printf(" round [ms]: T=%u idle=%u cont=%u data=%u sum=%u\r\n", 
         LWB_CONF_SCHED_PERIOD_IDLE_MS, 
         (uint16_t)LWB_T_IDLE_ROUND_MS, (uint16_t)LWB_T_REQ_ROUND_MS,
         (uint16_t)LWB_T_DATA_ROUND_MS, (uint16_t)LWB_T_ROUND_MAX_MS);
  
  /* check parameters */
  if(LWB_T_ROUND_MAX_MS >= LWB_CONF_SCHED_PERIOD_IDLE_MS) {
    printf("ERROR: invalid parameters for eLWB scheduler\r\n");
    while(1);
  }
      
  /* initialize streams member and list */
  memb_init(&streams_memb);
  list_init(streams_list);
  n_streams = 0;
  time      = 0;                                   /* global time starts now */
  period    = LWB_PERIOD_IDLE;
  sched_state = LWB_SCHED_STATE_IDLE;
  sched->n_slots = 0;
  sched->time    = 0;
  sched->period  = period;
  LWB_SCHED_SET_CONT_SLOT(sched);               /* include a contention slot */
  SET_STATE_IDLE(sched);
  
  /* NOTE: node IDs must be sorted in increasing order */
#if defined(LWB_CONF_SCHED_AE_SRC_NODE_CNT) && LWB_CONF_SCHED_AE_SRC_NODE_CNT
  uint16_t node_ids[LWB_CONF_SCHED_AE_SRC_NODE_CNT] = 
                                          { LWB_CONF_SCHED_AE_SRC_NODE_LIST };
 #if LWB_CONF_SCHED_AE_SRC_NODE_CNT > LWB_CONF_MAX_N_STREAMS
 #error "LWB_CONF_SCHED_AE_SRC_NODE_CNT is too high"
 #endif /* LWB_CONF_SCHED_AE_SRC_NODE_CNT */
  uint16_t i;
  lwb_stream_list_t *prev = 0;
  printf(" %u source nodes registered: ", LWB_CONF_SCHED_AE_SRC_NODE_CNT);
  for(i = 0; i < LWB_CONF_SCHED_AE_SRC_NODE_CNT; i++) {
    if(i && node_ids[i] < node_ids[i - 1]) {
      printf("ERROR node IDs are not sorted!");
      break;
    }
    lwb_stream_list_t* s = memb_alloc(&streams_memb);
    if(s == 0) {
      printf("ERROR out of memory: stream not added");
      break;
    }
    s->id = node_ids[i];
    s->n_pkts  = 0;         /* # packets the node wants to send */
    s->state   = 1;    
    list_insert(streams_list, prev, s); /* or use list_push(streams_list, s) */
    prev = s;
    n_streams++;
    printf("%u ", node_ids[i]);
  }
  printf("\r\n");
#endif /* LWB_CONF_SCHED_AE_INIT_NODES */
  
  return LWB_SCHED_PKT_HEADER_LEN; /* empty schedule, no slots allocated yet */
}
/*---------------------------------------------------------------------------*/
void 
lwb_sched_set_period(uint16_t p)
{
  if (p * 1000 > LWB_T_ROUND_MAX_MS) {
    period = (uint16_t)((uint32_t)p * LWB_PERIOD_SCALE);
  }
}
/*---------------------------------------------------------------------------*/
void
lwb_sched_set_time(uint32_t new_time)
{
  time = (uint64_t)new_time * LWB_PERIOD_SCALE;
}
/*---------------------------------------------------------------------------*/

#endif /* LWB_SCHED_ELWB_DYN */

/**
 * @}
 * @}
 */
