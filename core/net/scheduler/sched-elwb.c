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

#ifdef LWB_SCHED_ELWB

#ifndef LWB_CONF_PERIOD_SCALE
#define LWB_CONF_PERIOD_SCALE           100         /* also change in elwb.c */
#endif /* LWB_CONF_PERIOD_SCALE */
#ifndef LWB_CONF_PERIOD_MIN
#define LWB_CONF_PERIOD_MIN             4
#endif /* LWB_CONF_PERIOD_MIN */

/* parameter sanity check */

#if !defined(LWB_CONF_STREAM_EXTRA_DATA_LEN) || \
    (LWB_CONF_STREAM_EXTRA_DATA_LEN != 0)
#error "LWB_CONF_STREAM_EXTRA_DATA_LEN not set to 0!"
#endif

#if LWB_CONF_MAX_N_STREAMS > LWB_CONF_MAX_DATA_SLOTS
#error "max. # of streams mustn't be bigger than max. # of data slots"
#endif

/* duration of the 'request round', depends on the max. # nodes
 * (= LWB_CONF_MAX_N_STREAMS); add 20ms slack time to compute the schedule */
#ifndef LWB_CONF_T_REQ_ROUND
#define LWB_CONF_T_REQ_ROUND    (uint16_t)( \
                                 (LWB_CONF_T_SCHED + LWB_CONF_T_GAP + \
                                  LWB_CONF_MAX_N_STREAMS * \
                                   (LWB_CONF_T_CONT + LWB_CONF_T_GAP) + \
                                  (RTIMER_SECOND_HF / 50)) / \
                                 (RTIMER_SECOND_HF / LWB_CONF_PERIOD_SCALE))
#endif /* LWB_CONF_T_REQ_ROUND */


#define LWB_T_IDLE_ROUND_MS     ((2 * LWB_CONF_T_SCHED + 3 * LWB_CONF_T_GAP + \
                                  LWB_CONF_T_CONT) * 1000 / RTIMER_SECOND_HF)
#define LWB_T_REQ_ROUND_MS      (LWB_CONF_T_REQ_ROUND * 1000 / \
                                 LWB_CONF_PERIOD_SCALE)
#define LWB_T_DATA_ROUND_MS     ((LWB_CONF_T_SCHED + LWB_CONF_T_GAP + \
                                  LWB_CONF_MAX_DATA_SLOTS * \
                                  (LWB_CONF_T_DATA + LWB_CONF_T_GAP)) * 1000 /\
                                 RTIMER_SECOND_HF)
#define LWB_T_ROUND_MAX_MS       (LWB_CONF_PERIOD_MIN * 1000 / \
                                  LWB_CONF_PERIOD_SCALE + LWB_T_REQ_ROUND_MS +\
                                  LWB_T_DATA_ROUND_MS)

#ifndef MAX
#define MAX(x,y)        ((x) > (y) ? (x) : (y))
#endif /* MAX */

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
  LWB_SCHED_STATE_REQ,
  LWB_SCHED_STATE_DATA,
} lwb_sched_state_t;
/*---------------------------------------------------------------------------*/
static uint32_t          time;                                /* global time */
static uint16_t          n_streams;                      /* # active streams */
static lwb_sched_state_t sched_state;                     /* scheduler state */
static uint8_t           n_pending_sack;
static uint16_t          pending_sack[LWB_CONF_MAX_N_STREAMS];
LIST(streams_list);                    /* -> lists only work for data in RAM */
/* data structures to hold the stream info */
MEMB(streams_memb, lwb_stream_list_t, LWB_CONF_MAX_N_STREAMS);
/*---------------------------------------------------------------------------*/
uint8_t 
lwb_sched_prepare_sack(void *payload) 
{
  if(n_pending_sack) {
    DEBUG_PRINT_VERBOSE("%u S-ACKs pending", n_pending_sack);
    memcpy(payload, pending_sack, n_pending_sack * 2);
    return n_pending_sack * 2;
  }
  return 0;         /* return the length of the packet */
}
/*---------------------------------------------------------------------------*/
void 
lwb_sched_proc_srq(const lwb_stream_req_t* req) 
{
  lwb_stream_list_t *s = 0;
  uint8_t exists = 0;
    
  /* check if stream already exists */
  for(s = list_head(streams_list); s != 0; s = s->next) {
    if(req->id == s->id) {
      if(!s->state) {
        n_streams++;
        s->state = 1;
      }
      s->n_pkts = MAX(1, req->reserved); /* # packets the node wants to send */
      exists = 1;
      break;
    }
  }   
  if (!exists)
  {      
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
  }
  /* no need to send a stream acknowledgement */
}
/*---------------------------------------------------------------------------*/
uint16_t 
lwb_sched_compute(lwb_schedule_t * const sched, 
                  const uint8_t * const streams_to_update, 
                  uint8_t reserve_slot_host) 
{ 
  uint8_t n_slots_assigned = 0;  
  
  /* increment the timestamp */
  time += sched->period;
  
  /* scheduler states:
   * LWB_SCHED_STATE_IDLE: idle
   * LWB_SCHED_STATE_REQ: contention detected, request round is next
   * LWB_SCHED_STATE_DATA: data round is next
   * 
   * note: the schedule is sent at the beginning of the next round,
   * i.e. it must include the next period
   */
  if(sched_state == LWB_SCHED_STATE_IDLE) {
    if(sched->period == LWB_CONF_PERIOD_MIN) {
      /* LWB sets the period to LWB_CONF_PERIOD_MIN if at least a preamble and
       * sync word has been detected during the last contention slot! */
      DEBUG_PRINT_VERBOSE("initiating a request round");
      /* clear the content of the schedule */
      memset(sched->slot, 0, sizeof(sched->slot)); 
      /* every node gets one slot (a chance to request a stream) */
      lwb_stream_list_t *curr_stream = list_head(streams_list);
      while(curr_stream != NULL) {
        sched->slot[n_slots_assigned] = curr_stream->id;
        n_slots_assigned++;
        /* go to the next stream in the list */
        curr_stream = curr_stream->next;
      }
      sched->period  = LWB_CONF_T_REQ_ROUND;
      sched->n_slots = n_slots_assigned;
      sched_state = LWB_SCHED_STATE_REQ;
    } else {
      /* regular idle round */
      sched->n_slots = 0;
      sched->period  = LWB_CONF_SCHED_PERIOD_IDLE * LWB_CONF_PERIOD_SCALE;
      /* no data slots, just a contention slot */
      LWB_SCHED_SET_CONT_SLOT(sched);
    }
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
    sched->period = LWB_CONF_SCHED_PERIOD_IDLE * LWB_CONF_PERIOD_SCALE - 
                    LWB_CONF_T_REQ_ROUND - LWB_CONF_PERIOD_MIN;
    sched_state = LWB_SCHED_STATE_DATA;
    /* the next round will include an acknowledgement slot */
    LWB_SCHED_SET_SACK_SLOT(sched);
    /* note: even if the sack slot is not used, still keep this marking to
    * distinguish between a request and a data round! */

    /* important: don't skip the data round! otherwise the state machine falls
     * apart */
  } else if(sched_state == LWB_SCHED_STATE_DATA) {      
    n_pending_sack = 0;
    /* deactivate a stream if we received data */
    uint16_t i = 0;
    lwb_stream_list_t* curr_stream = NULL;
    while(i < LWB_SCHED_N_SLOTS(sched)) {
      /* the node IDs will be in order */
      if(!curr_stream || (sched->slot[i] != curr_stream->id)) {
        /* find the stream with this node ID in the list */
        curr_stream = list_head(streams_list);
        while(curr_stream != NULL) {
          if(sched->slot[i] == curr_stream->id) {
            break;
          }
          curr_stream = curr_stream->next;
        }
      }
      /* ignore inactive streams */
      if(curr_stream != NULL && curr_stream->state) {
        /* did we receive data from this node? */
        if(streams_to_update[i] != 0) {
          curr_stream->n_pkts--;
          if(curr_stream->n_pkts == 0) {
            /* schedule an S-ACK and deactivate the stream */
            pending_sack[n_pending_sack] = curr_stream->id;
            n_pending_sack++;
            curr_stream->state = 0;
            if(n_streams == 0) {
              DEBUG_PRINT_WARNING("invalid stream cound");
            } else {
              n_streams--;
            }
            DEBUG_PRINT_VERBOSE("data received, stream removed");
          }
          /* error check */
          if(curr_stream->n_pkts == 0xff) {
            DEBUG_PRINT_ERROR("invalid packet count in lwb_sched_compute");
            curr_stream->n_pkts = 0;
          }
        }
      } else {
        /* this is not supposed to happen */
        DEBUG_PRINT_WARNING("data received from unknown stream");
      }
      i++;
    }
    sched->n_slots = 0;    
    sched->period  = LWB_CONF_SCHED_PERIOD_IDLE * LWB_CONF_PERIOD_SCALE;
    LWB_SCHED_SET_CONT_SLOT(sched);
    sched_state = LWB_SCHED_STATE_IDLE;    /* back to idle state */
  }
  
  sched->time = time / LWB_CONF_PERIOD_SCALE;  
     
  /* log the parameters of the new schedule */
  DEBUG_PRINT_VERBOSE("schedule updated (s=%u T=%u n=%u|%u len=%u)", 
                      n_streams, sched->period / LWB_CONF_PERIOD_SCALE, 
                      n_slots_assigned, sched->n_slots >> 14, 
                      n_slots_assigned * 2);
  
  return (n_slots_assigned * 2) + LWB_SCHED_PKT_HEADER_LEN;
}
/*---------------------------------------------------------------------------*/
uint16_t 
lwb_sched_init(lwb_schedule_t* sched) 
{
  printf(" using eLWB scheduler with T=%u\r\n"
         " round times [ms]: idle=%u cont=%u data=%u total=%u\r\n", 
         LWB_CONF_SCHED_PERIOD_IDLE,
         (uint16_t)LWB_T_IDLE_ROUND_MS, (uint16_t)LWB_T_REQ_ROUND_MS,
         (uint16_t)LWB_T_DATA_ROUND_MS, (uint16_t)LWB_T_ROUND_MAX_MS);
  
  /* check parameters */
  if(((LWB_CONF_PERIOD_MIN * 1000 / LWB_CONF_PERIOD_SCALE) <= 
      LWB_T_IDLE_ROUND_MS) ||
     (LWB_T_ROUND_MAX_MS >= LWB_CONF_SCHED_PERIOD_IDLE * 1000)) {
    printf("ERROR: invalid parameters for eLWB scheduler\r\n");
    while(1);
  }
      
  /* initialize streams member and list */
  memb_init(&streams_memb);
  list_init(streams_list);
  n_streams = 0;
  sched_state = LWB_SCHED_STATE_IDLE;
  n_pending_sack = 0;
  time = 0;                             /* global time starts now */
  sched->n_slots = 0;
  LWB_SCHED_SET_CONT_SLOT(sched);       /* include a contention slot */
  sched->time = time;
  sched->period = LWB_CONF_SCHED_PERIOD_IDLE * LWB_CONF_PERIOD_SCALE;
  
#if defined(LWB_CONF_SCHED_AE_SRC_NODE_CNT) && LWB_CONF_SCHED_AE_SRC_NODE_CNT
  uint16_t nodes_ids[LWB_CONF_SCHED_AE_SRC_NODE_CNT] = 
                                          { LWB_CONF_SCHED_AE_SRC_NODE_LIST };
 #if LWB_CONF_SCHED_AE_SRC_NODE_CNT > LWB_CONF_MAX_N_STREAMS
 #error "LWB_CONF_SCHED_AE_SRC_NODE_CNT is too high"
 #endif /* LWB_CONF_SCHED_AE_SRC_NODE_CNT */
  uint16_t i;
  printf(" %u source nodes registered: ", LWB_CONF_SCHED_AE_SRC_NODE_CNT);
  for(i = 0; i < LWB_CONF_SCHED_AE_SRC_NODE_CNT; i++) {
    lwb_stream_list_t* s = memb_alloc(&streams_memb);
    if(s == 0) {
      printf("ERROR out of memory: stream not added");
      break;
    }
    s->id = nodes_ids[i];
    s->n_pkts  = 0;         /* # packets the node wants to send */
    s->state   = 1;
    list_push(streams_list, s);
    n_streams++;
    printf("%u ", nodes_ids[i]);
  }
  printf("\r\n");
#endif /* LWB_CONF_SCHED_AE_INIT_NODES */
  
  return LWB_SCHED_PKT_HEADER_LEN; /* empty schedule, no slots allocated yet */
}
/*---------------------------------------------------------------------------*/
void
lwb_sched_set_time(uint32_t new_time)
{
  time = new_time;
}
/*---------------------------------------------------------------------------*/

#endif /* LWB_SCHED_ELWB */

/**
 * @}
 * @}
 */
