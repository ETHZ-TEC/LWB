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
 */

/**
 * @addtogroup  lwb-scheduler
 * @{
 *
 * @defgroup    sched-fast scheduler supporting high throughput bursts
 * @{
 *
 * @brief
 * a simple scheduler implementation for the LWB
 * 
 * The scheduler has a static base period. 
 * There is no IPI (inter-packet interval) required for stream requests, the
 * scheduler will assign as many slots as possible to each stream until all
 * the data has been transmitted. The scheduler tries to be fair by assigning
 * an equal number of slots to each stream. There is no need to remove a 
 * stream. Once a certain number of slots passed without receiving data, the
 * scheduler will automatically remove the stream.
 * This scheduler is therefore suited for applications where most of the time 
 * no data is transmitted, but temporary high traffic demands can arise at any 
 * point in time. (event based high throughput bursts)
 * 
 * NOTE: 
 * - max. number of streams must not exceed the max. number of data slots
 * - the stream ID will be ignored for requests, there is only one stream per 
 *   node allowed.
 * - the scheduler is inefficient for periodic traffic
 * - in every round there is 1 contention slot
 * - the scheduler keeps a list of 'known sources', i.e. nodes that have sent
 *   requests in the past
 * - the minimal idle period is 2s
 */
 
#include "lwb.h"

#ifdef LWB_SCHED_BURST

/* parameter sanity check */

#if !defined(LWB_CONF_STREAM_EXTRA_DATA_LEN) || \
    (LWB_CONF_STREAM_EXTRA_DATA_LEN != 0)
#error "LWB_CONF_STREAM_EXTRA_DATA_LEN not set to 0!"
#endif

#if LWB_CONF_MAX_N_STREAMS > LWB_CONF_MAX_DATA_SLOTS
#error "max. # of streams mustn't be bigger than max. # of data slots"
#endif

#if LWB_CONF_SCHED_PERIOD_IDLE < 2
#error "LWB_CONF_SCHED_PERIOD_IDLE must be at least 2"
#endif

/* max. number of nodes in the network. the scheduler keeps the ID of each
 * node in the network in a list */
#ifndef LWB_CONF_SCHED_MAX_NODES
#define LWB_CONF_SCHED_MAX_NODES        10
#endif /* LWB_CONF_SCHED_MAX_NODES */

#ifndef MIN
#define MIN(x,y)        ((x) < (y) ? (x) : (y))
#endif /* MIN */

/*---------------------------------------------------------------------------*/
/**
 * @brief struct to store information about active streams on the host
 */
typedef struct stream_info {
  struct stream_info *next;
  uint16_t            node_id;
  uint8_t             n_cons_missed;
  uint8_t             state;
} lwb_stream_list_t;
/*---------------------------------------------------------------------------*/
uint16_t lwb_sched_compress(uint8_t* compressed_data, uint8_t n_slots);
/*---------------------------------------------------------------------------*/
static uint32_t           time;                               /* global time */
static uint16_t           n_streams;                     /* # active streams */
static volatile uint8_t   n_pending_sack = 0; 
/* factor of 4 because of the memory alignment and faster index calculation! */
static uint8_t            pending_sack[4 * LWB_CONF_SCHED_SACK_BUFFER_SIZE];  
LIST(streams_list);                    /* -> lists only work for data in RAM */
/* data structures to hold the stream info */
MEMB(streams_memb, lwb_stream_list_t, LWB_CONF_MAX_N_STREAMS);
/*---------------------------------------------------------------------------*/
uint8_t 
lwb_sched_prepare_sack(void *payload) 
{
  if(n_pending_sack) {
    DEBUG_PRINT_VERBOSE("%u S-ACKs pending", n_pending_sack);
    memcpy(payload, pending_sack, n_pending_sack * 4);
    ((lwb_stream_ack_t*)payload)->n_extra = n_pending_sack - 1;
    n_pending_sack = 0;
    return (((lwb_stream_ack_t*)payload)->n_extra + 1) * 4;
  }
  return 0;         /* return the length of the packet */
}
/*---------------------------------------------------------------------------*/
void 
lwb_sched_proc_srq(const lwb_stream_req_t* req) 
{
  lwb_stream_list_t *s = 0;
  uint8_t exists = 0;
  
  /* check error conditions */
  if(n_pending_sack >= LWB_CONF_SCHED_SACK_BUFFER_SIZE) {
    DEBUG_PRINT_WARNING("max. number of pending sack's reached, stream request"
                        " dropped");
    return;
  }
  
  /* check if stream already exists */
  for(s = list_head(streams_list); s != 0; s = s->next) {
    if(req->node_id == s->node_id) {
      s->n_cons_missed = 0;          /* reset this counter */
      if(!s->state) {
        n_streams++;
        s->state = 1;
        DEBUG_PRINT_INFO("stream of node %u reactivated", req->node_id);
      }
      exists = 1;                    /* already exists */
      break;
    }
  }   
  if (!exists)
  {   
    if(n_streams >= LWB_CONF_MAX_N_STREAMS) {
      DEBUG_PRINT_WARNING("stream request from node %u dropped, max #streams "
                          "reached", req->node_id);
      return;
    }
    /* does not exist: add the new stream */
    s = memb_alloc(&streams_memb);
    if(s == 0) {
      DEBUG_PRINT_ERROR("out of memory: stream request dropped");
      return;
    }
    s->node_id       = req->node_id;
    s->n_cons_missed = 0;
    s->state         = 1;
    /* insert the stream into the list, ordered by node id */
    lwb_stream_list_t *prev;
    for(prev = list_head(streams_list); prev != NULL; prev = prev->next) {
      if((req->node_id >= prev->node_id) && ((prev->next == NULL) || 
         (req->node_id < prev->next->node_id))) {
        break;
      }
    }
    list_insert(streams_list, prev, s);   
    n_streams++;
    DEBUG_PRINT_INFO("stream of node %u added", req->node_id);         
  }

  /* insert into the list of pending S-ACKs */
  /* use memcpy to avoid pointer misalignment errors */
  memcpy(pending_sack + n_pending_sack * 4, &req->node_id, 2);  
  pending_sack[n_pending_sack * 4 + 2] = req->stream_id;
  n_pending_sack++;
}
/*---------------------------------------------------------------------------*/
uint16_t 
lwb_sched_compute(lwb_schedule_t * const sched, 
                  const uint8_t * const streams_to_update, 
                  uint8_t reserve_slot_host) 
{ 
  uint8_t n_slots_assigned = 0;  
        
  /* request round next? all of the following conditions must apply:
   * - last slot was a contention slot (since there's always a contention slot 
   *   at the end of a round, this condition is always met)
   * - radio activity was detected during the last glossy slot
   * - there were no data slots in the last round */
  if(glossy_get_n_rx_started() && 
     !LWB_SCHED_N_SLOTS(sched)) {
    DEBUG_PRINT_INFO("initiating a request round");
    /* clear the content of the schedule */
    memset(sched->slot, 0, sizeof(sched->slot)); 
    /* this is a special round, each node gets one slot (a chance to request
     * a stream) */
    lwb_stream_list_t *curr_stream = list_head(streams_list);
    while(curr_stream != NULL) {
      sched->slot[n_slots_assigned] = curr_stream->node_id;
      n_slots_assigned++;
      /* go to the next stream in the list */
      curr_stream = curr_stream->next;
    }
    
    /* do not update the time */
    sched->period = LWB_CONF_SCHED_PERIOD_IDLE - 1;
    time++;
    
  } else {
    /* loop through all the slots of the last schedule */
    uint16_t i = 0;
    while (i < LWB_SCHED_N_SLOTS(sched)) {
      /* the node IDs will be in order */
      uint16_t curr_node_id = sched->slot[i];
      /* find the stream with this node ID in the list */
      lwb_stream_list_t *curr_stream = list_head(streams_list);
      while(curr_stream != NULL) {
        if(sched->slot[i] == curr_stream->node_id) {
          break;
        }
        curr_stream = curr_stream->next;
      }
      /* ignore inactive streams */
      if(curr_stream != NULL && curr_stream->state) {
        /* go through all slots with the same node ID */
        do {
          if(streams_to_update[i] != LWB_INVALID_STREAM_ID) {
            curr_stream->n_cons_missed = 0;  /* data received, reset counter */
          } else {
            /* no data received in this slot -> increment counter */
            curr_stream->n_cons_missed &= 0x7f; /* clear the last bit */
            curr_stream->n_cons_missed++;
          }
          i++;
        } while (curr_node_id == sched->slot[i]);
    
        if(curr_stream->n_cons_missed > LWB_CONF_SCHED_STREAM_REMOVAL_THRES) {
          /* too many consecutive slots without reception: remove stream */
          curr_stream->state = 0;
          n_streams--;
          DEBUG_PRINT_INFO("stream of node %u removed", curr_stream->node_id);
        }
      } else {
        /* this is not supposed to happen */
        DEBUG_PRINT_WARNING("data received from unknown stream");
      }
    }
    /* clear the content of the schedule */
    memset(sched->slot, 0, sizeof(sched->slot)); 
    
    /* assign slots to the host */
    if(reserve_slot_host) {
      DEBUG_PRINT_INFO("assigning a slot to the host");
      sched->slot[0] = node_id;
      n_slots_assigned++;
    } 
    
    /* update time, keep the period constant */
    time += sched->period; //LWB_CONF_SCHED_PERIOD_IDLE;
    
    /* are there any streams? */
    if(n_streams) {
      uint8_t n_slots_per_stream = LWB_CONF_MAX_DATA_SLOTS / n_streams;
  
      /* go again through the list of streams */
      lwb_stream_list_t *curr_stream = list_head(streams_list);
      while(curr_stream != NULL && 
            (n_slots_assigned < LWB_CONF_MAX_DATA_SLOTS)) {
        if(curr_stream->state) {
          /* the number of slots to assign to curr_stream */
          uint8_t to_assign = MIN((LWB_CONF_MAX_DATA_SLOTS - n_slots_assigned),
                                  n_slots_per_stream);
          do {
            sched->slot[n_slots_assigned] = curr_stream->node_id;
            n_slots_assigned++;
            to_assign--;
          } while (to_assign);
          /* set the last bit, we are expecting a packet from this stream in 
          * the next round */
          curr_stream->n_cons_missed |= 0x80; 
        }    
        /* go to the next stream in the list */
        curr_stream = curr_stream->next;
      }
    }    
    sched->period = LWB_CONF_SCHED_PERIOD_IDLE;
  }
  sched->time = time;
  
  sched->n_slots = n_slots_assigned;
  if(n_pending_sack) {
    LWB_SCHED_SET_SACK_SLOT(sched);
  }  
  /* always schedule a contention slot! */
  LWB_SCHED_SET_CONT_SLOT(sched);
  
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
    
  /* log the parameters of the new schedule */
  DEBUG_PRINT_INFO("schedule updated (s=%u T=%u n=%u|%u len=%u)", 
                   n_streams, sched->period & 0x7fff, n_slots_assigned, 
                   sched->n_slots >> 14, compressed_size);
  
  return compressed_size + LWB_SCHED_PKT_HEADER_LEN;
}
/*---------------------------------------------------------------------------*/
uint16_t 
lwb_sched_init(lwb_schedule_t* sched) 
{
  /* initialize streams member and list */
  memb_init(&streams_memb);
  list_init(streams_list);
  n_streams = 0;
  n_pending_sack = 0;
  time = 0;                             /* global time starts now */
  sched->n_slots = 0;
  LWB_SCHED_SET_CONT_SLOT(sched);       /* include a contention slot */
  sched->time = time;
  sched->period = LWB_CONF_SCHED_PERIOD_IDLE;
  
  DEBUG_PRINT_INFO("burst scheduler initialized (max streams: %u)", 
                   LWB_CONF_MAX_N_STREAMS);
  
  return LWB_SCHED_PKT_HEADER_LEN; /* empty schedule, no slots allocated yet */
}
/*---------------------------------------------------------------------------*/

#endif /* LWB_SCHED_BURST */

/**
 * @}
 * @}
 */
