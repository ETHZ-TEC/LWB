/*
 * Copyright (c) 2015, Swiss Federal Institute of Technology (ETH Zurich).
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
 *          Federico Ferrari
 *          Marco Zimmerling
 */

/**
 * @addtogroup  lwb-scheduler
 * @{
 *
 * @defgroup    static-sched Static scheduler
 * @{
 *
 * @brief
 * a very basic scheduler implementation for the LWB
 * 
 * The scheduler operates with a constant period and only accepts new
 * stream requests if the network is not yet saturated.
 * There is exactly one contention slot per round.
 */
 
#include "lwb.h"

#ifdef LWB_SCHED_STATIC

#if !defined(LWB_CONF_STREAM_EXTRA_DATA_LEN) || \
    (LWB_CONF_STREAM_EXTRA_DATA_LEN != 0)
#error "LWB_CONF_STREAM_EXTRA_DATA_LEN not set to 0!"
#endif

/* max. number of packets per period */
#define BANDWIDTH_LIMIT LWB_CONF_MAX_DATA_SLOTS

#ifndef MAX
#define MAX(x, y)       ((x) > (y) ? (x) : (y))
#endif /* MAX */
/*---------------------------------------------------------------------------*/
/**
 * @brief struct to store information about active streams on the host
 */
typedef struct stream_info {
  struct stream_info *next;
  uint16_t node_id;
  uint16_t ipi;
  uint32_t last_assigned;
  uint8_t  stream_id;
  uint8_t  n_cons_missed;
} lwb_stream_list_t;
/*---------------------------------------------------------------------------*/
uint16_t lwb_sched_compress(uint8_t* compressed_data, uint8_t n_slots);
/*---------------------------------------------------------------------------*/
static uint16_t           period;
static uint32_t           time;                               /* global time */
static uint16_t           n_streams;                            /* # streams */
static uint8_t            first_index;         /* offset for the stream list */
static uint8_t            n_slots_assigned;              /* # slots assigned */
static int32_t            used_bw;   /* used bandwidth: # packets per period */
static volatile uint8_t   n_pending_sack = 0;
/* factor of 4 because of the memory alignment and faster index calculation! */
static uint8_t            pending_sack[4 * LWB_CONF_SCHED_SACK_BUFFER_SIZE]; 
/* a list of pointers to the stream info structures, for faster access 
 * (constant time vs. linear time) */
static lwb_stream_list_t* streams[LWB_CONF_MAX_DATA_SLOTS];   
LIST(streams_list);                    /* -> lists only work for data in RAM */
/* data structures to hold the stream info */
MEMB(streams_memb, lwb_stream_list_t, LWB_CONF_MAX_N_STREAMS);
/*---------------------------------------------------------------------------*/
/**
 * @brief   remove a stream from the stream list on the host
 * @param[in] the stream to remove
 * @param[in] address (in the ext. mem.) of the stream to remove
 */
static inline void 
lwb_sched_del_stream(lwb_stream_list_t* stream) 
{
  if(0 == stream) {    
    return;  /* entry not found, don't do anything */
  }
  uint16_t node  = stream->node_id;
  uint8_t  stream_id  = stream->stream_id;
  used_bw = used_bw - MAX(1, (LWB_CONF_SCHED_PERIOD_IDLE / stream->ipi));
  if(used_bw < 0) {
      DEBUG_PRINT_ERROR("something went wrong, used_bw < 0");
      used_bw = 0;
  }
  list_remove(streams_list, stream);
  memb_free(&streams_memb, stream);
  n_streams--;
  DEBUG_PRINT_INFO("stream %u.%u removed", node, stream_id);
}
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
  
  if(LWB_INVALID_STREAM_ID == req->stream_id) { 
    DEBUG_PRINT_WARNING("invalid stream request (LWB_INVALID_STREAM_ID)");
    return; 
  }  
  if(n_pending_sack >= LWB_CONF_SCHED_SACK_BUFFER_SIZE) {
    DEBUG_PRINT_WARNING("max. number of pending sack's reached, stream request"
                        " dropped");
    return;
  }
  
  /* add and remove requests are implicitly given by the ipi
   * an ipi of 0 implies 'remove' */
  if(req->ipi > 0) { 
    /* check if stream already exists */
    if(n_streams) {
      for(s = list_head(streams_list); s != 0; s = s->next) {
        if(req->node_id == s->node_id && req->stream_id == s->stream_id) {
          /* already exists -> update the IPI...
           * ... but first, check whether the scheduler can support the 
           * requested data_ipi */
          if(used_bw + MAX(1, (LWB_CONF_SCHED_PERIOD_IDLE / req->ipi)) > 
             BANDWIDTH_LIMIT) {
            DEBUG_PRINT_ERROR("stream req %u.%u dropped, network saturated", 
                              req->node_id, req->stream_id);
            return;
          }
          used_bw = used_bw - MAX(1, (LWB_CONF_SCHED_PERIOD_IDLE / s->ipi)) + 
                    MAX(1, (LWB_CONF_SCHED_PERIOD_IDLE / req->ipi));
          s->ipi = req->ipi;
          s->last_assigned = time;
          s->n_cons_missed = 0;         /* reset this counter */
          DEBUG_PRINT_INFO("stream %u.%u updated (IPI %u)", 
                           req->node_id, req->stream_id, req->ipi);
          goto add_sack;
        }
      }  
    }
    /* does not exist: add the new stream...
     * but first, check whether the scheduler can support the requested ipi */
    if(used_bw + MAX(1, (LWB_CONF_SCHED_PERIOD_IDLE / req->ipi)) > 
       BANDWIDTH_LIMIT) {
      DEBUG_PRINT_WARNING("stream request %u.%u dropped, network saturated", 
                          req->node_id, req->stream_id);
      return;
    }
    used_bw = used_bw + MAX(1, (LWB_CONF_SCHED_PERIOD_IDLE / req->ipi));
    s = memb_alloc(&streams_memb);
    if(s == 0) {
      DEBUG_PRINT_ERROR("out of memory: stream request dropped");
      return;
    }
    s->node_id       = req->node_id;
    s->ipi           = req->ipi;
    s->last_assigned = time;
    s->stream_id     = req->stream_id;
    s->n_cons_missed = 0;
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
    DEBUG_PRINT_INFO("stream %u.%u added (IPI %u)", req->node_id, 
                     req->stream_id, req->ipi);         
  } else {
    /* remove this stream */
    for(s = list_head(streams_list); s != 0; s = s->next) {
      if(req->node_id == s->node_id && req->stream_id == s->stream_id) {
        break;
      }
    }
    lwb_sched_del_stream(s);
  }
add_sack:
  /* insert into the list of pending S-ACKs */
  /* use memcpy to avoid pointer misalignment errors */
  memcpy(pending_sack + n_pending_sack * 4, &req->node_id, 2);  
  pending_sack[n_pending_sack * 4 + 2] = req->stream_id;
  n_pending_sack++;   
}
/*---------------------------------------------------------------------------*/
static inline uint8_t 
lwb_sched_stream_in_list(uint16_t node_id, 
                         uint8_t stream_id, 
                         const uint16_t* node_list, 
                         const uint8_t* stream_list, 
                         uint8_t list_len) 
{
  /* assume that the node IDs in node_list are sorted in increasing order */
  while(list_len && *node_list <= node_id) {
    if(*node_list == node_id && *stream_list == stream_id) {
      return 1;
    }
    node_list++;
    stream_list++;  
    list_len--;
  }
  return 0;
}
/*---------------------------------------------------------------------------*/
void 
lwb_sched_set_period(uint16_t p)
{
  if (p) { period = p; }
}
/*---------------------------------------------------------------------------*/
uint16_t 
lwb_sched_compute(lwb_schedule_t * const sched, 
                  const uint8_t * const streams_to_update, 
                  uint8_t reserve_slot_host) 
{
  static uint16_t slots_tmp[LWB_CONF_MAX_DATA_SLOTS];
    
  first_index = 0; 
  n_slots_assigned = 0;
  
  memset(streams, 0, sizeof(streams));   /* clear content of the stream list */
  lwb_stream_list_t *curr_stream = list_head(streams_list);
  /* loop through all the streams in the list */
  while(curr_stream != NULL) {
    if(lwb_sched_stream_in_list(curr_stream->node_id, curr_stream->stream_id, 
        sched->slot, streams_to_update, LWB_SCHED_N_SLOTS(sched))) {
      curr_stream->n_cons_missed = 0;
    } else if(curr_stream->n_cons_missed & 0x80) {
      /* no packet received from this stream */
      curr_stream->n_cons_missed &= 0x7f; /* clear the last bit */
      curr_stream->n_cons_missed++;
    }
    if(curr_stream->n_cons_missed > LWB_CONF_SCHED_STREAM_REMOVAL_THRES) {
      /* too many consecutive slots without reception: delete this stream */
      lwb_stream_list_t *stream_to_remove = curr_stream;
      curr_stream = curr_stream->next;
      lwb_sched_del_stream(stream_to_remove);
    } else {
      curr_stream = curr_stream->next;
    }
  }
  /* clear the content of the schedule (do NOT move this line further above!)*/
  memset(sched->slot, 0, sizeof(sched->slot));  
  
  /* assign slots to the host */
  if(reserve_slot_host) {
    DEBUG_PRINT_INFO("assigning a slot to the host");
    sched->slot[0] = node_id;
    n_slots_assigned++;
  }
  
  /* keep the round period constant */

  time += period;   /* increment time by the current period */

  if(n_streams == 0) {
    /* no streams to process */
    goto set_schedule;
  }

  /* random initial position in the list */
  uint16_t rand_init_pos = (random_rand() >> 1) % n_streams;
  uint16_t i;
  
  curr_stream = list_head(streams_list);
  /* make curr_stream point to the random initial position */
  for(i = 0; i < rand_init_pos; i++) {
    curr_stream = curr_stream->next;
  }
  /* initial stream being processed */
  lwb_stream_list_t *init_stream = curr_stream;
  do {
    /* assign slots for this stream, if possible */
    if((n_slots_assigned < LWB_CONF_MAX_DATA_SLOTS) && 
       (time >= (curr_stream->ipi + curr_stream->last_assigned))) {
      /* the number of slots to assign to curr_stream */
      uint16_t to_assign = (time - curr_stream->last_assigned) / 
                           curr_stream->ipi;  /* elapsed time / period */
      if(to_assign > LWB_CONF_MAX_DATA_SLOTS - n_slots_assigned) {
        to_assign = LWB_CONF_MAX_DATA_SLOTS - n_slots_assigned;
      }
      curr_stream->last_assigned += to_assign * curr_stream->ipi;
      for(; to_assign > 0; to_assign--, n_slots_assigned++) {
        slots_tmp[n_slots_assigned] = curr_stream->node_id;
        streams[n_slots_assigned] = curr_stream;
      }
      /* set the last bit, we are expecting a packet from this stream in 
       * the next round */
      curr_stream->n_cons_missed |= 0x80; 
    }
    /* go to the next stream in the list */
    curr_stream = curr_stream->next;
    if(curr_stream == NULL) {
      /* end of the list: start again from the head of the list */
      curr_stream = list_head(streams_list);
      first_index = n_slots_assigned; 
    }
  } while(curr_stream != init_stream);
  
  /* copy into new data structure to keep the node IDs ordered */
  memcpy(&sched->slot[reserve_slot_host], &slots_tmp[first_index], 
         (n_slots_assigned - first_index) * sizeof(sched->slot[0]));
  memcpy(&sched->slot[n_slots_assigned - first_index + reserve_slot_host], 
         slots_tmp, first_index * sizeof(sched->slot[0]));
  
set_schedule:
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

  /* this schedule is sent at the end of a round: do not communicate 
   * (i.e. do not set the first bit of period) */
  sched->period = period;   /* no need to clear the last bit */
  sched->time   = time;
    
  /* log the parameters of the new schedule */
  DEBUG_PRINT_INFO("schedule updated (s=%u T=%u n=%u|%u l=%u load=%u%%)", 
                   n_streams, sched->period, n_slots_assigned, 
                   sched->n_slots >> 14, compressed_size,
                   (uint16_t)(used_bw * 100 / BANDWIDTH_LIMIT));
  
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
  n_slots_assigned = 0;
  n_pending_sack = 0;
  time = 0;                             /* global time starts now */
  period = LWB_CONF_SCHED_PERIOD_IDLE; 
  sched->n_slots = 0;
  LWB_SCHED_SET_CONT_SLOT(sched);       /* include a contention slot */
  sched->time = time;
  sched->period = period;
  /* mark as the first schedule (beginning of a round) */
  LWB_SCHED_SET_AS_1ST(sched); 
  
  DEBUG_PRINT_INFO("static scheduler initialized (max streams: %u)", 
                   LWB_CONF_MAX_N_STREAMS);
  
  return LWB_SCHED_PKT_HEADER_LEN; /* empty schedule, no slots allocated yet */
}
/*---------------------------------------------------------------------------*/

#endif /* LWB_SCHED_STATIC */

/**
 * @}
 * @}
 */
