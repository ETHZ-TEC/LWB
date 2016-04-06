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
 * @defgroup    sched-min-energy Min energy scheduler
 * @{
 *
 * @brief
 * a scheduler implementation for the LWB
 * 
 * The scheduler tries to minimize energy consumption by maximizing the
 * period such that the bandwidth demands in the network are still met.
 * Besides, the scheduler keeps the period changes to a minimum
 * to reduce chances of a link loss and thus increase stability.
 * 
 * @remarks:
 * - TIME_SCALE removed (i.e. fixed to 1)
 * - dynamic period only
 * - JOINING_NODES, TWO_SCHEDS, COMPRESS, REMOVE_NODES and DYNAMIC_FREE_SLOTS 
 *   set to 1 (i.e. removed)
 * - everything with MINIMIZE_LATENCY removed
 * - external memory support added
 * - list for pending S-ACKs added
 */
 
#include "lwb.h"

#ifdef LWB_SCHED_MIN_ENERGY

#if !defined(LWB_CONF_STREAM_EXTRA_DATA_LEN) || \
    (LWB_CONF_STREAM_EXTRA_DATA_LEN != 1)
#error "LWB_CONF_STREAM_EXTRA_DATA_LEN not set to 1!"
#endif

/*---------------------------------------------------------------------------*/
typedef struct {
  /* starting time offset (necessary to get rid of e.g. a backlog of messages 
   * within a short time) */
  int8_t   t_offset;    
} lwb_stream_extra_data_t;

typedef struct {
  uint16_t n_added;
  uint16_t n_deleted;
  uint16_t n_no_space;
  uint32_t t_last_req;    /* timestamp of the last stream request  */
  uint32_t t_last_cont;   /* timestamp of the last contention slot */
} lwb_sched_stats_t;
/*---------------------------------------------------------------------------*/
/**
 * @brief struct to store information about active streams on the host
 */
typedef struct stream_info {
#if !LWB_CONF_SCHED_USE_XMEM
  struct stream_info *next;
#else
  uint32_t next;      
#endif /* LWB_CONF_SCHED_USE_XMEM */
  uint16_t node_id;
  uint16_t ipi;
  uint32_t last_assigned;
  uint8_t  stream_id;
  uint8_t  n_cons_missed;
} lwb_stream_list_t;
/*---------------------------------------------------------------------------*/
uint16_t lwb_sched_compress(uint8_t* compressed_data, uint8_t n_slots);
/*---------------------------------------------------------------------------*/
static uint8_t           period;
static uint32_t          time;               /* global time */
static uint16_t          n_streams;          /* # streams */
static lwb_sched_stats_t sched_stats = { 0 };
static uint8_t           first_index;        /* offset for the stream list */
static uint8_t           n_slots_assigned;   /* # assigned slots */
static uint32_t          data_cnt;
static uint16_t          data_ipi;
static volatile uint8_t  n_pending_sack = 0;
/* factor of 4 because of the memory alignment and faster index calculation! */
static uint8_t           pending_sack[4 * LWB_CONF_SCHED_SACK_BUFFER_SIZE]; 
#if !LWB_CONF_SCHED_USE_XMEM
  /* a list of pointers to the stream info structures, for faster access 
   * (constant time vs. linear time) */
  static lwb_stream_list_t  *streams[LWB_CONF_MAX_DATA_SLOTS];   
  LIST(streams_list);                  /* -> lists only work for data in RAM */
  /* data structures to hold the stream info */
  MEMB(streams_memb, lwb_stream_list_t, LWB_CONF_MAX_N_STREAMS);  
#else /* LWB_CONF_SCHED_USE_XMEM */
  /* address of the first linked list element (head) in the external memory 
   * note: do NOT dereference this pointer! */
  static uint32_t streams_list = MEMBX_INVALID_ADDR;  
  /* data structures to hold the stream info */
  MEMBX(streams_memb, sizeof(lwb_stream_list_t), LWB_CONF_MAX_N_STREAMS);
#endif /* LWB_CONF_SCHED_USE_XMEM */
/*---------------------------------------------------------------------------*/
/**
 * @brief   remove a stream from the stream list on the host
 * @param[in] the stream to remove
 * @param[in] address (in the ext. mem.) of the stream to remove
 */
#if !LWB_CONF_SCHED_USE_XMEM
static inline void 
lwb_sched_del_stream(lwb_stream_list_t* stream) 
{
  if(0 == stream) {    
    return;  /* entry not found, don't do anything */
  }
  uint16_t node_id  = stream->node_id;
  uint8_t  stream_id  = stream->stream_id;
  list_remove(streams_list, stream);
  memb_free(&streams_memb, stream);
  n_streams--;
  sched_stats.n_deleted++;  
  DEBUG_PRINT_INFO("stream %u.%u removed", node_id, stream_id);
}
#else
static void 
lwb_sched_del_stream(uint32_t stream_addr) 
{
  if(stream_addr == MEMBX_INVALID_ADDR || streams_list == MEMBX_INVALID_ADDR) {    
    return;
  }  
  lwb_stream_list_t stream;
  uint16_t    node;
  uint8_t     stream_id;
  xmem_read(stream_addr, sizeof(lwb_stream_list_t), (uint8_t*)&stream);
  uint32_t next_addr = (uint32_t)stream.next; 
  if(streams_list == stream_addr) {  /* special case: it's the first element */
    node  = stream.node_id;
    stream_id  = stream.stream_id;
    streams_list = next_addr; 
  } else {
    uint32_t prev_addr = streams_list;
    do {
      xmem_read(prev_addr, sizeof(lwb_stream_list_t), (uint8_t*)&stream);
      if(stream.next == stream_addr) {
        node  = stream.node_id;
        stream_id  = stream.stream_id;
        break;
      }
      prev_addr = stream.next;
    } while(prev_addr != MEMBX_INVALID_ADDR);
    if(prev_addr == MEMBX_INVALID_ADDR) {
      /* not found! */
      DEBUG_PRINT_WARNING("memory block not freed (invalid address)");
      return;
    }
    /* read, modify and write back */
    xmem_read(prev_addr, sizeof(lwb_stream_list_t), (uint8_t*)&stream);
    stream.next = next_addr;                      /* adjust the next-pointer */
    xmem_write(prev_addr, sizeof(lwb_stream_list_t), (uint8_t*)&stream);
  }
  membx_free(&streams_memb, stream_addr);   /* mark the memory block as free */
  n_streams--;
  sched_stats.n_deleted++;
  
  DEBUG_PRINT_INFO("stream %u.%u removed", node, stream_id);
}
#endif /* LWB_CONF_SCHED_USE_XMEM */
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
#if !LWB_CONF_SCHED_USE_XMEM
  lwb_stream_list_t *s  = 0;   
#else /* LWB_CONF_SCHED_USE_XMEM */
  lwb_stream_list_t s, new_stream;
  uint32_t stream_addr, prev_addr;
#endif /* LWB_CONF_SCHED_USE_XMEM */
  lwb_stream_extra_data_t* extra_data = 
    (lwb_stream_extra_data_t*)req->extra_data;
  sched_stats.t_last_req = time;
     
  if(LWB_INVALID_STREAM_ID == req->stream_id) { 
    DEBUG_PRINT_WARNING("invalid stream request (LWB_INVALID_STREAM_ID)");
    return; 
  }  
  if(n_pending_sack >= LWB_CONF_SCHED_SACK_BUFFER_SIZE) {
    DEBUG_PRINT_WARNING("max. number of sack's reached, stream request "
                        "dropped");
    return;
  }
  
  /* add and remove requests are implicitly given by the ipi
   * an ipi of 0 implies 'remove' */
  if(req->ipi > 0) { 
  
#if !LWB_CONF_SCHED_USE_XMEM
    /* check if stream already exists */
    if(n_streams) {
      for(s = list_head(streams_list); s != 0; s = s->next) {
        if(req->node_id == s->node_id && req->stream_id == s->stream_id) {
          /* already exists -> update the IPI */
          s->ipi = req->ipi;
          s->last_assigned = time + extra_data->t_offset;
          s->n_cons_missed = 0;         /* reset this counter */
          DEBUG_PRINT_VERBOSE("stream request %u.%u processed (IPI updated)",
                              req->node_id, req->stream_id);
          goto add_sack;
        }
      }  
    }
    /* does not exist: add the new stream */
    s = memb_alloc(&streams_memb);
    if(s == 0) {
      DEBUG_PRINT_ERROR("out of memory: stream request dropped");
      sched_stats.n_no_space++;  /* no space for new streams */
      return;
    }
    s->node_id       = req->node_id;
    s->ipi           = req->ipi;
    s->last_assigned = ((int32_t)time + (int32_t)extra_data->t_offset > 0) ?
                        (time + extra_data->t_offset) : time;
                        /* (time > req->ipi) ? (time - req->ipi) : time; */
    s->stream_id     = req->stream_id;
    s->n_cons_missed = 0;
    /* insert the stream into the list, ordered by node id */
    lwb_stream_list_t *prev;
    for(prev = list_head(streams_list); prev != NULL; prev = prev->next) {
      if((req->node_id >= prev->node_id) && 
         ((prev->next == NULL) || (req->node_id < prev->next->node_id))) {
        break;
      }
    }
    list_insert(streams_list, prev, s);   
#else      
    /* check whether stream already exists */
    if(n_streams) {
      stream_addr = streams_list;
      do {
        /* load the first block */
        xmem_read(stream_addr, sizeof(lwb_stream_list_t), (uint8_t*)&s);
        /* check the ID */
        if(req->node_id == s.node_id && req->stream_id == s.stream_id) {
          /* already exists -> update the IPI */
          s.ipi = req->ipi;
          s.last_assigned = time + extra_data->t_offset;
          s.n_cons_missed = 0;         /* reset this counter */
          /* insert into the list of pending S-ACKs */
          memcpy(pending_sack + n_pending_sack * 4, &req->node_id, 2);
          pending_sack[n_pending_sack * 4 + 2] = req->stream_id;
          n_pending_sack++;
          DEBUG_PRINT_VERBOSE("stream %u.%u updated (IPI %u)", 
                              req->node_id, req->stream_id, req->ipi);
          /* save the changes */
          xmem_write(stream_addr, sizeof(lwb_stream_list_t), (uint8_t*)&s);
          return;
        }        
        stream_addr = s.next;  /* go to the next address */  
      } while(stream_addr != MEMBX_INVALID_ADDR);
    }        
    /* does not exist: add the new stream */
    stream_addr = membx_alloc(&streams_memb);
    if(stream_addr == MEMBX_INVALID_ADDR) {
      DEBUG_PRINT_ERROR("no memory available to store stream info");
      sched_stats.n_no_space++;  /* no space for new streams */
      return;
    }
    new_stream.node_id       = req->node_id;
    new_stream.ipi           = req->ipi;
    new_stream.last_assigned = ((int32_t)time + 
                                (int32_t)extra_data->t_offset > 0) ? 
                                (time + extra_data->t_offset) : time;
                            /* (time > req->ipi) ? (time - req->ipi) : time; */
    new_stream.stream_id     = req->stream_id;
    new_stream.n_cons_missed = 0;
    new_stream.next          = MEMBX_INVALID_ADDR;
    /* insert the stream into the list, ordered by node id */
    if(streams_list == MEMBX_INVALID_ADDR) {   /* empty list? */
      streams_list = stream_addr;
    } else {
      prev_addr = streams_list;
      do {
        xmem_read(prev_addr, sizeof(lwb_stream_list_t), (uint8_t*)&s);
        /* check the ID */
        if(req->node_id <= s.node_id || s.next == MEMBX_INVALID_ADDR) { 
          /* the element is inserted at the head of the list */
          if(streams_list == prev_addr) { 
            streams_list = stream_addr;
            new_stream.next = prev_addr;
          } else {
            new_stream.next = s.next;
            s.next = stream_addr;
            xmem_write(prev_addr, sizeof(lwb_stream_list_t), (uint8_t*)&s);
          }        
          break;
        }      
        prev_addr = s.next;  /* go to the next address */
      } while(prev_addr != MEMBX_INVALID_ADDR);      
    }
    xmem_write(stream_addr, sizeof(lwb_stream_list_t), (uint8_t*)&new_stream);
#endif /* LWB_CONF_SCHED_USE_XMEM */
    n_streams++;
    sched_stats.n_added++;     
    DEBUG_PRINT_VERBOSE("stream %u.%u added", req->node_id, req->stream_id);

  } else {
#if !LWB_CONF_SCHED_USE_XMEM  
    /* remove this stream */
    for(s = list_head(streams_list); s != 0; s = s->next) {
      if(req->node_id == s->node_id && req->stream_id == s->stream_id) {
        break;
      }
    }
    lwb_sched_del_stream(s);    
#else
    stream_addr = streams_list;
    while(stream_addr != MEMBX_INVALID_ADDR) {
      xmem_read(stream_addr, sizeof(lwb_stream_list_t), (uint8_t*)&s);
      if(req->node_id == s.node_id && req->stream_id == s.stream_id) {
        break;
      }
      stream_addr = s.next;  /* go to the next address */ 
    }
    lwb_sched_del_stream(stream_addr);
#endif  /* LWB_CONF_SCHED_USE_XMEM */
  }
add_sack:
  /* insert into the list of pending S-ACKs */
  memcpy(pending_sack + n_pending_sack * 4, &req->node_id, 2);
  pending_sack[n_pending_sack * 4 + 2] = req->stream_id;
  n_pending_sack++;   
}
/*---------------------------------------------------------------------------*/
/**
 * @brief implementation of the binary GCD algorithm
 * @param[in] u an unsigned integer
 * @param[in] v an unsigned integer
 * @return the greatest common divider of u and v
 */
static inline uint16_t 
gcd(uint16_t u, uint16_t v) 
{
  uint16_t shift;

  /* GCD(0,x) := x */
  if(u == 0 || v == 0) {
    return u | v;
  }

  /* Let shift := lg K, where K is the greatest power of 2
     dividing both u and v. */
  for(shift = 0; ((u | v) & 1) == 0; ++shift) {
    u >>= 1;
    v >>= 1;
  }
  while((u & 1) == 0) {
    u >>= 1;
  }
  /* From here on, u is always odd. */
  do {
    while((v & 1) == 0) {  /* Loop X */
      v >>= 1;
    }
    /* Now u and v are both odd, so diff(u, v) is even.
       Let u = min(u, v), v = diff(u, v)/2. */
    if(u < v) {
      v -= u;
    } else {
      uint16_t diff = u - v;
      u = v;
      v = diff;
    }
    v >>= 1;
  } while(v != 0);

  return u << shift;
}
/*---------------------------------------------------------------------------*/
/**
 * @brief adapts the communication period T according to the traffic demand
 * @return the new period
 */
static inline uint16_t 
lwb_sched_adapt_period(void)
{  
  if(time < (sched_stats.t_last_req + LWB_CONF_SCHED_T_NO_REQ)) {
    /* we have received stream requests in the last LWB_CONF_SCHED_T_NO_REQ 
     * seconds: set the period to a low value */
    return LWB_CONF_SCHED_PERIOD_MIN;
  }
  if(!data_cnt) {
    return LWB_CONF_SCHED_PERIOD_IDLE;     /* no streams */
  }
  uint16_t new_period = (uint16_t)(((uint32_t)data_ipi * 
                                    LWB_CONF_MAX_DATA_SLOTS) / data_cnt);
  /*new_period = (((uint32_t)data_ipi * LWB_CONF_MAX_DATA_SLOTS) / data_cnt) *
   *             n_pkts / n_slots;*/
  /* check for saturation */
  if(new_period < LWB_CONF_SCHED_PERIOD_MIN) {
    /* T_opt is smaller than LWB_CONF_SCHED_PERIOD_MIN */
    DEBUG_PRINT_WARNING("network saturated!");
    return LWB_CONF_SCHED_PERIOD_MIN;
  }
  /* limit the period */
  if(new_period > LWB_CONF_SCHED_PERIOD_MAX) {
    return LWB_CONF_SCHED_PERIOD_MAX;
  }
  return new_period;
}
/*---------------------------------------------------------------------------*/
/**
 * @brief searches the list node_list/stream_list for the node_id/stream_id
 * @param[in] node_id the node ID to search for
 * @param[in] stream_id the stream ID to search for
 * @param[in] node_list the list of node IDs to search
 * @param[in] stream_list the list of stream IDs to search
 * @param[in] list_len the number of entries in the node and stream list
 * @return one if the node_id/stream_id exists in the list, zero otherwise
 * @remark this search could be optimized as the node ids in the list are 
 * supposed to be sorted in increasing order
 */
static inline uint8_t 
lwb_sched_stream_in_list(uint16_t node_id, 
                         uint8_t stream_id, 
                         const uint16_t* node_list, 
                         const uint8_t* stream_list, 
                         uint8_t list_len) 
{
  while(list_len) {
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
uint16_t 
lwb_sched_compute(lwb_schedule_t * const sched, 
                  const uint8_t * const streams_to_update, 
                  uint8_t reserve_slot_host) 
{  
  static uint16_t slots_tmp[LWB_CONF_MAX_DATA_SLOTS];

  data_ipi = 1;
  data_cnt = 0;
  first_index = 0; 
  n_slots_assigned = 0;
  /*sched->host_id = node_id;  -> not necessary */
  
  /* loop through all the streams in the list */
#if !LWB_CONF_SCHED_USE_XMEM
  memset(streams, 0, sizeof(streams)); /* clear content of the stream list */
  lwb_stream_list_t *curr_stream = list_head(streams_list);
  while(curr_stream != NULL) {
    if(lwb_sched_stream_in_list(curr_stream->node_id, 
                                curr_stream->stream_id, 
                                sched->slot, 
                                streams_to_update, 
                                LWB_SCHED_N_SLOTS(sched))) {
      curr_stream->n_cons_missed = 0;
    } else if(curr_stream->n_cons_missed & 0x80) {
      /* no packet received from this stream */
      curr_stream->n_cons_missed &= 0x7f;              /* clear the last bit */
      curr_stream->n_cons_missed++;
    }
    if(curr_stream->n_cons_missed > LWB_CONF_SCHED_STREAM_REMOVAL_THRES) {
      /* too many consecutive slots without reception: delete this stream */
      lwb_stream_list_t *stream_to_remove = curr_stream;
      curr_stream = curr_stream->next;
      lwb_sched_del_stream(stream_to_remove);
    } else {
      uint16_t curr_gcd = gcd(data_ipi, curr_stream->ipi);
      uint16_t k1 = curr_stream->ipi / curr_gcd;
      uint16_t k2 = data_ipi / curr_gcd;
      data_cnt = data_cnt * k1 + k2;
      data_ipi = data_ipi * k1;
      curr_stream = curr_stream->next;
    }
  }
#else /* LWB_CONF_SCHED_USE_XMEM */
  uint32_t stream_addr = streams_list;
  lwb_stream_list_t curr_stream;
  while(stream_addr != MEMBX_INVALID_ADDR) {
    xmem_read(stream_addr, sizeof(lwb_stream_list_t), (uint8_t*)&curr_stream);
    if(lwb_sched_stream_in_list(curr_stream.node_id, 
                                curr_stream.stream_id, 
                                sched->slot, 
                                streams_to_update, 
                                LWB_SCHED_N_SLOTS(sched))) {
      curr_stream.n_cons_missed = 0;
      xmem_write(stream_addr, sizeof(lwb_stream_list_t), 
                 (uint8_t*)&curr_stream);
    } else if(curr_stream.n_cons_missed & 0x80) {
      curr_stream.n_cons_missed &= 0x7f;
      curr_stream.n_cons_missed++;
      xmem_write(stream_addr, sizeof(lwb_stream_list_t), 
                 (uint8_t*)&curr_stream);
    }
    
    if(curr_stream.n_cons_missed > LWB_CONF_SCHED_STREAM_REMOVAL_THRES) {
      /* too many consecutive slots without reception: delete this stream */
      lwb_sched_del_stream(stream_addr);
    } else {
      uint16_t curr_gcd = gcd(data_ipi, curr_stream.ipi);
      uint16_t k1 = curr_stream.ipi / curr_gcd;
      uint16_t k2 = data_ipi / curr_gcd;
      data_cnt = data_cnt * k1 + k2;
      data_ipi = data_ipi * k1;
    }    
    if(stream_addr == curr_stream.next) {  /* prevent endless loop */
      DEBUG_PRINT_WARNING("unexpected stream address!");
      break;
    }
    stream_addr = curr_stream.next;      
  }
#endif /* LWB_CONF_SCHED_USE_XMEM */

  /* clear content of the schedule (do NOT move this line further above!) */
  memset(sched->slot, 0, sizeof(sched->slot));  
  /* assign slots to the host (max. 1 in this case) */
  if(reserve_slot_host) {
    DEBUG_PRINT_INFO("assigning a slot to the host");
    sched->slot[0] = node_id;
    n_slots_assigned++;
    sched_stats.t_last_req = time;
  }  
  period = lwb_sched_adapt_period();               /* adapt the round period */
  time += period;                    /* increment time by the current period */

  if(n_streams == 0) {  
    goto set_schedule;                              /* no streams to process */
  }
  /* random initial position in the list */
  uint16_t rand_init_pos = (random_rand() >> 1) % n_streams;
  uint16_t i;
  
#if !LWB_CONF_SCHED_USE_XMEM
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
      if(period == LWB_CONF_SCHED_PERIOD_MIN) {   /* if saturated */
        if((curr_stream->next == init_stream) || 
           (curr_stream->next == NULL && rand_init_pos == 0)) {
          /* last random stream: assign all possible slots */
        } else {
          /* ensure fairness among source nodes when the bandwidth saturates
           * assigned a number of slots proportional to (1/IPI) */
          uint16_t slots_ipi = period / curr_stream->ipi;
          if(to_assign > slots_ipi) {
            to_assign = slots_ipi;
            if(to_assign == 0 && curr_stream == init_stream) {
              /* first random stream: assign one slot to it, even if it has 
               * very long IPI */
              to_assign = 1;
            }
          }
        }
      }
      if(to_assign > LWB_CONF_MAX_DATA_SLOTS - n_slots_assigned) {
        to_assign = LWB_CONF_MAX_DATA_SLOTS - n_slots_assigned;
      }
      curr_stream->last_assigned += to_assign * curr_stream->ipi;
      for(; to_assign > 0; to_assign--, n_slots_assigned++) {
        slots_tmp[n_slots_assigned] = curr_stream->node_id;
        streams[n_slots_assigned] = curr_stream;
      }
      /* set the last bit, we are expecting a packet from this stream in the
       * next round */
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

#else /* LWB_CONF_SCHED_USE_XMEM */

  if(streams_list == MEMBX_INVALID_ADDR) {
    DEBUG_PRINT_WARNING("unexpected invalid stream address");
    goto set_schedule;
  }
  stream_addr = streams_list;
  /* make curr_stream point to the random initial position */
  for(i = 0; i < rand_init_pos && stream_addr != MEMBX_INVALID_ADDR; i++) {
    xmem_read(stream_addr, sizeof(lwb_stream_list_t), (uint8_t*)&curr_stream);
    stream_addr = curr_stream.next;
    if(stream_addr == MEMBX_INVALID_ADDR) {
      DEBUG_PRINT_WARNING("unexpected invalid stream address");
      stream_addr = streams_list;
      break;
    }
  }
  uint32_t init_stream = stream_addr;
  do {
    xmem_read(stream_addr, sizeof(lwb_stream_list_t), (uint8_t*)&curr_stream);
    /* assign slots for this stream, if possible */
    if((n_slots_assigned < LWB_CONF_MAX_DATA_SLOTS) && 
       (time >= (curr_stream.ipi + curr_stream.last_assigned))) {
      /* the number of slots to assign to curr_stream */
      uint16_t to_assign = (time - curr_stream.last_assigned) /
                           curr_stream.ipi;  /* elapsed time / period */
      if(period == LWB_CONF_SCHED_PERIOD_MIN) {   /* if saturated */
        if((curr_stream.next == init_stream) || 
           (curr_stream.next == MEMBX_INVALID_ADDR && rand_init_pos == 0)) {
          /* last random stream: assign all possible slots */
        } else {
          /* ensure fairness among source nodes when the bandwidth saturates
           * assigned a number of slots proportional to (1/IPI) */
          uint16_t slots_ipi = period / curr_stream.ipi;
          if(to_assign > slots_ipi) {
            to_assign = slots_ipi;
            if(to_assign == 0 && init_stream == stream_addr) {
              /* first random stream: assign one slot to it, even if it has 
               * very long IPI */
              to_assign = 1;
            }
          }
        }
      }
      if(to_assign > LWB_CONF_MAX_DATA_SLOTS - n_slots_assigned) {
        to_assign = LWB_CONF_MAX_DATA_SLOTS - n_slots_assigned;   /* limit */
      }
      curr_stream.last_assigned += to_assign * curr_stream.ipi;
      for(; to_assign > 0; to_assign--, n_slots_assigned++) {
        slots_tmp[n_slots_assigned] = curr_stream.node_id;
      }
      /* set the last bit, we are expecting a packet from this stream in the 
       * next round and save the changes */
      curr_stream.n_cons_missed |= 0x80;
      xmem_write(stream_addr, sizeof(lwb_stream_list_t), 
                 (uint8_t*)&curr_stream);
    }
    /* go to the next stream in the list */
    stream_addr = curr_stream.next;
    if(stream_addr == MEMBX_INVALID_ADDR) {
      /* end of the list: start again from the head of the list */
      stream_addr = streams_list;
      first_index = n_slots_assigned; 
    }    
  } while(stream_addr != init_stream);
#endif /* LWB_CONF_SCHED_USE_XMEM */
  
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
  //if((time < (sched_stats.t_last_req + LWB_CONF_SCHED_T_NO_REQ)) || 
  //   (time >= (sched_stats.t_last_cont + LWB_CONF_SCHED_T_NO_REQ)) ||
  //   !data_cnt) {    -> always schedule a contention slot
    /* schedule a contention slot */
    sched_stats.t_last_cont = time;
    LWB_SCHED_SET_CONT_SLOT(sched);
  //}  
  
#if LWB_CONF_SCHED_COMPRESS
  uint8_t len = lwb_sched_compress((uint8_t*)sched->slot, n_slots_assigned);
  if((len + LWB_SCHED_PKT_HEADER_LEN) > LWB_CONF_MAX_PKT_LEN) {
    DEBUG_PRINT_ERROR("compressed schedule is too big!");
  }
#else /* LWB_CONF_SCHED_COMPRESS */
  uint8_t len = n_slots_assigned * 2;
#endif /* LWB_CONF_SCHED_COMPRESS */

  /* this schedule is sent at the end of a round: do not communicate 
   * (i.e. do not set the first bit of period) */
  sched->period = period;   /* no need to clear the last bit */
  sched->time   = time;
  /* log the parameters of the new schedule */
  DEBUG_PRINT_INFO("schedule updated (s=%u T=%u n=%u|%u l=%u|%u)", 
                   n_streams, sched->period, n_slots_assigned, 
                   sched->n_slots >> 14, len, n_slots_assigned * 2);
  
  return len + LWB_SCHED_PKT_HEADER_LEN;
}
/*---------------------------------------------------------------------------*/
uint16_t 
lwb_sched_init(lwb_schedule_t* sched) 
{
#if !LWB_CONF_SCHED_USE_XMEM
  memb_init(&streams_memb);
  list_init(streams_list);
#else /* LWB_CONF_SCHED_USE_XMEM */
  membx_init(&streams_memb, xmem_alloc(streams_memb.size * streams_memb.num));
  streams_list = MEMBX_INVALID_ADDR;
#endif /* LWB_CONF_SCHED_USE_XMEM */

  data_ipi = 1;
  data_cnt = 0;
  n_streams = 0;
  n_slots_assigned = 0;
  n_pending_sack = 0;
  time = 0;                                        /* global time starts now */
  period = LWB_CONF_SCHED_PERIOD_IDLE;
  sched->n_slots = 0;                                       /* no data slots */
  LWB_SCHED_SET_CONT_SLOT(sched);               /* include a contention slot */
  sched_stats.t_last_cont = time;
  sched_stats.t_last_req  = -LWB_CONF_SCHED_T_NO_REQ;
  sched->time = time;
  sched->period = period;
  /* mark as first schedule (beginning of a round) */
  LWB_SCHED_SET_AS_1ST(sched); 
  
  DEBUG_PRINT_INFO("min-energy scheduler initialized");
  
  return LWB_SCHED_PKT_HEADER_LEN; /* empty schedule, no slots allocated yet */
}
/*---------------------------------------------------------------------------*/

#endif /* LWB_SCHED_MIN_ENERGY */

/**
 * @}
 * @}
 */
