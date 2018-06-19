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
 */

/**
 * @addtogroup  lwb-scheduler
 * @{
 *
 * @brief
 * a simple scheduler implementation for the eLWB
 */
 
#include "main.h"


/* macros for time unit conversions */
#define HFTICKS_TO_SCHEDUNITS(x) ((x) / (RTIMER_SECOND_HF / ELWB_PERIOD_SCALE))
#define SCHEDUNITS_TO_MS(x)      ((x) * (1000 / ELWB_PERIOD_SCALE))

/* earliest start (offset) of the request round, + 10ms slack (necessary due
 * to rounding issue) */
#define ELWB_T_IDLE_ROUND     (HFTICKS_TO_SCHEDUNITS(ELWB_CONF_T_SCHED + \
                                2 * ELWB_CONF_T_CONT + 2 * ELWB_CONF_T_GAP +\
                                ELWB_CONF_SCHED_COMP_TIME))

/* duration of the 'request round' = start of the data round,
 * depends on the max. # nodes (= ELWB_CONF_MAX_N_NODES), + add 10ms slack */
#define ELWB_T_REQ_ROUND_MAX  (HFTICKS_TO_SCHEDUNITS( \
                                ELWB_CONF_T_SCHED + ELWB_CONF_T_GAP + \
                                ELWB_CONF_MAX_N_NODES * \
                                 (ELWB_CONF_T_CONT + ELWB_CONF_T_GAP) + \
                                ELWB_CONF_SCHED_COMP_TIME))

#define ELWB_T_DATA_ROUND_MAX (HFTICKS_TO_SCHEDUNITS( \
                                ELWB_CONF_T_SCHED + ELWB_CONF_T_GAP + \
                                ELWB_CONF_MAX_DATA_SLOTS * \
                                 (ELWB_CONF_T_DATA + ELWB_CONF_T_GAP)))

/* note: round up for the following values */
#define ELWB_T_ROUND_MAX      (ELWB_T_IDLE_ROUND + ELWB_T_REQ_ROUND_MAX + \
                               ELWB_T_DATA_ROUND_MAX + \
                               HFTICKS_TO_SCHEDUNITS(2 * \
                                ELWB_CONF_SCHED_COMP_TIME))

/*---------------------------------------------------------------------------*/
#define elwb_sched_compress(data, slot_cnt)  lwb_sched_compress(data, slot_cnt)
/*---------------------------------------------------------------------------*/
/**
 * @brief struct to store information about active nodes on the host
 */
typedef struct elwb_node_info {
  struct elwb_node_info *next;      /* linked list */
  uint16_t               id;        /* node ID */
  uint16_t               n_pkts;    /* bandwidth demand in number of packets */
  /* note: use 16 bits due to alignment */
  uint32_t               t_last_req;/* time of the last request */
} elwb_node_list_t;
/*---------------------------------------------------------------------------*/
typedef enum {
  ELWB_SCHED_STATE_IDLE = 0,
  ELWB_SCHED_STATE_CONT_DET,
  ELWB_SCHED_STATE_REQ,
  ELWB_SCHED_STATE_DATA,
} elwb_sched_state_t;
/*---------------------------------------------------------------------------*/
static uint64_t           time;                               /* global time */
static uint16_t           period;           /* base (idle) period in seconds */
static uint16_t           n_nodes;                         /* # active nodes */
static elwb_sched_state_t sched_state;
LIST(nodes_list);                            /* linked list for active nodes */
elwb_node_list_t nodes_mem[ELWB_CONF_MAX_N_NODES];
/*---------------------------------------------------------------------------*/
uint16_t 
uint16_to_str(uint16_t val, char* out_buffer)
{
  uint16_t div = 10000;
  uint16_t num = 0;
  while(div) {
    uint16_t digit = val / div;
    if(num || digit || div == 1) { /* skip leading zeros */
      *out_buffer++ = '0' + digit; /* write the digit into the output buffer */
      val -= digit * div;          /* subtract the most significant digit */
      num++;
    }
    div /= 10;
  }
  *out_buffer = 0;                 /* close the string */
  return num;                      /* return the # written characters */
}
/*---------------------------------------------------------------------------*/
void
elwb_sched_add_node(uint16_t id)
{
  if(id == 0) {
    return;     /* invalid argument */
  }
  if(n_nodes >= ELWB_CONF_MAX_N_NODES) {
    DEBUG_PRINT_WARNING("request from node %u ignored, max #nodes reached",
                        id);
    EVENT_WARNING(EVENT_CC430_NODE_REMOVED, id);
    return;
  }
  elwb_node_list_t* node = 0;
  uint16_t i;
  /* find a free spot */
  for(i = 0; i < ELWB_CONF_MAX_N_NODES; i++) {
    if(nodes_mem[i].id == 0) {
      node = &nodes_mem[i];   /* use this spot */
      break;
    }
  }
  if(node == 0) {
    /* this will never happen (if there's no bug in the code) */
    DEBUG_PRINT_ERROR("out of memory: request dropped");
    return;
  }
  node->id         = id;
  node->n_pkts     = 0;
  node->t_last_req = (time / ELWB_PERIOD_SCALE);
  /* insert the node into the list, ordered by node id */
  elwb_node_list_t *prev;
  for(prev = list_head(nodes_list); prev != 0; prev = prev->next) {
    if((id >= prev->id) && ((prev->next == 0) || 
       (id < prev->next->id))) {
      break;
    }
  }
  list_insert(nodes_list, prev, node);
  n_nodes++;
  DEBUG_PRINT_INFO("node %u registered", id);
  EVENT_INFO(EVENT_CC430_NODE_ADDED, id);
}
/*---------------------------------------------------------------------------*/
void
elwb_sched_remove_node(elwb_node_list_t* node)
{
  if(!node) {
    return;
  }
  DEBUG_PRINT_INFO("node %u removed", node->id);
  EVENT_INFO(EVENT_CC430_NODE_REMOVED, node->id);
  node->id = 0;   /* mark as 'unused' by setting the ID to zero */
  list_remove(nodes_list, node);
}
/*---------------------------------------------------------------------------*/
void 
elwb_sched_process_req(uint16_t node,
                       uint8_t n_pkts) 
{
  if(n_pkts > ELWB_CONF_MAX_DATA_SLOTS) {
    n_pkts = ELWB_CONF_MAX_DATA_SLOTS;      /* cap */
  }
  elwb_node_list_t *s = 0;
  /* check if node already exists */
  for(s = list_head(nodes_list); s != 0; s = s->next) {
    if(node == s->id) {
      s->n_pkts = n_pkts;
      s->t_last_req = (time / ELWB_PERIOD_SCALE);
      return;
    }
  }
  /* does not exist: add the node */
  elwb_sched_add_node(node);
}
/*---------------------------------------------------------------------------*/
uint16_t 
elwb_sched_compute(elwb_schedule_t * const sched,
                   uint8_t reserve_slots_host) 
{ 
  static uint16_t t_round   = 0;
  uint16_t n_slots_assigned = 0;

  /*
   * note: the schedule is sent at the beginning of the next round,
   * i.e. it must include the next period
   */
  if(sched_state == ELWB_SCHED_STATE_IDLE && sched->period == 0) {
    /* request detected! prepare 2nd schedule (just update the period) */
    sched->n_slots = 0;
    sched->period = t_round;                   /* use current round duration */
    sched_state = ELWB_SCHED_STATE_CONT_DET;                 /* change state */
    return 2;                            /* return value doesn't matter here */
  }
  /* use the period of the last round to update the network time */
  time += sched->period;
  
  if(sched_state == ELWB_SCHED_STATE_CONT_DET) {
    /* contention has been detected, now schedule request slots for all 
     * registered nodes in the network */
    
    memset(sched->slot, 0, sizeof(sched->slot));        /* clear the content */
    /* every node gets one slot (a chance to request slots) */
    elwb_node_list_t *curr_node = list_head(nodes_list);
    while(curr_node != 0) {
      sched->slot[n_slots_assigned++] = curr_node->id;
      curr_node = curr_node->next;
    }
    if(n_slots_assigned == 0) {
      /* if there are no registered nodes, then add a dummy slot */
      sched->slot[n_slots_assigned++] = 0xffff;
    }
    sched->n_slots = n_slots_assigned;
    /* calculate next round period (or use ELWB_PERIOD_T_DATA_MAX) */
    sched->period  = ((ELWB_CONF_T_SCHED + ELWB_CONF_T_GAP + 
                      n_slots_assigned * (ELWB_CONF_T_CONT + ELWB_CONF_T_GAP) + 
                      ELWB_CONF_SCHED_COMP_TIME) / 
                     (RTIMER_SECOND_HF / ELWB_PERIOD_SCALE));
    t_round += sched->period;
    
    sched_state = ELWB_SCHED_STATE_REQ;
    
  } else if(sched_state == ELWB_SCHED_STATE_REQ) {
    /* a request round has just finished, now calculate the schedule based on
     * the received requests */
    
    memset(sched->slot, 0, sizeof(sched->slot));        /* clear the content */
    uint16_t node_cnt = 0;
    
    /* first, go through the list of nodes and calculate the traffic demand */
    elwb_node_list_t *curr_node = list_head(nodes_list);
    uint16_t req_slots = 0;
    while(curr_node != 0) {
      if(curr_node->n_pkts) {
        node_cnt++;
        req_slots += curr_node->n_pkts;
      }
      curr_node = curr_node->next;
    }
  #if ELWB_CONF_SCHED_FAIR
    /* if total demand exceeds the avail. bandwidth, then scale each request */
    if(req_slots > ELWB_CONF_MAX_DATA_SLOTS) {
      /* note: don't use floating point calculations here, takes up a lot of
       * flash memory space! */
      /* assumption: n_pkts < 100 and ELWB_CONF_MAX_DATA_SLOTS < 100 */
      uint16_t scale = ELWB_CONF_MAX_DATA_SLOTS * 256 / req_slots;
      /* go again through the list of nodes and assign slots in a 'fair' way */
      curr_node = list_head(nodes_list);
      while(curr_node != 0 && 
            (n_slots_assigned < ELWB_CONF_MAX_DATA_SLOTS)) {
        if(curr_node->n_pkts) {
          uint16_t n = (scale * curr_node->n_pkts + 128) >> 8;
          while(n && (n_slots_assigned < ELWB_CONF_MAX_DATA_SLOTS)) {
            sched->slot[n_slots_assigned++] = curr_node->id;
            n--;
          }
        }
        curr_node = curr_node->next;
      }
    } else 
  #endif /* ELWB_CONF_SCHED_FAIR */
    {
      /* go again through the list of nodes and assign the requested slots */
      curr_node = list_head(nodes_list);
      while(curr_node != 0 && (n_slots_assigned < ELWB_CONF_MAX_DATA_SLOTS)) {
        /* assign as many slots as the node requested */
        while(curr_node->n_pkts &&
              (n_slots_assigned < ELWB_CONF_MAX_DATA_SLOTS)) {
          sched->slot[n_slots_assigned++] = curr_node->id;
          curr_node->n_pkts--;
        }
        curr_node = curr_node->next;
      }
    }
    DEBUG_PRINT_INFO("%u of %u nodes requested slots, %u assigned", 
                     node_cnt, n_nodes, n_slots_assigned);

    sched->n_slots = n_slots_assigned;
    sched->period  = period - t_round;
    t_round += (n_slots_assigned * (ELWB_CONF_T_CONT + ELWB_CONF_T_GAP) + 
                ELWB_CONF_SCHED_COMP_TIME) / 
                (RTIMER_SECOND_HF / ELWB_PERIOD_SCALE);
    ELWB_SCHED_SET_DATA_ROUND(sched); /* mark the next round as 'data round' */
    ELWB_SCHED_SET_STATE_IDLE(sched);     /* mark as 'idle' after this round */
    
    sched_state = ELWB_SCHED_STATE_DATA;
    
  } else if(sched_state == ELWB_SCHED_STATE_DATA) {
    /* a data round has just finished */
    
    memset(sched->slot, 0, sizeof(sched->slot));        /* clear the content */
    /* reset all requests (set n_pkts to 0) */
    elwb_node_list_t* curr_node = list_head(nodes_list);
    DEBUG_PRINT_INFO("registered nodes:");
    uint32_t time_s = (time / ELWB_PERIOD_SCALE);
    while(curr_node != 0) {
      /* remove old nodes */
      if((time_s - curr_node->t_last_req) > ELWB_CONF_SCHED_NODE_TIMEOUT) {
        elwb_node_list_t* next = curr_node->next;
        elwb_sched_remove_node(curr_node);
        curr_node = next;
      } else {
        /* convert the node ID to a string and write it into the debug print
         * buffer */
        char buffer[6];       /* note IDs are 5 characters at most (16 bits) */
        uint16_t num_chars = uint16_to_str(curr_node->id, buffer);
        buffer[num_chars++] = ' ';
        buffer[num_chars] = 0;
        debug_print_buffer_put(buffer);
        curr_node->n_pkts = 0;  /* reset */
        curr_node = curr_node->next;
      }
    }
    debug_print_buffer_put("\r\n");
    DEBUG_PRINT_VERBOSE("round duration: %u0ms", t_round);
    
    sched_state = ELWB_SCHED_STATE_IDLE;
    /* schedule for next round will be set below */
  }
  
  if(sched_state == ELWB_SCHED_STATE_IDLE) {
    /* regular idle round */
    /* add slots for the host if requested */
    while(reserve_slots_host && n_slots_assigned < ELWB_CONF_MAX_SLOTS_HOST) {
      sched->slot[n_slots_assigned++] = node_id;
      reserve_slots_host--;
    }
    sched->n_slots = n_slots_assigned;
    /* calculate round duration */
    t_round = ELWB_T_IDLE_ROUND + (n_slots_assigned * 
                                      (ELWB_CONF_T_DATA + ELWB_CONF_T_GAP) /
                                      (RTIMER_SECOND_HF / ELWB_PERIOD_SCALE));
    sched->period = period;   /* assume idle period for next round */
    if(n_slots_assigned) {
      ELWB_SCHED_SET_DATA_ROUND(sched);
    }
    ELWB_SCHED_SET_CONT_SLOT(sched);
    ELWB_SCHED_SET_STATE_IDLE(sched);  /* will be used by source nodes */
  }
  
  uint8_t compressed_size;
#if ELWB_CONF_SCHED_COMPRESS
  compressed_size = elwb_sched_compress((uint8_t*)sched->slot, 
                                       n_slots_assigned);
  if((compressed_size + ELWB_SCHED_HDR_LEN) > ELWB_CONF_MAX_PKT_LEN) {
    DEBUG_PRINT_ERROR("compressed schedule is too big!");
  }
#else
  compressed_size = n_slots_assigned * 2;
#endif /* ELWB_CONF_SCHED_COMPRESS */
  
  /* increment the timestamp */
  sched->time = time / ELWB_PERIOD_SCALE;
     
  /* log the parameters of the new schedule */
  DEBUG_PRINT_VERBOSE("schedule updated (s=%u T=%u0 n=%u|%u l=%u)", 
                      n_nodes, sched->period, 
                      n_slots_assigned, sched->n_slots >> 13, 
                      compressed_size);
  
  return compressed_size + ELWB_SCHED_HDR_LEN;
}
/*---------------------------------------------------------------------------*/
uint16_t 
elwb_sched_init(elwb_schedule_t* sched) 
{
  printf(" round [ms]: T=%u000 idle=%u req=%u data=%u sum=%u\r\n", 
         ELWB_CONF_SCHED_PERIOD_IDLE, 
         (uint16_t)SCHEDUNITS_TO_MS(ELWB_T_IDLE_ROUND),
         (uint16_t)SCHEDUNITS_TO_MS(ELWB_T_REQ_ROUND_MAX),
         (uint16_t)SCHEDUNITS_TO_MS(ELWB_T_DATA_ROUND_MAX),
         (uint16_t)SCHEDUNITS_TO_MS(ELWB_T_ROUND_MAX));
  
  /* make sure the minimal round period is not smaller than the max. round
   * duration! */
  if(((uint32_t)ELWB_CONF_SCHED_PERIOD_MIN * 1000) <= 
     (SCHEDUNITS_TO_MS(ELWB_T_ROUND_MAX) + 
      ((uint32_t)RTIMER_LF_TO_MS(ELWB_CONF_T_PREPROCESS_LF)))) {
    printf("ERROR: invalid parameters for eLWB scheduler\r\n");
    while(1);
  }
  /* initialize node list */
  list_init(nodes_list);
  memset(nodes_mem, 0, sizeof(elwb_node_list_t) * ELWB_CONF_MAX_N_NODES);
  n_nodes        = 0;
  time           = 0;
  period         = ELWB_CONF_SCHED_PERIOD_IDLE * ELWB_PERIOD_SCALE;
  sched_state    = ELWB_SCHED_STATE_IDLE;
  sched->n_slots = 0;
  sched->time    = 0;
  sched->period  = period;
  ELWB_SCHED_SET_CONT_SLOT(sched);              /* include a contention slot */
  ELWB_SCHED_SET_STATE_IDLE(sched);
  
  /* NOTE: node IDs must be sorted in increasing order */
#if defined(ELWB_CONF_SCHED_AE_SRC_NODE_CNT) && ELWB_CONF_SCHED_AE_SRC_NODE_CNT
  uint16_t node_ids[ELWB_CONF_SCHED_AE_SRC_NODE_CNT] = 
                                          { ELWB_CONF_SCHED_AE_SRC_NODE_LIST };
 #if ELWB_CONF_SCHED_AE_SRC_NODE_CNT > ELWB_CONF_MAX_N_NODES
 #error "ELWB_CONF_SCHED_AE_SRC_NODE_CNT is too high"
 #endif /* ELWB_CONF_SCHED_AE_SRC_NODE_CNT */
  uint16_t i;
  elwb_node_list_t *prev = 0;
  printf(" %u source nodes registered: ", ELWB_CONF_SCHED_AE_SRC_NODE_CNT);
  for(i = 0; i < ELWB_CONF_SCHED_AE_SRC_NODE_CNT; i++) {
    elwb_sched_add_node(node_ids[i]);
    printf("%u", node_ids[i]);
  }
  printf("\r\n");
#endif /* ELWB_CONF_SCHED_AE_INIT_NODES */
  
  return ELWB_SCHED_HDR_LEN;/* empty schedule, no slots allocated yet */
}
/*---------------------------------------------------------------------------*/
uint16_t 
elwb_sched_get_period(void)
{
  /* period in seconds */
  return period / ELWB_PERIOD_SCALE;
}
/*---------------------------------------------------------------------------*/
void 
elwb_sched_set_period(uint16_t p)
{
  if (p >= ELWB_CONF_SCHED_PERIOD_MIN && p <= ELWB_SCHED_PERIOD_MAX) {
    period = (uint16_t)((uint32_t)p * ELWB_PERIOD_SCALE);
  }
}
/*---------------------------------------------------------------------------*/
uint32_t
elwb_sched_get_time(void)
{
  /* network time in seconds */
  return (time / ELWB_PERIOD_SCALE);
}
/*---------------------------------------------------------------------------*/
void
elwb_sched_set_time(uint32_t new_time)
{
  time = (uint64_t)new_time * ELWB_PERIOD_SCALE;
}
/*---------------------------------------------------------------------------*/

/**
 * @}
 * @}
 */
