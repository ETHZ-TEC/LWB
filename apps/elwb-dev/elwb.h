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

/* only to include project files files and define global/external variables
 * do not store any configuration in this file */

#ifndef __ELWB_H__
#define __ELWB_H__

/* includes */

#include "contiki.h"


/* --------------- START OF CONFIG (default values) ------------------------ */

#if ELWB_CONF_HEADER_LEN != 0
#error "ELWB_CONF_HEADER_LEN must be set to 0!"
#endif /* ELWB_CONF_HEADER_LEN */

/* expected packet length of a slot request */
#ifndef ELWB_CONF_SRQ_PKT_LEN
#define ELWB_CONF_SRQ_PKT_LEN     2 
#endif /* ELWB_CONF_SRQ_PKT_LEN */


/* set to 1 to directly forward all received messages to BOLT */
#ifndef ELWB_CONF_WRITE_TO_BOLT
#define ELWB_CONF_WRITE_TO_BOLT   0
#endif /* ELWB_CONF_WRITE_TO_BOLT */

/* guard time used for the wakeup before the round (LF timer) */
#ifndef ELWB_CONF_T_GUARD_LF
#define ELWB_CONF_T_GUARD_LF      (ELWB_CONF_T_GUARD / RTIMER_HF_LF_RATIO)
#endif /* ELWB_CONF_T_GUARD_LF */

/* by default, forward all received packets to the app task on the source nodes
 * that originated from the host (or ID 0) */
#ifndef ELWB_CONF_SRC_PKT_FILTER
/* if expression evaluates to 'true', the packet is forwarded/kept (by default
 * keep packets originating from the host (first field in pkt is device ID) */
#define ELWB_CONF_SRC_PKT_FILTER(data)  \
                         (schedule.slot[i] == 0 || schedule.slot[i] == HOST_ID)
#endif /* ELWB_CONF_SRC_PKT_FILTER */

#ifndef ELWB_CONF_MAX_SLOTS_HOST
#define ELWB_CONF_MAX_SLOTS_HOST    (ELWB_CONF_MAX_DATA_SLOTS / 2)
#endif /* ELWB_CONF_MAX_SLOTS_HOST */

/* slack time for schedule computation, in HF ticks */
#ifndef ELWB_CONF_SCHED_COMP_TIME
#define ELWB_CONF_SCHED_COMP_TIME   (RTIMER_SECOND_HF / 50)          /* 20ms */
#endif /* ELWB_CONF_SCHED_COMP_TIME */

#ifndef ELWB_CONF_SCHED_FAIR
#define ELWB_CONF_SCHED_FAIR      1
#endif /* ELWB_CONF_SCHED_FAIR */

#ifndef ELWB_CONF_SCHED_COMPRESS
#define ELWB_CONF_SCHED_COMPRESS  1
#endif /* ELWB_CONF_SCHED_COMPRESS */

#define ELWB_CONF_LF_RTIMER_ID    RTIMER_LF_1
#define ELWB_CONF_RTIMER_ID       RTIMER_HF_1

#ifndef ELWB_CONF_USE_XMEM
#define ELWB_CONF_USE_XMEM        0
#endif /* ELWB_CONF_USE_XMEM */

#ifndef ELWB_CONF_MAX_PKT_LEN
#define ELWB_CONF_MAX_PKT_LEN     126
#endif /* ELWB_CONF_MAX_PKT_LEN */

#ifndef ELWB_CONF_T_SCHED
#define ELWB_CONF_T_SCHED         ELWB_T_SLOT_MIN(ELWB_CONF_MAX_PKT_LEN + \
                                                  GLOSSY_MAX_HEADER_LEN)
#endif /* ELWB_CONF_T_SCHED */
  
#ifndef ELWB_CONF_T_DATA
#define ELWB_CONF_T_DATA          ELWB_T_SLOT_MIN(LWB_CONF_MAX_DATA_PKT_LEN + \
                                                  GLOSSY_MAX_HEADER_LEN)
#endif /* ELWB_CONF_T_DATA */

/* --------------- END OF CONFIG, do not change values below --------------- */

#define ELWB_PERIOD_SCALE       100
#define ELWB_REQ_PKT_LEN        2
#define ELWB_SCHED_HDR_LEN      LWB_SCHED_PKT_HEADER_LEN  /* use same header */
#define ELWB_SCHED_PERIOD_MAX   (65535 / ELWB_PERIOD_SCALE)

/* use LWB_T_HOP() for this macro */
#define ELWB_T_SLOT_MIN(len)    ((ELWB_CONF_MAX_HOPS + \
                                 (2 * ELWB_CONF_TX_CNT_DATA) - 2) * \
                                 LWB_T_HOP(len) + (RTIMER_SECOND_HF / 4000))

#ifndef RF_CONF_MAX_PKT_LEN
#define RF_CONF_MAX_PKT_LEN     (ELWB_CONF_MAX_PKT_LEN + GLOSSY_MAX_HEADER_LEN)
#endif /* RF_CONF_MAX_PKT_LEN */

/*---------------------------------------------------------------------------*/

/* parameter / error check */

#if RF_CONF_MAX_PKT_LEN < (LWB_CONF_MAX_PKT_LEN + GLOSSY_MAX_HEADER_LEN)
#error "LWB_CONF_MAX_PKT_LEN is too big"
#endif

#if !FRAM_CONF_ON && ELWB_CONF_USE_XMEM
#error "FRAM not enabled!"
#endif

#if ELWB_PERIOD_SCALE == 0 || ELWB_PERIOD_SCALE > 1000
#error "invalid ELWB_PERIOD_SCALE"
#endif

#if ELWB_CONF_SCHED_PERIOD_IDLE > ELWB_SCHED_PERIOD_MAX
#error "ELWB_CONF_SCHED_PERIOD_IDLE invalid"
#endif

#if ELWB_CONF_SCHED_PERIOD_IDLE < ELWB_CONF_SCHED_PERIOD_MIN
#error "ELWB_CONF_SCHED_PERIOD_IDLE < ELWB_CONF_SCHED_PERIOD_MIN"
#endif

#if ELWB_CONF_WRITE_TO_BOLT && !BOLT_CONF_ON
#error "ELWB_CONF_WRITE_TO_BOLT requires BOLT to be enabled!"
#endif

#if ELWB_CONF_MAX_N_NODES > ELWB_CONF_MAX_DATA_SLOTS
#error "ELWB_CONF_MAX_N_NODES is invalid"
#endif

#if ELWB_CONF_MAX_SLOTS_HOST > ELWB_CONF_MAX_DATA_SLOTS
#error "ELWB_CONF_MAX_SLOTS_HOST > ELWB_CONF_MAX_DATA_SLOTS!"
#endif

#if ELWB_CONF_SCHED_FAIR
  #if ELWB_CONF_MAX_DATA_SLOTS > 100
  #error "ELWB_CONF_MAX_DATA_SLOTS > 100 not allowed"
  #endif
#endif
/*---------------------------------------------------------------------------*/

/* macros */

#define ELWB_SCHED_N_SLOTS(s)           ((s)->n_slots & 0x1fff)
#define ELWB_SCHED_HAS_SLOTS(s)         (((s)->n_slots & 0x1fff) > 0)

#define ELWB_SCHED_IS_DATA_ROUND(s)     (((s)->n_slots & 0x8000) > 0)
#define ELWB_SCHED_HAS_CONT_SLOT(s)     (((s)->n_slots & 0x4000) > 0)
#define ELWB_SCHED_IS_FIRST(s)          ELWB_SCHED_HAS_CONT_SLOT(s)
#define ELWB_SCHED_IS_STATE_IDLE(s)     (((s)->n_slots & 0x2000) > 0)

#define ELWB_SCHED_SET_CONT_SLOT(s)     ((s)->n_slots |= 0x4000)
#define ELWB_SCHED_SET_DATA_ROUND(s)    ((s)->n_slots |= 0x8000)
#define ELWB_SCHED_SET_STATE_IDLE(s)    ((s)->n_slots |= 0x2000)

/*---------------------------------------------------------------------------*/

/* structs and typedefs */

typedef struct {
  uint8_t  relay_cnt;
  uint8_t  unsynced_cnt;
  uint8_t  bootstrap_cnt;
  uint8_t  sleep_cnt;   /* #times node went into LPM due to rf silence */
  uint8_t  reset_cnt;
  int8_t   glossy_snr;  /* SNR measured in last schedule slot */
  int16_t  drift;
  uint16_t pck_cnt;     /* total number of received packets */
  uint16_t t_sched_max; /* max. time needed to calc new schedule */
  uint16_t t_proc_max;  /* max. time needed to process rcvd data pkts */
  uint32_t t_slot_last; /* last slot assignment (in seconds) */
  uint32_t rx_total;    /* total amount of received bytes (payload) */
  uint16_t rxbuf_drop;  /* packets dropped due to input buffer full */
  uint16_t txbuf_drop;  /* packets dropped due to output buffer full */
  uint32_t pkts_nack;   /* not acknowledged data packets */
  uint32_t pkts_sent;   /* total number of sent data packets */
  uint16_t load;        /* bandwidth utilization */
} elwb_stats_t;

/* use the same schedule struct as defined in scheduler.h */
typedef lwb_schedule_t elwb_schedule_t;

/*---------------------------------------------------------------------------*/

/* global variables */


/*---------------------------------------------------------------------------*/

/* function prototypes */

void     elwb_start(struct process *pre_elwb_proc, 
                    struct process *post_elwb_proc);
uint8_t  elwb_send_pkt(const uint8_t* const data,
                       uint8_t len);
uint8_t  elwb_rcv_pkt(uint8_t* out_data);
uint8_t  elwb_get_rcv_buffer_state(void);
uint8_t  elwb_get_send_buffer_state(void);
uint32_t elwb_get_time(rtimer_clock_t* reception_time);
uint64_t elwb_get_timestamp(void);
const elwb_stats_t * const elwb_get_stats(void);

uint16_t elwb_sched_init(elwb_schedule_t* sched);
void     elwb_sched_process_req(uint16_t node_id, 
                                uint8_t n_pkts);
uint16_t elwb_sched_compute(elwb_schedule_t * const sched,
                            uint8_t reserve_slots_host);
uint16_t elwb_sched_get_period(void);
void     elwb_sched_set_period(uint16_t p);
uint32_t elwb_sched_get_time(void);
void     elwb_sched_set_time(uint32_t new_time);

/*---------------------------------------------------------------------------*/

#endif /* __ELWB_H__ */
