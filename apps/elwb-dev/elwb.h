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

#ifndef __ELWB_H__
#define __ELWB_H__


#include "contiki.h"


/* --------------- START OF CONFIG (default values) ------------------------ */

/* max. packet length */
#ifndef ELWB_CONF_MAX_PKT_LEN
#define ELWB_CONF_MAX_PKT_LEN     126
#endif /* ELWB_CONF_MAX_PKT_LEN */

/* use the external memory to store the input/output queue? (requires FRAM) */
#ifndef ELWB_CONF_USE_XMEM
#define ELWB_CONF_USE_XMEM        0
#endif /* ELWB_CONF_USE_XMEM */

/* set to 1 to directly forward all received messages to BOLT */
#ifndef ELWB_CONF_WRITE_TO_BOLT
#define ELWB_CONF_WRITE_TO_BOLT   0
#endif /* ELWB_CONF_WRITE_TO_BOLT */

/* by default, forward all received packets to the app task on the source nodes
 * that originated from the host (or ID 0) */
#ifndef ELWB_CONF_SRC_PKT_FILTER
/* if expression evaluates to 'true', the packet is forwarded/kept (by default
 * keep packets originating from the host (first field in pkt is device ID) */
#define ELWB_CONF_SRC_PKT_FILTER(data)  \
                         (schedule.slot[i] == 0 || schedule.slot[i] == HOST_ID)
#endif /* ELWB_CONF_SRC_PKT_FILTER */

#ifndef ELWB_CONF_MAX_SLOTS_HOST
#define ELWB_CONF_MAX_SLOTS_HOST  (ELWB_CONF_MAX_DATA_SLOTS / 2)
#endif /* ELWB_CONF_MAX_SLOTS_HOST */

/* duration of a schedule slot in HF ticks, if not defined, the minimum
 * required slot time is calculated based on the network N_HOPS and N_TX */
#ifndef ELWB_CONF_T_SCHED
#define ELWB_CONF_T_SCHED         ELWB_T_SLOT_MIN(ELWB_CONF_MAX_PKT_LEN + \
                                                  GLOSSY_MAX_HEADER_LEN)
#endif /* ELWB_CONF_T_SCHED */

/* duration of a data slot in HF ticks, if not defined, the minimum required
 * slot time is calculated based on the network N_HOPS and N_TX */
#ifndef ELWB_CONF_T_DATA
#define ELWB_CONF_T_DATA          ELWB_T_SLOT_MIN(ELWB_CONF_MAX_PKT_LEN + \
                                                  GLOSSY_MAX_HEADER_LEN)
#endif /* ELWB_CONF_T_DATA */

/* duration of a contention slot in HF ticks */
#ifndef ELWB_CONF_T_CONT
#define ELWB_CONF_T_CONT          (RTIMER_SECOND_HF / 200)            /* 5ms */
#endif /* ELWB_CONF_T_CONT */

/* duration of a data ACK slot in HF ticks */
#ifndef ELWB_CONF_T_DACK
#define ELWB_CONF_T_DACK          (ELWB_CONF_T_CONT * 2)
#endif /* ELWB_CONF_T_DACK */

/* gap time between 2 slots in HF ticks */
#ifndef ELWB_CONF_T_GAP
#define ELWB_CONF_T_GAP           (RTIMER_SECOND_HF / 500)            /* 2ms */
#endif /* ELWB_CONF_T_GAP */

/* guard time before RX slots in HF ticks */
#ifndef ELWB_CONF_T_GUARD
#define ELWB_CONF_T_GUARD         (RTIMER_SECOND_HF / 4000)        /* 0.25ms */
#endif /* ELWB_CONF_T_GUARD */

/* guard time before a round in LF ticks */
#ifndef ELWB_CONF_T_GUARD_LF
#define ELWB_CONF_T_GUARD_LF      (RTIMER_SECOND_LF / 1000)           /* 1ms */
#endif /* ELWB_CONF_T_GUARD_LF */

/* time reserved for the preprocess task (before a round) in LF ticks */
#ifndef ELWB_CONF_T_PREPROCESS_LF
#define ELWB_CONF_T_PREPROCESS_LF 0                          /* 0 = disabled */
#endif /* ELWB_CONF_T_PREPROCESS_LF */

/* slack time for schedule computation, in HF ticks */
#ifndef ELWB_CONF_SCHED_COMP_TIME
#define ELWB_CONF_SCHED_COMP_TIME (RTIMER_SECOND_HF / 50)            /* 20ms */
#endif /* ELWB_CONF_SCHED_COMP_TIME */

/* use a 'fair' scheduler which tries to assign slots to all nodes */
#ifndef ELWB_CONF_SCHED_FAIR
#define ELWB_CONF_SCHED_FAIR      1
#endif /* ELWB_CONF_SCHED_FAIR */

/* compress the schedule? */
#ifndef ELWB_CONF_SCHED_COMPRESS
#define ELWB_CONF_SCHED_COMPRESS  1
#endif /* ELWB_CONF_SCHED_COMPRESS */

/* append CRC to the schedule? */
#ifndef ELWB_CONF_SCHED_CRC
#define ELWB_CONF_SCHED_CRC       0
#endif /* ELWB_CONF_SCHED_ADD_CRC */

/* allow preemption of the eLWB task, which runs in an interrupt context
 * (-> enables interrupt nesting) */
#ifndef ELWB_CONF_PREEMPTION
#define ELWB_CONF_PREEMPTION      0
#endif /* ELWB_CONF_PREEMPTION */

/* timers to use for the eLWB task */
#define ELWB_CONF_LF_RTIMER_ID    RTIMER_LF_1
#define ELWB_CONF_RTIMER_ID       RTIMER_HF_1

/* --------------- END OF CONFIG, do not change values below --------------- */

#define ELWB_PERIOD_SCALE         100
#define ELWB_REQ_PKT_LEN          2
#define ELWB_SCHED_CRC_LEN        (ELWB_CONF_SCHED_CRC ? 2 : 0)
#define ELWB_SCHED_PERIOD_MAX     (65535 / ELWB_PERIOD_SCALE)

#if GLOSSY_CONF_SETUPTIME_WITH_SYNC
#define ELWB_T_REF_OFS            ((GLOSSY_CONF_SETUPTIME_WITH_SYNC + 350) * RTIMER_SECOND_HF / 1000000)
#else  /* GLOSSY_CONF_SETUPTIME_WITH_SYNC */
#define ELWB_T_REF_OFS            3822 /* measured with logic analyzer */
#endif /* GLOSSY_CONF_SETUPTIME_WITH_SYNC */

#define ELWB_T_HOP(len)           ((RTIMER_SECOND_HF * \
                                   (3 + 24 + 192 + 192 + ((1000000 * \
                                   (len) * 8) / RF_CONF_TX_BITRATE))) \
                                    / 1000000)
#define ELWB_T_SLOT_MIN(len)      ((ELWB_CONF_N_HOPS + \
                                   (2 * ELWB_CONF_N_TX_DATA) - 2) * \
                                   ELWB_T_HOP(len) + (RTIMER_SECOND_HF / 4000))

#ifndef RF_CONF_MAX_PKT_LEN
#define RF_CONF_MAX_PKT_LEN       (ELWB_CONF_MAX_PKT_LEN + \
                                   GLOSSY_MAX_HEADER_LEN)
#endif /* RF_CONF_MAX_PKT_LEN */

/*---------------------------------------------------------------------------*/

/* parameter sanity checks */

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

#if (ELWB_CONF_MAX_DATA_SLOTS * 2 + ELWB_SCHED_HDR_LEN) > ELWB_CONF_MAX_PKT_LEN
#error "ELWB_CONF_MAX_DATA_SLOTS exceeds the packet size limit"
#endif

#if ELWB_CONF_MAX_SLOTS_HOST > ELWB_CONF_MAX_DATA_SLOTS
#error "ELWB_CONF_MAX_SLOTS_HOST > ELWB_CONF_MAX_DATA_SLOTS!"
#endif

#if ELWB_CONF_SCHED_FAIR
  /* make sure #slots is <= 100 to prevent an overflow in the calculations */
  #if ELWB_CONF_MAX_DATA_SLOTS > 100
  #error "ELWB_CONF_MAX_DATA_SLOTS > 100 not allowed"
  #endif
#endif

/*---------------------------------------------------------------------------*/

/* macros */

#define ELWB_SCHED_N_SLOTS(s)           ((s)->n_slots & 0x1fff)
#define ELWB_SCHED_CLR_SLOTS(s)         ((s)->n_slots &= ~0x1fff)
#define ELWB_SCHED_HAS_SLOTS(s)         (((s)->n_slots & 0x1fff) > 0)

#define ELWB_SCHED_HAS_DATA_SLOTS(s)    (((s)->n_slots & 0x8000) > 0)
#define ELWB_SCHED_HAS_CONT_SLOT(s)     (((s)->n_slots & 0x4000) > 0)
#define ELWB_SCHED_IS_FIRST(s)          ELWB_SCHED_HAS_CONT_SLOT(s)
#define ELWB_SCHED_IS_STATE_IDLE(s)     (((s)->n_slots & 0x2000) > 0)

#define ELWB_SCHED_SET_CONT_SLOT(s)     ((s)->n_slots |= 0x4000)
#define ELWB_SCHED_SET_DATA_SLOTS(s)    ((s)->n_slots |= 0x8000)
#define ELWB_SCHED_SET_STATE_IDLE(s)    ((s)->n_slots |= 0x2000)

/*---------------------------------------------------------------------------*/

/* structs and typedefs */

#define ELWB_STATS_LEN  26
typedef struct {
  uint8_t  bootstrap_cnt;  /* #times bootstrap state was entered */
  uint8_t  unsynced_cnt;   /* #times a schedule was missed */
  uint8_t  sleep_cnt;   /* #times node went into LPM due to rf silence */
  uint8_t  relay_cnt;   /* relay counter of first received packet */
  int16_t  glossy_snr;  /* SNR measured in last schedule slot */
  uint16_t glossy_t_to_rx; /* time from glossy_start() to packet start in ms */
  uint16_t glossy_t_flood; /* flood duration in ms */
  uint8_t  glossy_n_tx; /* # packet retransmissions */
  uint8_t  glossy_n_rx; /* # packet receptions */
  int16_t  drift;       /* current estimated drift in ppm */
  uint16_t pkt_rcv;     /* total number of received packets */
  uint16_t pkt_snd;     /* total number of sent data packets */
  uint16_t pkt_ack;     /* not acknowledged data packets */
  uint16_t pkt_fwd;     /* packets forwarded to the App task */
  uint16_t rxbuf_drop;  /* packets dropped due to input buffer full */
  uint16_t txbuf_drop;  /* packets dropped due to output buffer full */
  uint16_t load;        /* bandwidth utilization */
} elwb_stats_t;

#define ELWB_SCHED_HDR_LEN   8
#define ELWB_SCHED_MAX_SLOTS ((ELWB_CONF_MAX_PKT_LEN - ELWB_SCHED_HDR_LEN - \
                               ELWB_SCHED_CRC_LEN) / 2)
/* note: ELWB_SCHED_MAX_SLOTS != ELWB_CONF_MAX_DATA_SLOTS */
typedef struct {
  uint32_t time;
  uint16_t period;
    /* store num. of data slots and last two bits to indicate whether there is
    * a contention or an s-ack slot in this round */
  uint16_t n_slots;
  uint16_t slot[ELWB_SCHED_MAX_SLOTS + ELWB_SCHED_CRC_LEN];
} elwb_schedule_t;

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
void     elwb_sched_register_nodes(void);
void     elwb_sched_process_req(uint16_t id, 
                                uint8_t n_pkts);
uint16_t elwb_sched_compute(elwb_schedule_t * const sched,
                            uint8_t reserve_slots_host);
uint8_t  elwb_sched_uncompress(uint8_t* compressed_data, uint8_t n_slots);
uint16_t elwb_sched_get_period(void);
void     elwb_sched_set_period(uint16_t p);
uint32_t elwb_sched_get_time(void);
void     elwb_sched_set_time(uint32_t new_time);

/*---------------------------------------------------------------------------*/

#endif /* __ELWB_H__ */
