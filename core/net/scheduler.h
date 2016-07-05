/*
 * Copyright (c) 2016, Swiss Federal Institute of Technology (ETH Zurich).
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
 *          Federico Ferrari
 *          Marco Zimmerling
 */

/**
 * @addtogroup  lwb
 * @{
 *
 * @defgroup    lwb-scheduler Scheduler
 * @{
 *
 * @file 
 *
 * @brief 
 * interface definition for LWB schedulers
 * 
 * This is a generic template for scheduler implementations for the LWB.
 * The scheduler for the LWB runs on the host node only.
 */
 
#ifndef __SCHEDULER_H__
#define __SCHEDULER_H__


#ifndef LWB_CONF_SCHED_T_NO_REQ
/* how long no stream request until period is adjusted accordingly */
#define LWB_CONF_SCHED_T_NO_REQ              LWB_CONF_SCHED_PERIOD_MIN * 2
#endif /* LWB_CONF_SCHED_T_NO_REQ */

#ifndef LWB_CONF_SCHED_COMPRESS
#define LWB_CONF_SCHED_COMPRESS              1
#endif /* LWB_CONF_SCHED_COMPRESS */

/* --- defines for the HOST --- */

#ifndef LWB_CONF_SCHED_SACK_BUFFER_SIZE
/* max. number of processed stream requests per round. Any further requests 
 * will be ignored. Memory usage: 4x LWB_CONF_SCHED_SACK_BUFFER_SIZE bytes */
#define LWB_CONF_SCHED_SACK_BUFFER_SIZE      5       
#endif /* LWB_CONF_SCHED_SACK_BUFFER_SIZE */

#ifndef LWB_CONF_SCHED_USE_XMEM
/* use the external memory (FRAM) to store the stream information? (enable this
 * option if SRAM is too small) */
#define LWB_CONF_SCHED_USE_XMEM              0       
#endif /* LWB_CONF_SCHED_USE_XMEM */

/* SCHEDULER */

#ifndef LWB_CONF_SCHED_PERIOD_MAX
/* max. assignable round period in seconds, must not exceed 2^15 - 1 seconds!*/
#define LWB_CONF_SCHED_PERIOD_MAX            30      
#endif /* LWB_CONF_SCHED_PERIOD_MAX */

#ifndef LWB_CONF_SCHED_PERIOD_MIN
/* minimum round period, must be higher than T_ROUND_MAX */
#define LWB_CONF_SCHED_PERIOD_MIN            2       
#endif /* LWB_CONF_SCHED_PERIOD_MIN */ 

#ifndef LWB_CONF_SCHED_PERIOD_IDLE
/* default period (when no nodes are in the network, or the period for a 
 * static scheduler) */
#define LWB_CONF_SCHED_PERIOD_IDLE           10      
#endif /* LWB_CONF_SCHED_PERIOD_IDLE */

#ifndef LWB_CONF_SCHED_STREAM_REMOVAL_THRES
/* threshold for the stream removal (max. number of 'misses') */
#define LWB_CONF_SCHED_STREAM_REMOVAL_THRES  10      
#endif /* LWB_CONF_SCHED_STREAM_REMOVAL_THRES */

/* define the stream extra data length based on the selected scheduler */
#ifdef LWB_SCHED_MIN_ENERGY
#define LWB_CONF_STREAM_EXTRA_DATA_LEN       1
#else
#define LWB_CONF_STREAM_EXTRA_DATA_LEN       0
#endif
                         
#ifndef LWB_CONF_STREAM_EXTRA_DATA_LEN
#warning "stream info length (LWB_CONF_STREAM_EXTRA_DATA_LEN) not defined!"
/* size in bytes of the additional stream info (extra data) in lwb_stream_t */
#define LWB_CONF_STREAM_EXTRA_DATA_LEN  1                            
#endif /* LWB_CONF_STREAM_EXTRA_DATA_LEN */

                         
/**
 * @brief the structure of a schedule packet
 */
#define LWB_SCHED_PKT_HEADER_LEN    8
typedef struct {    
    uint32_t time;
    uint16_t period;
     /* store num. of data slots and last two bits to indicate whether there is
      * a contention or an s-ack slot in this round */
    uint16_t n_slots;
    uint16_t slot[LWB_CONF_MAX_DATA_SLOTS];
} lwb_schedule_t;

/**
 * @brief minimal meta data required for each stream request
 * @note the stream_info must be the same as the stream_info in the struct 
 * lwb_stream_min_t
 */
#define LWB_STREAM_REQ_HEADER_LEN  6
#define LWB_STREAM_REQ_PKT_LEN     (LWB_STREAM_REQ_HEADER_LEN + \
                                    LWB_CONF_STREAM_EXTRA_DATA_LEN)
typedef struct {
    uint16_t id;            /* ID of this node */
    uint8_t  reserved;      /* padding for alignment to 16-bit
                             * or use #pragma pack(1) [...] #pragma pack() */
    uint8_t  stream_id;     /* stream ID (chosen by the source node) */
    uint16_t ipi;
#if LWB_CONF_STREAM_EXTRA_DATA_LEN
    uint8_t  extra_data[LWB_CONF_STREAM_EXTRA_DATA_LEN];
#endif /* LWB_CONF_STREAM_EXTRA_DATA_LEN */
} lwb_stream_req_t;

#define LWB_SACK_MIN_PKT_LEN       4
typedef struct {                    
    uint16_t id;              
    uint8_t  stream_id;            
    uint8_t  n_extra;   /* number of additional sack's in this packet */
    /* additional sack's */
    uint8_t  extra[LWB_CONF_MAX_PKT_LEN - LWB_SACK_MIN_PKT_LEN];  
} lwb_stream_ack_t;     /* stream acknowledgement */

/* error checking */
#if LWB_CONF_MAX_DATA_SLOTS > \
    ((LWB_CONF_MAX_PKT_LEN - LWB_SCHED_PKT_HEADER_LEN) / 2) || \
    LWB_CONF_MAX_DATA_SLOTS > 63
#error "LWB_CONF_MAX_DATA_SLOTS is invalid"
#endif


/**
 * @brief marks the schedule as the 1st schedule
 */
#define LWB_SCHED_SET_AS_1ST(s)       ((s)->period |= 0x8000)
/**
 * @brief marks the schedule s as the 2nd schedule 
 */
#define LWB_SCHED_SET_AS_2ND(s)       ((s)->period &= ~0x8000)
/**
 * @brief checks whether schedule is the 1st schedule
 */
#define LWB_SCHED_IS_1ST(s)           (((s)->period & 0x8000) > 0)
/**
 * @brief checks whether schedule is the 2nd schedule
 */
#define LWB_SCHED_IS_2ND(s)           (((s)->period & 0x8000) == 0)
/**
 * @brief returns the number of data slots from schedule
 */
#define LWB_SCHED_N_SLOTS(s)          ((s)->n_slots & 0x1fff)
/**
 * @brief checks whether schedule has data slots
 */
#define LWB_SCHED_HAS_DATA_SLOT(s)    (((s)->n_slots & 0x3fff) > 0)
/**
 * @brief checks whether schedule has a contention slot
 */
#define LWB_SCHED_HAS_CONT_SLOT(s)    (((s)->n_slots & 0x4000) > 0)
/**
 * @brief checks whether schedule has an S-ACK slot
 */
#define LWB_SCHED_HAS_SACK_SLOT(s)    (((s)->n_slots & 0x8000) > 0)
/**
 * @brief checks whether schedule has a D-ACK slot
 */
#define LWB_SCHED_HAS_DACK_SLOT(s)    (((s)->n_slots & 0x2000) > 0)
/**
 * @brief marks schedule to have a contention slot
 */
#define LWB_SCHED_SET_CONT_SLOT(s)    ((s)->n_slots |= 0x4000)
/**
 * @brief marks schedule to have an S-ACK slot
 */
#define LWB_SCHED_SET_SACK_SLOT(s)    ((s)->n_slots |= 0x8000)
/**
 * @brief marks schedule to have a D-ACK slot
 */
#define LWB_SCHED_SET_DACK_SLOT(s)    ((s)->n_slots |= 0x2000)


/**
 * @brief prepare a stream acknowledgement (S-ACK) packet
 * @param[out] payload output buffer
 * @return the packet size or zero if there is no S-ACK pending
 */
uint8_t lwb_sched_prepare_sack(void *payload);

/**
 * @brief processes a stream request
 * adds new streams to the stream list, updates stream information for existing
 * streams or removes streams with an invalid IPI
 * @param[in] req the stream request to process
 */
void lwb_sched_proc_srq(const lwb_stream_req_t* req);

/**
 * @brief initializes the schedule
 * resets all the data structures and sets the initial values
 * @param[out] the schedule
 * @return the size of the (empty) schedule
 */
uint16_t lwb_sched_init(lwb_schedule_t* sched);

/**
 * @brief compute (and compress) the new schedule
 * @param[in,out] sched the old schedule and the output buffer for the new 
 * schedule
 * @param[in] streams_to_update the list of streams of the last round 
 * @param[in] reserve_slot_host set this parameter to one to reserve the first
 * slot of the next schedule for the host
 * @return the size of the new (compressed) schedule
 */
uint16_t lwb_sched_compute(lwb_schedule_t * const sched, 
                           const uint8_t * const streams_to_update, 
                           uint8_t n_slot_host);


uint8_t lwb_sched_uncompress(uint8_t* compressed_data, 
                             uint8_t n_slots);


#endif /* __SCHEDULER_H__ */

/**
 * @}
 * @}
 */
