
/**
 * @file
 * @ingroup LWB
 * @brief   scheduler/stream manager for the LWB, runs on the host node only
 * @author  fe
 * @author  rdaforno
 */
 
#ifndef __SCHEDULER_H__
#define __SCHEDULER_H__

#include "contiki.h"


#ifndef LWB_SCHED_T_NO_REQ
#define LWB_SCHED_T_NO_REQ              LWB_SCHED_PERIOD_MIN * 2          /* how long no stream request until period is adjusted accordingly */
#endif /* LWB_SCHED_T_NO_REQ */

#ifndef LWB_SCHED_COMPRESS
#define LWB_SCHED_COMPRESS              1
#endif /* LWB_SCHED_COMPRESS */

/* --- defines for the HOST --- */

#ifndef LWB_SCHED_SACK_BUFFER_SIZE
#define LWB_SCHED_SACK_BUFFER_SIZE      5       /* max. number of accepted stream acks per round. Any further requests will be ignored. Memory usage: 4x N_PENDING_SACK_MAX bytes */
#endif /* LWB_SCHED_SACK_BUFFER_SIZE */

#ifndef LWB_SCHED_USE_XMEM
#define LWB_SCHED_USE_XMEM              0       /* use the external memory (FRAM) to store the stream information? (enable this option if SRAM is too small) */
#endif /* LWB_SCHED_USE_XMEM */

// SCHEDULER

#ifndef LWB_SCHED_PERIOD_MAX
#define LWB_SCHED_PERIOD_MAX            30      // max. assignable round period in seconds, must not exceed 127 seconds!
#endif /* LWB_SCHED_PERIOD_MAX */

#ifndef LWB_SCHED_PERIOD_MIN
#define LWB_SCHED_PERIOD_MIN            2       // minimum round period, must be higher than T_ROUND_MAX
#endif /* LWB_SCHED_PERIOD_MIN */

#ifndef LWB_SCHED_PERIOD_IDLE
#define LWB_SCHED_PERIOD_IDLE           10      // default period (when no nodes are in the network)
#endif /* LWB_SCHED_PERIOD_IDLE */

#ifndef LWB_SCHED_STREAM_REMOVAL_THRES
#define LWB_SCHED_STREAM_REMOVAL_THRES  10      // threshold for the stream removal (max. number of 'misses')
#endif /* LWB_SCHED_STREAM_REMOVAL_THRES */

                                

/**
 * @brief the header length of a schedule packet
 */
#define LWB_SCHED_HEADER_LEN    8
/**
 * @brief the structure of a schedule packet
 */
typedef struct {    
    uint32_t time;
    uint16_t host_id;
    uint8_t  period;
    uint8_t  n_slots;       // store num. of data slots and last two bits to indicate whether there is a contention or an s-ack slot in this round
    uint16_t slot[LWB_MAX_DATA_SLOTS];
} lwb_schedule_t;

#define SACK_MIN_PKT_SIZE       4
typedef struct {                    // stream acknowledgement
    uint16_t node_id;              
    uint8_t  stream_id;            
    uint8_t  n_extra;               // number of additional sack's in this packet
    uint8_t  extra[LWB_MAX_PACKET_LEN - SACK_MIN_PKT_SIZE];  // additional sack's
} lwb_stream_ack_t;

/**
 * @brief statistics for the scheduler (on host node)
 */
typedef struct {
    uint16_t n_added;
    uint16_t n_deleted;
    uint16_t n_no_space;
    uint32_t t_last_req;        // timestamp of the last stream request
    uint32_t t_last_cont;       // timestamp of the last contention slot in a schedule
} lwb_sched_stats_t;


/**
 * @brief marks the schedule s as the 1st schedule (at the beginning of a round)
 */
#define LWB_SCHED_SET_AS_1ST(s)     ((s)->period |= 0x80)
/**
 * @brief marks the schedule s as the 2nd schedule (at the end of a round)
 */
#define LWB_SCHED_SET_AS_2ND(s)     ((s)->period &= ~0x80)
/**
 * @brief checks whether schedule s is the 1st schedule (at the beginning of a round)
 */
#define LWB_SCHED_IS_1ST(s)         (((s)->period & 0x80) > 0)
/**
 * @brief checks whether schedule s is the 2nd schedule (at the end of a round)
 */
#define LWB_SCHED_IS_2ND(s)         (((s)->period & 0x80) == 0)
/**
 * @brief returns the number of data slots from schedule s
 */
#define LWB_SCHED_N_SLOTS(s)          ((s)->n_slots & 0x3f)
/**
 * @brief checks whether schedule s has data slots
 */
#define LWB_SCHED_HAS_DATA_SLOT(s)    (((s)->n_slots & 0x3f) > 0)
/**
 * @brief checks whether schedule s has a contention slot
 */
#define LWB_SCHED_HAS_CONT_SLOT(s)    (((s)->n_slots & 0x40) > 0)
/**
 * @brief checks whether schedule s has an S-ACK slot
 */
#define LWB_SCHED_HAS_SACK_SLOT(s)    (((s)->n_slots & 0x80) > 0)
/**
 * @brief marks schedule s to have a contention slot
 */
#define LWB_SCHED_SET_CONT_SLOT(s)    ((s)->n_slots |= 0x40)
/**
 * @brief marks schedule s to have an S-ACK slot
 */
#define LWB_SCHED_SET_SACK_SLOT(s)    ((s)->n_slots |= 0x80)


uint8_t  lwb_sched_prep_sack(void *payload);        // prepare a stream ack
void     lwb_sched_proc_srq(const lwb_stream_req_t* req);       // process a stream request
uint16_t lwb_sched_init(lwb_schedule_t* sched);
uint16_t lwb_sched_compute(lwb_schedule_t * const sched, const uint8_t * const streams_to_update, uint8_t n_slot_host);
uint8_t  lwb_sched_uncompress(uint8_t* compressed_data, uint8_t n_slots);


#endif /* __SCHEDULER_H__ */
