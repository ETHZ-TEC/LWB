
/**
 * @file
 * @ingroup LWB
 * @brief   scheduler for the LWB, runs on the host node only
 * @author  fe
 * @author  rdaforno
 */
 
#ifndef SCHEDULER_H
#define SCHEDULER_H


#define PREPARE_1ST_SCHED(s)    {\
                                    s.time += (s.period - 1); /* update the timestamp */\
                                    SCHED_SET_AS_1ST(&s);     /* mark this schedule as first */\
                                }
                                

/**
 * @brief the header length of a schedule packet
 */
#define SCHED_HEADER_LENGTH 8
/**
 * @brief the structure of a schedule packet
 */
typedef struct {	
	uint32_t time;
	uint16_t host_id;
	uint8_t  period;
	uint8_t  n_slots;       // store num. of data slots and last two bits to indicate whether there is a contention or an s-ack slot in this round
	uint16_t slot[N_SLOTS_MAX];
} schedule_t;

/**
 * @brief the structure of a stream request 
 */
typedef struct {
    uint16_t node_id;       // ID of the node that wants to request a new stream
    uint8_t  stream_id;     // stream ID (chosen by the source node)
    uint8_t  ipi;           // load (inter-packet interval) in seconds 
    int8_t   t_offset;      // starting time offset (necessary to get rid of e.g. a backlog of messages within a short time)
} stream_request_t;

/**
 * @brief struct to store information about active streams on the host
 */
typedef struct stream_info {
#ifndef SCHEDULER_USE_XMEM
	struct stream_info *next;
#else
    uint32_t next;          
#endif // SCHEDULER_USE_XMEM
	uint16_t node_id;
	uint16_t ipi;
	uint32_t last_assigned;
	uint8_t  stream_id;
	uint8_t  n_cons_missed;
} stream_info_t;

/**
 * @brief statistics for the scheduler (on host node)
 */
typedef struct {
    uint16_t n_added;
    uint16_t n_deleted;
    uint16_t n_no_space;
    uint32_t t_last_req;        // timestamp of the last stream request
    uint32_t t_last_cont;       // timestamp of the last contention slot in a schedule
} sched_stats_t;



/**
 * @brief sets the last bit of the byte b
 */
#define SET_LAST_BIT(b)         ((b) |= 0x80)
/**
 * @brief clears the last bit of the byte b
 */
#define CLEAR_LAST_BIT(b)       ((b) &= ~0x80)
/**
 * @brief checks whether the last bit of byte b is set
 */
#define IS_LAST_BIT_SET(b)      (((b) & 0x80) > 0)

/**
 * @brief marks the schedule s as the 1st schedule (at the beginning of a round)
 */
#define SCHED_SET_AS_1ST(s)     (SET_LAST_BIT((s)->period))
/**
 * @brief marks the schedule s as the 2nd schedule (at the end of a round)
 */
#define SCHED_SET_AS_2ND(s)     (CLEAR_LAST_BIT((s)->period))
/**
 * @brief checks whether schedule s is the 1st schedule (at the beginning of a round)
 */
#define SCHED_IS_1ST(s)         (IS_LAST_BIT_SET((s)->period))
/**
 * @brief checks whether schedule s is the 2nd schedule (at the end of a round)
 */
#define SCHED_IS_2ND(s)         (!IS_LAST_BIT_SET((s)->period))

/**
 * @brief returns the number of data slots from schedule s
 */
#define SCHED_N_SLOTS(s)          ((s)->n_slots & 0x3f)
/**
 * @brief checks whether schedule s has data slots
 */
#define SCHED_HAS_DATA_SLOT(s)    (((s)->n_slots & 0x3f) > 0)
/**
 * @brief checks whether schedule s has a contention slot
 */
#define SCHED_HAS_CONT_SLOT(s)    (((s)->n_slots & 0x40) > 0)
/**
 * @brief checks whether schedule s has an S-ACK slot
 */
#define SCHED_HAS_SACK_SLOT(s)    (((s)->n_slots & 0x80) > 0)
/**
 * @brief marks schedule s to have a contention slot
 */
#define SCHED_SET_CONT_SLOT(s)    ((s)->n_slots |= 0x40)
/**
 * @brief marks schedule s to have an S-ACK slot
 */
#define SCHED_SET_SACK_SLOT(s)    ((s)->n_slots |= 0x80)



uint8_t sched_prep_sack(glossy_payload_t *payload);     // prepare a stream ack
void sched_proc_srq(const stream_request_t* req);       // process a stream request
uint16_t sched_init(schedule_t* sched);
uint16_t sched_compute(schedule_t * const sched, const uint8_t * const streams_to_update, uint8_t n_slot_host);
void sched_uncompress(uint8_t* compressed_data, uint8_t n_slots);


#endif
