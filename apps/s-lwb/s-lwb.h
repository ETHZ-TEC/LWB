
/**
 * @defgroup LWB
 * @file
 * @ingroup  LWB
 * @brief    the low-power wireless bus (LWB)
 * @author   rdaforno
 */
 
#ifndef SLWB_H
#define SLWB_H


#include "contiki.h"
#include "platform.h"


/**
 * @brief the start time of slot i relative to the t_start (offset)
 */
#define T_SLOT_START(i)         ( (T_SCHED + T_GAP + T_GAP) + (T_DATA + T_GAP) * i )   
/**
 * @brief checks if data was received in the last glossy flood
 */
#define DATA_RECEIVED           (glossy_get_n_rx() > 0)

/**
 * @brief bound (saturate) x between min and max
 */
#define LIMIT(x, min, max)      ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))
/**
 * @brief returns the maximum of x and y
 */
#define MAX(x, y)               ((x) > (y) ? (x) : (y))

#define LWB_RTIMER_ID           RTIMER_TA0_0        // ID of the rtimer used for the LWB (must be of type rtimer_ta0_t)
#define WAKEUP_RTIMER_ID        RTIMER_TA1_1        // ID of the rtimer used to trigger the wake-up
#define RECIPIENT_BROADCAST     0xffff  // send the packet to all receivers
#define RECIPIENT_LOCAL         0       // destined for this node

#ifdef FLOCKLAB
  #define START_LWB(offset)     ( rtimer_schedule(LWB_RTIMER_ID, rtimer_now() + RTIMER_SECOND * (offset), 0, (node_id == FLOCKLAB_HOST_ID) ? slwb_thread_host : slwb_thread_source) )
#else
  #ifdef HOST_NODE
    #define START_LWB(offset)   ( rtimer_schedule(LWB_RTIMER_ID, rtimer_now() + RTIMER_SECOND * (offset), 0, slwb_thread_host) )
  #else
    #define START_LWB(offset)   ( rtimer_schedule(LWB_RTIMER_ID, rtimer_now() + RTIMER_SECOND * (offset), 0, slwb_thread_source) )
  #endif
#endif

/**
 * @brief saves the current timestamp in t_now
 */
#define RTIMER_CAPTURE          ( t_now = rtimer_now() )
/**
 * @brief returns the elapsed time in milliseconds since the last call of RTIMER_CAPTURE
 */
#define RTIMER_ELAPSED          ( (rtimer_now() - t_now) * 1000 / 3250 )        

/**
 * @brief debug pin to indicate the LWB task activity
 */
#ifdef LWB_TASK_ACT_PIN
  #define LWB_TASK_ACTIVE       PIN_SET(LWB_TASK_ACT_PIN)
  #define LWB_TASK_SUSPENDED    PIN_CLEAR(LWB_TASK_ACT_PIN)
#else
  #define LWB_TASK_ACTIVE       
  #define LWB_TASK_SUSPENDED    
#endif


/**
 * @brief source node synchronization states
 */
typedef enum {
    BOOTSTRAP = 0,
    QUASI_SYNCED,
    SYNCED,
    ALREADY_SYNCED,
    UNSYNCED_1,
    UNSYNCED_2,
    UNSYNCED_3,
    NUM_OF_SYNC_STATES
} sync_state_t;

/**
 * @brief synchronization events on a source node
 */
typedef enum {
    EVT_1ST_SCHED_RCVD = 0,
    EVT_2ND_SCHED_RCVD,
    EVT_SCHED_MISSED,
    NUM_OF_SYNC_EVENTS
} sync_event_t;

/**
 * @brief keep some statistics
 */
typedef struct {
    uint8_t  relay_cnt;
    uint8_t  period_last;
    uint8_t  unsynced_cnt;
    uint8_t  bootstrap_cnt;
    uint16_t reset_cnt;
    uint16_t pck_cnt;           // total number of received packets
    uint16_t t_sched_max;       // max. time needed to calculate the new schedule
    uint16_t t_proc_max;        // max. time needed to process the received data packets
    uint16_t t_prep_max;        // max. time needed to prepare a packet
    uint16_t crc;               // crc of this struct (with crc set to 0!)
    uint32_t t_slot_last;       // last time a slot has been assigned to this node (in seconds)
    uint32_t data_tot;
} statistics_t;


/**
 * @brief size of a stream request packet (see glossy_payload_t)
 */
#define STREAM_REQ_PKT_SIZE     5
/**
 * @brief the minimum size of an S-ACK packet (see glossy_payload_t)
 */
#define SACK_PKT_SIZE_MIN       4  
/**
 * @brief the minimum size of an S-ACK packet (see glossy_payload_t)
 */
#define DATA_PKT_HEADER_SIZE    3  

/**
 * @brief a glossy packet (can be data, a stream request or a stream acknowledgement)
 */
typedef struct { 
    
    // be aware of structure alignment (8-bit are aligned to 8-bit, 16 to 16 etc.)
    // note: the first 3 bytes of a glossy packet are always node_id and stream_id
    union {    
        struct {            
            uint16_t recipient; // target node ID
            uint8_t  stream_id; // message type and connection ID (used as stream ID in LWB); first bit is msg type
            uint8_t  payload[PACKET_LEN_MAX - DATA_PKT_HEADER_SIZE];           
        } data_pkt;
        
        struct {                            // stream request
            uint16_t node_id;               // ID of the node that wants to request a new stream
            uint8_t  stream_id;             // stream ID (chosen by the source node)
            uint8_t  ipi;                   // load (inter-packet interval) in seconds 
            int8_t   t_offset;              // starting time offset (necessary to get rid of e.g. a backlog of messages within a short time)
            uint8_t  reserved[PACKET_LEN_MAX - STREAM_REQ_PKT_SIZE];  // unused
        } srq_pkt;
        
        struct {                            // stream acknowledgement
            uint16_t node_id;              
            uint8_t  stream_id;            
            uint8_t  n_extra;               // number of additional sack's in this packet
            uint8_t  extra[PACKET_LEN_MAX - SACK_PKT_SIZE_MIN];  // additional sack's
        } sack_pkt;
        
        uint8_t raw_data[PACKET_LEN_MAX];   // access to the bytes of the packet
    };
} glossy_payload_t;



void stats_reset(void);



#include "scheduler.h"
#include "mm.h"
#include "stream.h"
#include "log.h"



#endif
