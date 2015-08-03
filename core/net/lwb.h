
/**
 * @defgroup LWB
 * @file
 * @ingroup  LWB
 * @brief    the low-power wireless bus (LWB)
 * @author   rdaforno
 */
 
#ifndef __LWB_H__
#define __LWB_H__

#include "platform.h"   // TODO: make LWB independent of platform!


#ifndef LWB_T_REF_OFS
#define LWB_T_REF_OFS           (RTIMER_SECOND / 800)       // constant time offset that is added to t_ref in each round (to align the glossy start pulses on host and source nodes)
#endif /* LWB_T_REF_OFS */

#ifndef LWB_T_SCHED
#define LWB_T_SCHED             (RTIMER_SECOND / 30)        // length of a schedule slot: approx. 33 ms 
#endif /* LWB_T_SCHED */

#ifndef LWB_T_DATA
#define LWB_T_DATA              MAX(LWB_T_SLOT_MIN, RTIMER_SECOND / 50)    // length of a data slot: 20 ms (make them long enough to allow 3 - 5 floods per node and 5 - 6 hops through the network)
#endif /* LWB_T_DATA */

#ifndef LWB_T_GAP
#define LWB_T_GAP               (RTIMER_SECOND / 250)       // gap between two consecutive slots: 4 ms, must be high enough to finish all local computations between the slots
#endif /* LWB_T_GAP */

#ifndef LWB_T_GUARD
#define LWB_T_GUARD             (RTIMER_SECOND / 2000)      // initial guard time, (RTIMER_SECOND / 2000) = 0.5 ms, guard times are needed to account for clock drifts
#endif /* LWB_T_GUARD */

#ifndef LWB_T_GUARD_1
#define LWB_T_GUARD_1           (RTIMER_SECOND / 1000)      //   1 ms (must not be higher than 1 ms, otherwise T_GAP is too short!)
#endif /* LWB_T_GUARD_1 */

#ifndef LWB_T_GUARD_2
#define LWB_T_GUARD_2           (RTIMER_SECOND / 200)       //   5 ms
#endif /* LWB_T_GUARD_2 */

#ifndef LWB_T_GUARD_3
#define LWB_T_GUARD_3           (RTIMER_SECOND / 50)        //  20 ms
#endif /* LWB_T_GUARD_3 */

#ifndef LWB_IN_BUFFER_SIZE         
#define LWB_IN_BUFFER_SIZE      10        /* size (#elements) of the internal data buffer/queue for incoming messages */
#endif /* LWB_IN_BUFFER_SIZE */

#ifndef LWB_OUT_BUFFER_SIZE         
#define LWB_OUT_BUFFER_SIZE     10        /* size (#elements) of the internal data buffer/queue for outgoing messages */
#endif /* LWB_IN_BUFFER_SIZE */

#ifndef LWB_STATS_NVMEM
#define LWB_STATS_NVMEM         1         /* keep stats in non-volatile memory? */ // TODO: use this define in the code, implement Flash/FRAM/SRAM stats logging
#endif /* LWB_STATS_NVMEM */

#ifndef LWB_MAX_PACKET_LEN
#define LWB_MAX_PACKET_LEN      127       /* the max. length of a packet (limits the message size as well as the max. size of a LWB packet and the schedule) */
#endif /* LWB_MAX_PACKET_LEN */

#ifndef LWB_MAX_DATA_SLOTS
#define LWB_MAX_DATA_SLOTS      40        /* max. number of data slots per round, must not exceed MIN(63, (LWB_MAX_PACKET_LEN - LWB_SCHED_HEADER_LEN) / 2) */
#endif /* LWB_MAX_DATA_SLOTS */

#ifndef LWB_TX_CNT_SCHED
#define LWB_TX_CNT_SCHED        3         /* max. number of TX phases for a schedule packet (how many times each node transmits a packet) */
#endif /* LWB_TX_CNT_SCHED */

#ifndef LWB_TX_CNT_DATA
#define LWB_TX_CNT_DATA         2               /* max. number of TX phases for a data packet */
#endif

#ifndef LWB_MAX_HOPS
#define LWB_MAX_HOPS            3               /* max. number of hops in the network to reach all nodes (should be <= 3) */
#endif /* LWB_MAX_HOPS */

//#define LWB_N_DATA_PACKETS_MAX  10            // max. number of data packets buffered on the host node

#ifndef LWB_T_SILENT
#define LWB_T_SILENT            (LWB_SCHED_PERIOD_MAX * 4 * RTIMER_SECOND)    // threshold for the host fail-over policy (if no communication within this time, we assume the host has failed and thus, a new host must be selected)
#endif /* LWB_T_SILENT */

#ifndef RF1A_CONF_TX_POWER        
#define RF1A_CONF_TX_POWER      RF1A_TX_POWER_MINUS_12_dBm   // set the antenna gain
#endif /* RF1A_TX_POWER */

#ifndef RF1A_CONF_TX_BITRATE
#define RF1A_CONF_TX_BITRATE    250000          // radio transmission bitrate, do not change
#endif /* RF1A_TX_BITRATE */

#ifndef RF1A_CONF_TX_CHANNEL
#define RF1A_CONF_TX_CHANNEL    10              // set the wireless channel
#endif /* RF1A_TX_CHANNEL */

#ifndef RF1A_CONF_MAX_PKT_LEN
#define RF1A_CONF_MAX_PKT_LEN   LWB_MAX_PACKET_LEN
#endif /* RF1A_CONF_MAX_PKT_LEN */

#ifndef LWB_T_SCHED2_START
#define LWB_T_SCHED2_START      MAX(LWB_T_ROUND_MAX, RTIMER_SECOND) // start point (offset) of the second schedule at the end of a round
#endif /* LWB_T_SCHED2_START */

#ifndef LWB_MAX_CLOCK_DEV
#define LWB_MAX_CLOCK_DEV       500             // the max. clock deviation (according to the specs of the oscillator), in SMCLK cycles per second (3.25 MHz)
#endif /* LWB_MAX_CLOCK_DEV */

#ifndef LWB_RTIMER_ID
#define LWB_RTIMER_ID           RTIMER_TA0_0    // ID of the rtimer used for the LWB (must be of type rtimer_ta0_t, use TA0_0 for highest priority)
#endif /* LWB_RTIMER_ID */

#ifndef LWB_WAKEUP_RTIMER_ID
#define LWB_WAKEUP_RTIMER_ID    RTIMER_TA1_1    // ID of the rtimer used to trigger the wake-up
#endif /* LWB_WAKEUP_RTIMER_ID */

#ifndef LWB_T_PREPROCESS        
#define LWB_T_PREPROCESS        0       /* (RTIMER_SECOND / 50)  -> how much time to do processing before a round starts, 0 = disabled */
#endif /* LWB_T_PREPROCESS */

#ifndef LWB_T_PREPROCESS_TA1
#define LWB_T_PREPROCESS_TA1    0       /* (RTIMER_SECOND_TA1 / 50)  -> how much time to do processing before a round starts, 0 = disabled */
#endif /* LWB_T_PREPROCESS_TA1 */

#ifndef LWB_MAX_STREAM_CNT_PER_NODE
#define LWB_MAX_STREAM_CNT_PER_NODE     32
#endif /* LWB_MAX_STREAM_CNT_PER_NODE */

#ifndef LWB_MAX_STREAM_CNT
#define LWB_MAX_STREAM_CNT      50      /* max. number of streams (bounds the required memory on the host) */
#endif /* N_STREAMS_MAX */

/* important values */
#define LWB_T_ROUND_MAX         ( (LWB_MAX_DATA_SLOTS + 2) * (LWB_T_DATA + LWB_T_GAP) + LWB_T_SCHED + LWB_T_GAP * 2 )
#define LWB_T_HOP               ( (RTIMER_SECOND * (3 + 24 + 192 + 192 + (1000000 * LWB_MAX_PACKET_LEN * 8 / RF1A_CONF_TX_BITRATE) )) / 1000000 )   // min. duration of 1 packet transmission with glossy (approx. values, taken from TelosB platform measurements) -> for 127b packets ~4.5ms, for 50b packets just over 2ms
#define LWB_T_SLOT_MIN          ( (LWB_MAX_HOPS + 2 * LWB_TX_CNT_DATA - 2) * LWB_T_HOP )         // minimum duration of a data slot according to "Energy-efficient Real-time Communication in Multi-hop Low-power Wireless Networks" (Zimmerling et al.), Appendix I. For 127b packets ~22.5ms, for 50b packets just over 10ms
#define MAX(x, y)               ((x) > (y) ? (x) : (y))

#define LWB_INVALID_STREAM_ID   0xff

#define LWB_RECIPIENT_LOCAL     0       // destined for this node
#define LWB_RECIPIENT_HOST      0xfffd  // send it to all the host
#define LWB_RECIPIENT_SINK      0xfffe  // send it to all sinks
#define LWB_RECIPIENT_BROADCAST 0xffff  // send the packet to all receivers

#define LWB_RECIPIENT_GROUP_MASK  0xf000  // group ID mask 
#define LWB_RECIPIENT_NODE_MASK   0x0fff  // node ID mask


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
} lwb_statistics_t;

typedef enum {
    NOT_CONNECTED = 0,
    CONNECTED,
    CONNECTION_LOST,
} lwb_conn_state_t;

/**
 * @brief the structure of a stream request 
 */
typedef struct {
    uint16_t node_id;       // ID of the node that wants to request a new stream
    uint8_t  stream_id;     // stream ID (chosen by the source node)
    uint8_t  ipi;           // load (inter-packet interval) in seconds 
    int8_t   t_offset;      // starting time offset (necessary to get rid of e.g. a backlog of messages within a short time)
} lwb_stream_req_t;


/**
 * @brief start the Low-Power Wireless Bus
 * @param application_proc a pointer to the application task process control block (struct process)
 */
void lwb_start(void *application_proc);

/**
 * @brief query the connection status of the LWB
 * @return the 
 */
lwb_conn_state_t lwb_get_state(void);

/**
 * @brief schedule a packet for transmission over the LWB
 * @param data a pointer to the data packet to send
 * @param len the length of the data packet (must be less or equal 
 * LWB_MAX_PACKET_LEN)
 */
uint8_t lwb_send_data(uint8_t stream_id, const uint8_t * const data, uint8_t len);

/**
 * @brief receive a data packet that have been received during the previous LWB
 * rounds
 * @param out_data A valid memory block that can hold one data packet. The 
 * buffer must be at least LWB_MAX_PACKET_LEN bytes long.
 * @param out_stream_id Pointer to a variable where the stream ID will be copied
 * to (optional parameter).
 * @note once a data packet was retrieved, it will be removed from the internal
 * buffer
 */
uint8_t lwb_rcv_data(uint8_t* out_data, uint8_t * const out_stream_id);

/**
 * @brief check the status of the receive buffer (incoming messages)
 * @return 1 if there is at least 1 message in the queue, 0 otherwise
 */
uint8_t lwb_get_rcv_buffer_state(void);

/**
 * @brief check the status of the send buffer (outgoing messages)
 * @return 1 if there is at least 1 message in the queue, 0 otherwise
 */
uint8_t lwb_get_send_buffer_state(void);

/**
 * @brief schedules a stream request to be sent during the contention slot
 * @param urgent if you set this parameter to 1, the LWB tries to piggyback the stream
 * request onto a data packet
 * @return 0 if an error occurred (e.g. queue full), 1 otherwise
 */
uint8_t lwb_request_stream(uint8_t stream_id, uint8_t ipi, int8_t offset, uint8_t urgent);

uint8_t lwb_get_stream_state(uint8_t stream_id);

/**
 * @brief get the LWB statistics
 */
const lwb_statistics_t * const lwb_get_stats(void);

/**
 * @brief reset the statistics
 */
void lwb_stats_reset(void);


#endif /* __LWB_H__ */
