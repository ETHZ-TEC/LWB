
/**
 * @ingroup LWB
 * @file
 * @brief   configuration file for the LWB
 * @author  rdaforno
 */
 
#ifndef CONFIG_H
#define CONFIG_H


#define HOST_NODE                                     // uncomment if this is a host node
//#define FLOCKLAB                                        // uncomment if you run the SW on FlockLAB
#define FLOCKLAB_HOST_ID    2


// LWB

#ifdef FLOCKLAB
  #define TX_POWER          RF1A_TX_POWER_PLUS_10_dBm   // set the antenna gain
  #define TX_CHANNEL        5                           // set the wireless channel
#else
  #define TX_POWER          RF1A_TX_POWER_MINUS_12_dBm  // set the antenna gain
  #define TX_CHANNEL        10                          // set the wireless channel
#endif // FLOCKLAB
#define TX_BITRATE          250000                      // radio transmission bitrate, do not change
#define PACKET_LEN_MAX      127                         // the max. length of a packet (limits the message size as well as the max. size of a LWB packet and the schedule)
#define N_TX_MAX_SCHEDULE   3                           // max. number of TX phases for a schedule packet (how many times each node transmits a packet)
#define N_TX_MAX_DATA       2                           // max. number of TX phases for a data packet
#define N_HOPS_MAX          3                           // max. number of hops in the network to reach all nodes (should be <= 3)
#define N_SLOTS_MAX         40                          // max. number of data slots per round, must not exceed MIN(63, (PACKET_LEN_MAX - SCHED_HEADER_LENGTH) / 2)
#define N_STREAMS_MAX       50                          // max. number of streams (bounds the required memory on the host)
#define N_PENDING_SACK_MAX  5                           // max. number of pending stream acks per round, should be lower than 31. Any further requests will be ignored. Memory usage: 4x N_PENDING_SACK_MAX bytes
#define N_DATA_PACKETS_MAX  10                          // max. number of data packets buffered on the host node
#define N_STREAMS_MAX_SRC   10                          // max. number of streams that a source node may allocate (MUST NOT be higher than 32)
// slot durations (in timer ticks)
#ifndef FLOCKLAB
  #define T_PREPROCESS      (RTIMER_SECOND / 50)        // how much time to do processing before a round starts
#endif
#define T_REF_OFFSET        (RTIMER_SECOND / 800)       // constant time offset that is added to t_ref in each round (to align the glossy start pulses on host and source nodes)
#define T_SCHED             (RTIMER_SECOND / 30)        // length of a schedule slot: approx. 33 ms 
#define T_DATA              (RTIMER_SECOND / 50)        // length of a data slot: 20 ms (make them long enough to allow 3 - 5 floods per node and 5 - 6 hops through the network)
#define T_GAP               (RTIMER_SECOND / 250)       // gap between two consecutive slots: 4 ms, must be high enough to finish all local computations between the slots
#define T_GUARD             (RTIMER_SECOND / 2000)      // initial guard time, (RTIMER_SECOND / 2000) = 0.5 ms, guard times are needed to account for clock drifts
#define T_GUARD_1           (RTIMER_SECOND / 1000)      //   1 ms (must not be higher than 1 ms, otherwise T_GAP is too short!)
#define T_GUARD_2           (RTIMER_SECOND / 200)       //   5 ms
#define T_GUARD_3           (RTIMER_SECOND / 50)        //  20 ms
#define T_SCHED2_START      (RTIMER_SECOND)             // start point (offset) of the second schedule at the end of a round
#define T_ROUND_MAX         ((N_SLOTS_MAX + 2) * (T_DATA + T_GAP) + T_SCHED + T_GAP * 2)
#define T_SLOT_MIN          ((N_HOPS_MAX + 2 * N_TX_MAX_DATA - 2) * T_HOP)         // minimum duration of a data slotaccording to "Energy-efficient Real-time Communication in Multi-hop Low-power Wireless Networks" (Zimmerling et al.), Appendix I. For 127b packets ~22.5ms, for 50b packets just over 10ms
#define T_HOP               ((RTIMER_SECOND * (3 + 24 + 192 + 192 + (1000000 * PACKET_LEN_MAX * 8 / TX_BITRATE) )) / 1000000)   // min. duration of 1 packet transmission with glossy (approx. values, taken from TelosB platform measurements) -> for 127b packets ~4.5ms, for 50b packets just over 2ms


// SCHEDULER

#define COMPRESS_SCHEDULE                               // uncomment to enable schedule compression
#ifndef FLOCKLAB
  #define SCHEDULER_USE_XMEM                              // uncomment to use the external memory to store the stream information (on the host node)
#endif
#define PERIOD_MAX          30                          // max. assignable round period in seconds, must not exceed 127 seconds!
#define PERIOD_MIN          2                           // minimum round period, must be higher than T_ROUND_MAX
#define PERIOD_IDLE         10                          // default period (when no nodes are in the network)
#define T_NO_REQ            PERIOD_MIN * 2              // how long no stream request until period is adjusted accordingly
#define T_NO_COMM           (PERIOD_MAX * 4 * RTIMER_SECOND)    // threshold for the host fail-over policy (if no communication within this time, we assume the host has failed and thus, a new host must be selected)
#define N_CONS_MISSED_MAX   10                          // threshold for the stream removal


// MESSAGE MANAGER

#define MESSAGE_SIZE        48                          // size of one message (for the asynchronous interface)
#define N_BUFFERED_IN_MSG   10                          // max. number of buffered input messages (in the external memory)
#define N_BUFFERED_OUT_MSG  10                          // max. number of buffered output messages (in the external memory)


// HARDWARE (see hal.h for more parameters)

// select the used CC430 board
//#define BOARD_EM430F5137RF900                         // currently used evaluation board for the CC430F5137
#ifdef FLOCKLAB
  #define BOARD_MSP430CCRF                              // the red board (Olimex)
#else 
  #define BOARD_COMM_V1                                 // custom board v1.0
#endif // FLOCKLAB
//#define ASYNC_INT_USE_DMA                             // uncomment to use DMA-driven data transfer from and to the asynchronous data interface
//#define ASYNC_INT_TIMEREQ_POLLING                     // uncomment to use polling (and DMA) for the timestamp request instead of interrupts
//#define FRAM_USE_DMA                                  // uncomment to use DMA-driven data transfer from and to the external FRAM
#define MAX_CLOCK_DEV       500                         // the max. clock deviation (according to the specification of the crystal), in ACLK cycles per second (3.25 MHz)
//#define SPI_FAST_READ                                 // in polling mode only: comment this line to disable fast read from SPI (i.e. transfer 2 bytes, then start reading -> there's a chance to miss a byte, should not be enabled for SPI speeds > MCLK / 2)
    
    
// DEBUGING and STATISTICS

#define USE_LEDS                                        // comment this line to disable all LEDs
#define DEBUG_LEVEL                 DEBUG_LVL_INFO      // select the debug level, must be of type debug_level_t
#ifndef FLOCKLAB
  #define DEBUG_PRINT_BUFFER_XMEM                       // uncomment to buffer the print messages on the external memory  
  #define STORE_STATS_XMEM                              // uncomment to store the statistics in the external memory
#endif
#define DEBUG_PRINT_MAX_LENGTH      80                  // including the null-character if required
#ifdef DEBUG_PRINT_BUFFER_XMEM
  #define N_DEBUG_MSG               20                  // buffer size for debug prints (if more debug messages are generated per round, they will be dropped)
#else
  #define N_DEBUG_MSG               8                   // buffer size for debug prints (if more debug messages are generated per round, they will be dropped)
#endif


#endif
