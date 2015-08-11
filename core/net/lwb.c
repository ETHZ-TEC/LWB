/**
 * @file
 *
 * To-do:
 * - extended verification / test (sync state machine, scheduler, ext. memory usage...) -> think of a way to test it
 * - go through all TODO tags in the code
 * - connect pwr_sel pins
 * - fix occasional transmission errors (corrupted packets) if unexpected
 * - implement a command to update the firmware (broadcast)
 * - agree on a max. msg size and adjust the slot durations and gap times
 * - verify the implementation of the message manager and prevent possible deadlocks (e.g.: what if a 'resume' message gets lost?)
 * - code review
 * 
 * Optimization suggestions:
 * - use DMA to drive the SPI data transfer in the background (-> requires optimization and analysis with logic analyser to verify the gain)
 * 
 * Assumptions and constraints:
 * - see report
 * - further: if mm stays the way it is: the application layer may not change the structure of message_t (at least it must maintain the first two fields).
 * 
 * Hints:
 * - use "msp430-objdump -h <filename>" to print out all linker sections, their location and size; besides, gcc outputs the size of the RO (.text) and RAM (.bss + .data) sections
 * - make sure the unused RAM space for the stack is at least 100 B (check .bss section)
 * - the max. period may be much longer than 30s, test showed that even 30min are feasible (note though that the period is a uint8 variable with the last bit reserved)
 * - sprintf is very inefficient (~2000 cycles to copy a string) -> use memcpy instead
 * - structs always cause problems: misalignments due to "compiler optimizations", use uintx_t types instead of enum and union instead of conversion to a pointer to a struct
 */
 
#include "contiki.h"
#include "glossy.h"
#include "scheduler.h"
#include "stream.h"

/*---------------------------------------------------------------------------*/
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
/*---------------------------------------------------------------------------*/
typedef enum {
    EVT_1ST_SCHED_RCVD = 0,
    EVT_2ND_SCHED_RCVD,
    EVT_SCHED_MISSED,
    NUM_OF_SYNC_EVENTS
} sync_event_t; 
#define LWB_DATA_PKT_HEADER_LEN    3  
#define LWB_DATA_PKT_PAYLOAD_LEN   (LWB_MAX_PACKET_LEN - LWB_DATA_PKT_HEADER_LEN)
/*---------------------------------------------------------------------------*/
typedef struct {
    uint16_t recipient;         // target node ID
    uint8_t  stream_id;         // message type and connection ID (used as stream ID in LWB); first bit is msg type
    uint8_t  payload[LWB_DATA_PKT_PAYLOAD_LEN];           
} lwb_data_pkt_t;
#define STREAM_REQ_PKT_SIZE     5
/*---------------------------------------------------------------------------*/
typedef struct {    
    // be aware of structure alignment (8-bit are aligned to 8-bit, 16 to 16 etc.)
    // note: the first 3 bytes of a glossy packet are always node_id and stream_id
    union {    
        lwb_data_pkt_t data_pkt;
        
        struct {                            // stream request
            uint16_t node_id;               // ID of the node that wants to request a new stream
            uint8_t  stream_id;             // stream ID (chosen by the source node)
            uint8_t  ipi;                   // load (inter-packet interval) in seconds 
            int8_t   t_offset;              // starting time offset (necessary to get rid of e.g. a backlog of messages within a short time)
            uint8_t  reserved[LWB_MAX_PACKET_LEN - STREAM_REQ_PKT_SIZE];  // unused
        } srq_pkt;
        
        lwb_stream_ack_t sack_pkt;
        
        uint8_t raw_data[LWB_MAX_PACKET_LEN];   // access to the bytes of the packet
    };
    uint8_t reserved;           // extra byte (padding)
    
} glossy_payload_t;
/*---------------------------------------------------------------------------*/
/**
 * @brief the finite state machine for the time synchronization on a source node
 * the next state can be retrieved from the current state (column) and the latest event (row)
 * @note  undefined transitions force the SM to go back into bootstrap
 */
static const sync_state_t next_state[NUM_OF_SYNC_EVENTS][NUM_OF_SYNC_STATES] = 
{   // STATES:                                                                                                        // EVENTS:
    // BOOTSTRAP,   QUASI_SYNCED, SYNCED,         ALREADY_SYNCED, UNSYNCED_1,     UNSYNCED_2,     UNSYNCED_3
    { QUASI_SYNCED, SYNCED,       BOOTSTRAP,      SYNCED,         SYNCED,         SYNCED,         SYNCED         },   // 1st schedule received
    { BOOTSTRAP,    QUASI_SYNCED, ALREADY_SYNCED, BOOTSTRAP,      ALREADY_SYNCED, ALREADY_SYNCED, ALREADY_SYNCED },   // 2nd schedule received
    { BOOTSTRAP,    BOOTSTRAP,    UNSYNCED_1,     UNSYNCED_1,     UNSYNCED_2,     UNSYNCED_3,     BOOTSTRAP      }    // schedule missed
};
static const char* sync_state_to_string[NUM_OF_SYNC_STATES] = { "BOOTSTRAP", "Q_SYN", "SYN", "A_SYN", "USYN_1", "USYN_2", "USYN_3" };
static const uint32_t guard_time[NUM_OF_SYNC_STATES] = {
// STATES: BOOTSTRAP,   QUASI_SYNCED, SYNCED,      ALREADY_SYNCED, UNSYNCED_1,    UNSYNCED_2,    UNSYNCED_3 
           LWB_T_GUARD, LWB_T_GUARD,  LWB_T_GUARD, LWB_T_GUARD,    LWB_T_GUARD_1, LWB_T_GUARD_2, LWB_T_GUARD_3  };
/*---------------------------------------------------------------------------*/
#ifdef LWB_TASK_ACT_PIN
  #define LWB_TASK_RESUMED      PIN_SET(LWB_TASK_ACT_PIN)
  #define LWB_TASK_SUSPENDED    PIN_CLEAR(LWB_TASK_ACT_PIN)
#else
  #define LWB_TASK_RESUMED       
  #define LWB_TASK_SUSPENDED    
#endif
/*---------------------------------------------------------------------------*/
#define LWB_T_SLOT_START(i)     ( (LWB_T_SCHED + LWB_T_GAP + LWB_T_GAP) + (LWB_T_DATA + LWB_T_GAP) * i )   
#define LWB_DATA_RCVD           ( glossy_get_n_rx() > 0 )
#define RTIMER_CAPTURE          ( t_now = rtimer_now() )
#define RTIMER_ELAPSED          ( (rtimer_now() - t_now) * 1000 / 3250 )        
#define GET_EVENT       ( glossy_is_t_ref_updated() ? (LWB_SCHED_IS_1ST(&schedule) ? EVT_1ST_SCHED_RCVD : EVT_2ND_SCHED_RCVD) : EVT_SCHED_MISSED )
/*---------------------------------------------------------------------------*/
#define LWB_SEND_SCHEDULE() \
{\
    glossy_start(node_id, (uint8_t *)&schedule, schedule_len, LWB_TX_CNT_SCHED, GLOSSY_WITH_SYNC, GLOSSY_WITH_RF_CAL);\
    LWB_WAIT_UNTIL(rt->time + LWB_T_SCHED);\
    glossy_stop();\
}   
#define LWB_RCV_SCHEDULE() \
{\
    glossy_start(0, (uint8_t *)&schedule, 0, LWB_TX_CNT_SCHED, GLOSSY_WITH_SYNC, GLOSSY_WITH_RF_CAL);\
    LWB_WAIT_UNTIL(rt->time + LWB_T_SCHED + t_guard);\
    glossy_stop();\
}   
#define LWB_SEND_PACKET(ini) \
{\
    glossy_start(ini, (uint8_t*)&glossy_payload, payload_len, LWB_TX_CNT_DATA, GLOSSY_WITHOUT_SYNC, GLOSSY_WITHOUT_RF_CAL);\
    LWB_WAIT_UNTIL(rt->time + LWB_T_DATA + 0);\
    glossy_stop();\
}

#define LWB_RCV_PACKET() \
{\
    glossy_start(GLOSSY_UNKNOWN_INITIATOR, (uint8_t*)&glossy_payload, 0, LWB_TX_CNT_DATA, GLOSSY_WITHOUT_SYNC, GLOSSY_WITHOUT_RF_CAL);\
    LWB_WAIT_UNTIL(rt->time + LWB_T_DATA + t_guard);\
    glossy_stop();\
}
/*---------------------------------------------------------------------------*/
/**
 * @brief suspends the lwb proto-thread until the rtimer reaches the specified timestamp time
 */
#define LWB_WAIT_UNTIL(time) \
{\
    rtimer_schedule(LWB_RTIMER_ID, time, 0, callback_func);\
    LWB_TASK_SUSPENDED;\
    PT_YIELD(&lwb_pt);\
    LWB_TASK_RESUMED;\
}
#define LWB_SLEEP_UNTIL(time) \
{\
    rtimer_schedule(LWB_WAKEUP_RTIMER_ID, time, 0, callback_func);\
    /*BEFORE_SLEEP();*/\
    LWB_TASK_SUSPENDED;\
    PT_YIELD(&lwb_pt);\
    /*AFTER_SLEEP();*/\
    LWB_TASK_RESUMED;\
}
#define BEFORE_SLEEP() \
{\
    fram_sleep();\
    TA0CTL  &= ~MC_3;                                      /* stop timer A0 */\
    /*UCSCTL4  = SELA__XT1CLK | SELS__XT1CLK | SELM__XT1CLK;*/ /* clock src */\
    UCSCTL6 |= XT2OFF;                                       /* disable XT2 */\
    /*P1SEL   &= ~(BIT2 | BIT3 | BIT4 | BIT5 | BIT6);*//* reconfigure GPIOs */\
    P1DIR   |= BIT2 | BIT5;\
}
#define AFTER_SLEEP() \
{\
    SFRIE1  &= ~OFIE;\
    UCSCTL6 &= ~XT2OFF;\
    do {\
        UCSCTL7 &= ~(XT2OFFG + DCOFFG);\
        SFRIFG1 &= ~OFIFG;\
    } while (SFRIFG1 & OFIFG);\
    SFRIE1  |= OFIE;\
    TA0CTL  |= MC_2;\
    UCSCTL4  = SELA | SELS | SELM;\
    P1SEL   |= (BIT2 | BIT3 | BIT4 | BIT5 | BIT6);\
    P1DIR   &= ~(BIT2 | BIT5);\
}    
#define LWB_UPDATE_SYNC_STATE \
{\
    sync_state = next_state[GET_EVENT][sync_state]; /* get the new state based on the event */\
    t_guard = guard_time[sync_state];               /* adjust the guard time */\
}
/*---------------------------------------------------------------------------*/
static struct pt          lwb_pt;            // the proto-thread 
static struct process*    app_proc;
static lwb_statistics_t   stats = { 0 };     // some statistics
static uint32_t           stats_addr = 0;
static uint8_t            urgent_stream_req = 0;
#if !LWB_USE_XMEM
static uint8_t            in_buffer_mem[LWB_IN_BUFFER_SIZE * (LWB_MAX_PACKET_LEN + 1)];  // allocate memory in the SRAM (+1 to store the message length)
static uint8_t            out_buffer_mem[LWB_OUT_BUFFER_SIZE * (LWB_MAX_PACKET_LEN + 1)]; 
#else /* LWB_USE_XMEM */
static uint8_t            msg_buffer[LWB_MAX_PACKET_LEN + 1];   /* LWB_USE_XMEM */
#endif /* LWB_USE_XMEM */
FIFO(in_buffer, LWB_MAX_PACKET_LEN + 1, LWB_IN_BUFFER_SIZE);
FIFO(out_buffer, LWB_MAX_PACKET_LEN + 1, LWB_OUT_BUFFER_SIZE);
/*---------------------------------------------------------------------------*/
static uint16_t calc_crc16(const uint8_t* data, uint8_t num_bytes) {
    uint16_t crc  = 0,
             mask = 0xa001;
    while (num_bytes) {
        uint8_t ch = *data;
        int8_t bit = 0;
        while (bit < 8) {
            if ((crc & 1) ^ (ch & 1)) {
                crc = (crc >> 1) ^ mask;
            } else {
                crc >>= 1;
            }
            ch >>= 1; 
            bit += 1;
        }
        data++;
        num_bytes--;
    }
    return crc;
}
/*---------------------------------------------------------------------------*/
uint8_t lwb_stats_load(void) {
#if LWB_USE_XMEM
    uint16_t crc;
    if(!xmem_init()) { /* make sure external memory is accessible */
      return 0;
    }
    stats_addr = xmem_alloc(sizeof(lwb_statistics_t));
    if (XMEM_ALLOC_ERROR == stats_addr || !xmem_read(stats_addr, sizeof(lwb_statistics_t), (uint8_t*)&stats)) {
        DEBUG_PRINT_MSG_NOW("WARNING: failed to load stats");
    }
    crc = stats.crc;
    stats.crc = 0;
    if (calc_crc16((uint8_t*)&stats, sizeof(lwb_statistics_t)) != crc) {
        DEBUG_PRINT_MSG_NOW("WARNING: stats corrupted, values reset");
        memset(&stats, 0, sizeof(lwb_statistics_t));
    }
    stats.reset_cnt++;
    DEBUG_PRINT_MSG_NOW("stats loaded, reset count: %d", stats.reset_cnt);
    return 1;
#else
    return 0;
#endif /* LWB_USE_XMEM */
}
/*---------------------------------------------------------------------------*/
void lwb_stats_save(void) {
#if LWB_USE_XMEM
    stats.crc = 0;      // necessary
    stats.crc = calc_crc16((uint8_t*)&stats, sizeof(lwb_statistics_t));
    if (!xmem_write(stats_addr, sizeof(lwb_statistics_t), (uint8_t*)&stats)) {
        DEBUG_PRINT_WARNING("failed to write stats");
    }
#endif /* LWB_USE_XMEM */
}
/*---------------------------------------------------------------------------*/
void lwb_stats_reset(void) {
#if LWB_USE_XMEM
    memset(&stats, 0, sizeof(lwb_statistics_t));
    lwb_stats_save();
#endif /* LWB_USE_XMEM */
}
/*---------------------------------------------------------------------------*/
// store a received message in the incoming queue, returns 1 if successful, 0 otherwise
uint8_t 
lwb_in_buffer_put(const uint8_t * const data, uint8_t len)
{    
  // received messages will have the max. length LWB_MAX_PACKET_LEN
#if !LWB_USE_XMEM
  uint8_t* next_msg = (uint8_t*)((uint16_t)fifo_put(&in_buffer));  // assume pointers are 16-bit
  if(FIFO_ERROR != (uint16_t)next_msg) {
    // copy the data into the queue
    memcpy(next_msg, data, len);
    *(next_msg + LWB_MAX_PACKET_LEN) = len;        // last byte holds the payload length
    return 1;
  }
#else /* LWB_USE_XMEM */
  uint32_t next_msg = fifo_put(&in_buffer);
  if(FIFO_ERROR != next_msg) {
    // write the data into the queue in the external memory
    xmem_write(next_msg, len, data);
    xmem_write(next_msg + LWB_MAX_PACKET_LEN, 1, &len);
    return 1;
  }
#endif /* LWB_USE_XMEM */
  DEBUG_PRINT_ERROR("in queue full");
  return 0;
}
/*---------------------------------------------------------------------------*/
/* fetch the next 'ready-to-send' message from the outgoing queue
 * returns the message length in bytes */
uint8_t 
lwb_out_buffer_get(uint8_t* out_data)
{   
  // messages have the max. length LWB_MAX_PACKET_LEN and are already formatted according to glossy_payload_t
#if !LWB_USE_XMEM
  uint8_t* next_msg = (uint8_t*)((uint16_t)fifo_get(&out_buffer));  // assume pointers are 16-bit
  if(FIFO_ERROR != (uint16_t)next_msg) {
    memcpy(out_data, next_msg, *(next_msg + LWB_MAX_PACKET_LEN) + LWB_DATA_PKT_HEADER_LEN);
    return *(next_msg + LWB_MAX_PACKET_LEN);
  }
#else /* LWB_USE_XMEM */
  uint32_t next_msg = fifo_get(&out_buffer);
  if(FIFO_ERROR != next_msg) {
    // NOTE: make sure out_data can hold (LWB_MAX_PACKET_LEN + 1) bytes!
    xmem_read(next_msg, LWB_MAX_PACKET_LEN + 1, out_data);
    return *(out_data + LWB_MAX_PACKET_LEN);
  }
#endif /* LWB_USE_XMEM */
  DEBUG_PRINT_ERROR("out queue empty");
  return 0;
}
/*---------------------------------------------------------------------------*/
// puts a message into the outgoing queue, returns 1 if successful, 0 otherwise
uint8_t
lwb_send_data(uint16_t recipient, 
              uint8_t stream_id, 
              const uint8_t * const data, 
              uint8_t len)
{
  // data has the max. length LWB_DATA_PKT_PAYLOAD_LEN, lwb header needs to be added before the data is inserted into the queue
  if(len > LWB_DATA_PKT_PAYLOAD_LEN) {
    return 0;
  }
#if !LWB_USE_XMEM
  uint8_t* next_msg = (uint8_t*)((uint16_t)fifo_put(&out_buffer));  // assume pointers are 16-bit
  if(FIFO_ERROR != (uint16_t)next_msg) {
    *(next_msg + LWB_MAX_PACKET_LEN) = len;       // payload length
    /* recipient is always 'all sinks' */
    *(next_msg) = (uint8_t)recipient;   /* recipient L */  
    *(next_msg + 1) = recipient >> 8;   /* recipient H */  
    *(next_msg + 2) = stream_id;        /* stream id */
    memcpy(next_msg + LWB_DATA_PKT_HEADER_LEN, data, len);
    return 1;
  }
#else /* LWB_USE_XMEM */
  uint32_t next_msg = fifo_put(&out_buffer);
  if(FIFO_ERROR != next_msg) {
    *(msg_buffer + LWB_MAX_PACKET_LEN) = len;
    /* recipient is always 'all sinks' */
    *(msg_buffer) = (uint8_t)recipient;   /* recipient L */  
    *(msg_buffer + 1) = recipient >> 8;   /* recipient H */
    *(msg_buffer + 2) = stream_id;        /* stream id */
    memcpy(msg_buffer + LWB_DATA_PKT_HEADER_LEN, data, len);
    xmem_write(next_msg, LWB_MAX_PACKET_LEN + 1, msg_buffer);
    return 1;
  }
#endif /* LWB_USE_XMEM */
  DEBUG_PRINT_ERROR("out queue full");
  return 0;
}
/*---------------------------------------------------------------------------*/
// copies the oldest received message in the queue into out_data and returns the message size (in bytes)
uint8_t
lwb_rcv_data(uint8_t* out_data, uint8_t * const out_stream_id)
{ 
  // messages in the queue have the max. length LWB_MAX_PACKET_LEN, lwb header needs to be stripped off; payload has max. length LWB_DATA_PKT_PAYLOAD_LEN
#if !LWB_USE_XMEM
  uint8_t* next_msg = (uint8_t*)((uint16_t)fifo_get(&in_buffer));  // assume pointers are 16-bit
  if(FIFO_ERROR != (uint16_t)next_msg) {
    memcpy(out_data, next_msg + LWB_DATA_PKT_HEADER_LEN, *(next_msg + LWB_MAX_PACKET_LEN));
    if(out_stream_id) {
      *out_stream_id = next_msg[2];
    }
    return *(next_msg + LWB_MAX_PACKET_LEN);
  }
#else /* LWB_USE_XMEM */
  uint32_t next_msg = fifo_get(&in_buffer);
  if(FIFO_ERROR != next_msg) {
    xmem_read(next_msg, LWB_MAX_PACKET_LEN + 1, msg_buffer);
    memcpy(out_data, msg_buffer + LWB_DATA_PKT_HEADER_LEN, *(msg_buffer + LWB_MAX_PACKET_LEN));
    if(out_stream_id) {
      *out_stream_id = msg_buffer[2];
    }
    return *(msg_buffer + LWB_MAX_PACKET_LEN);
  }
#endif /* LWB_USE_XMEM */
  DEBUG_PRINT_ERROR("in queue empty");
  return 0;
}
/*---------------------------------------------------------------------------*/
uint8_t
lwb_get_rcv_buffer_status(void)
{
  return !FIFO_EMPTY(&in_buffer);
}
/*---------------------------------------------------------------------------*/
uint8_t
lwb_get_send_buffer_status(void)
{
  return !FIFO_EMPTY(&out_buffer);
}
/*---------------------------------------------------------------------------*/
const lwb_statistics_t * const
lwb_get_stats(void)
{
    return &stats;
}
/*---------------------------------------------------------------------------*/
uint8_t
lwb_request_stream(uint8_t stream_id, uint8_t ipi, int8_t offset, uint8_t urgent)
{
    urgent_stream_req = (urgent << 15) + stream_id;
    return lwb_stream_add(stream_id, ipi, offset);
}
/*---------------------------------------------------------------------------*/
/**
 * @brief thread of the host node
 */
PT_THREAD(lwb_thread_host(rtimer_t *rt)) {
    
    // all variables must be static (because this function may be interrupted at any of the LWB_WAIT_UNTIL statements)
    static lwb_schedule_t schedule;
    static rtimer_clock_t t_start, 
                          t_now, 
                          t_start_ta1;
    static glossy_payload_t glossy_payload;             // packet buffer, used to send and receive the packets
    static const uint32_t t_guard = LWB_T_GUARD;        // guard time is constant for the host 
    static uint8_t slot_idx;
    static uint8_t streams_to_update[LWB_MAX_DATA_SLOTS];
    static uint8_t schedule_len, 
                   payload_len;
    static uint8_t rcvd_data_pkts;
    static const void* callback_func = lwb_thread_host;

    // note: all statements above PT_BEGIN() will be executed each time the protothread is scheduled
    
    PT_BEGIN(&lwb_pt);     // declare variables before this statement!
        
    memset(&schedule, 0, sizeof(schedule));
    
    // initialization specific to the host node
    schedule_len = lwb_sched_init(&schedule);
    
    while (1) {
            
        t_start_ta1 = rtimer_now_ta1();
        t_start = rt->time;
        
        schedule.time += (schedule.period - 1); /* update the timestamp */
        LWB_SCHED_SET_AS_1ST(&schedule);        /* mark this schedule as first */
        LWB_SEND_SCHEDULE();         /* send the previously computed schedule */
        
        stats.relay_cnt = glossy_get_relay_cnt_first_rx();
        slot_idx = 0;       // reset the packet counter

        // --- COMMUNICATION ROUND STARTS ---
        
#if LWB_USE_XMEM
        xmem_wakeup();      // put the external memory back into active mode (takes ~500us)
#endif
        // uncompress the schedule
#if LWB_SCHED_COMPRESS
        lwb_sched_uncompress((uint8_t*)schedule.slot, LWB_SCHED_N_SLOTS(&schedule));
#endif /* LWB_SCHED_COMPRESS */
        
        // --- S-ACK SLOT ---
        
        if (LWB_SCHED_HAS_SACK_SLOT(&schedule)) {
            payload_len = lwb_sched_prep_sack(&glossy_payload.sack_pkt); 
            LWB_WAIT_UNTIL(t_start + LWB_T_SLOT_START(0));                        // wait for the slot to start
            LWB_SEND_PACKET(node_id);   // transmit s-ack
            DEBUG_PRINT_INFO("S-ACK sent (l=%d)", payload_len);
            slot_idx++;     // increment the packet counter
        } else {
            DEBUG_PRINT_VERBOSE("no sack slot");
        }
               
        // --- DATA SLOTS ---
        
        rcvd_data_pkts = 0;      // number of received data packets in this round
        if (LWB_SCHED_HAS_DATA_SLOT(&schedule)) {
            static uint8_t i = 0;
            for (i = 0; i < LWB_SCHED_N_SLOTS(&schedule); i++, slot_idx++) {
                streams_to_update[i] = LWB_INVALID_STREAM_ID;
                // is this our slot? Note: slots assigned to node ID 0 always belong to the host
                if (schedule.slot[i] == 0 || schedule.slot[i] == node_id) {
                    // send a data packet
                    payload_len = lwb_out_buffer_get(glossy_payload.raw_data);
                    if (payload_len) { 
                        // note: stream ID is irrelevant here 
                        //excess bytes = msg_size - (glossy_payload.data_pkt.len + MESSAGE_HEADER_SIZE)
                        //payload_len = ((message_t*)&glossy_payload.data_pkt)->header.len + MESSAGE_HEADER_SIZE;  // use the 'true' length                        
                        LWB_WAIT_UNTIL(t_start + LWB_T_SLOT_START(slot_idx));    // wait until the data slot starts
                        LWB_SEND_PACKET(node_id);
                        DEBUG_PRINT_INFO("message sent to %u (l=%u)", glossy_payload.data_pkt.recipient, payload_len);
                    }
                } else {
                    // receive a data packet
                    LWB_WAIT_UNTIL(t_start + LWB_T_SLOT_START(slot_idx) - t_guard);    // wait until the data slot starts
                    LWB_RCV_PACKET();
                    if (LWB_DATA_RCVD && glossy_get_payload_len()) {
                        RTIMER_CAPTURE;     // measure the time it takes to process the received message
                        if (glossy_payload.data_pkt.recipient == node_id || 
                            glossy_payload.data_pkt.recipient == LWB_RECIPIENT_BROADCAST || 
                            glossy_payload.data_pkt.recipient == 0) {
                            // is it a stream request? (piggyback on data packet)
                            if (LWB_INVALID_STREAM_ID == glossy_payload.data_pkt.stream_id) {                            
                                DEBUG_PRINT_VERBOSE("piggyback stream request from node %u (ipi=%u id=%u ofs=%d)", glossy_payload.srq_pkt.node_id, glossy_payload.srq_pkt.ipi, glossy_payload.srq_pkt.stream_id, glossy_payload.srq_pkt.t_offset);
                                lwb_sched_proc_srq((const lwb_stream_req_t*)(glossy_payload.raw_data + 3));
                            } else {
                                streams_to_update[i] = glossy_payload.data_pkt.stream_id;
                                DEBUG_PRINT_INFO("data received (s=%u.%u l=%u)", schedule.slot[i], glossy_payload.data_pkt.stream_id, glossy_get_payload_len());
                                lwb_in_buffer_put(glossy_payload.raw_data, glossy_get_payload_len());
                            }
                        } else {
                            DEBUG_PRINT_VERBOSE("packet dropped, data not destined for this node");            
                        }
                        // update statistics
                        stats.data_tot += glossy_get_payload_len();
                        stats.pck_cnt++;
                        rcvd_data_pkts++;
                        stats.t_proc_max = MAX((uint16_t)RTIMER_ELAPSED, stats.t_proc_max); // measure time (must always be smaller than LWB_T_GAP!)
                    } else {
                        DEBUG_PRINT_VERBOSE("no data received from node %u", schedule.slot[i]);
                    }
                }
            }
        }
        
        // --- CONTENTION SLOT ---
        
        if (LWB_SCHED_HAS_CONT_SLOT(&schedule)) {
            // wait until the slot starts, then receive the packet
            LWB_WAIT_UNTIL(t_start + LWB_T_SLOT_START(slot_idx) - t_guard);         // wait until the contention slot starts
            LWB_RCV_PACKET();
            if (LWB_DATA_RCVD) {
                // check the request
                DEBUG_PRINT_INFO("stream request from node %u (ipi=%u id=%u ofs=%d)", glossy_payload.srq_pkt.node_id, glossy_payload.srq_pkt.ipi, glossy_payload.srq_pkt.stream_id, glossy_payload.srq_pkt.t_offset);
                lwb_sched_proc_srq((const lwb_stream_req_t*)&glossy_payload.srq_pkt);
            }
        }
        
        // compute the new schedule
        RTIMER_CAPTURE;
        schedule_len = lwb_sched_compute(&schedule, streams_to_update, 0); // TODO: last param. was mm_status()       // check if there are any new packets to send in the next round   
        stats.t_sched_max = MAX((uint16_t)RTIMER_ELAPSED, stats.t_sched_max);

        LWB_WAIT_UNTIL(t_start + LWB_T_SCHED2_START);
        LWB_SEND_SCHEDULE();      // send the schedule for the next round
        
        // --- COMMUNICATION ROUND ENDS ---
        // -> time for other computations
        
        // print out some stats
        DEBUG_PRINT_INFO("ts=%u td=%u dp=%u d=%lu p=%u r=%u", stats.t_sched_max, stats.t_proc_max, rcvd_data_pkts, stats.data_tot, stats.pck_cnt, stats.relay_cnt);
        
#if LWB_STATS_NVMEM
        lwb_stats_save();
#endif
        debug_print_poll();     // TODO verify that this process polling works as expected
        process_poll(app_proc);

        // old code: 
        //LWB_WAIT_UNTIL(t_start + schedule.period * RTIMER_SECOND);
        //rt->time = t_start + schedule.period * RTIMER_SECOND;
                  
        // modified code:       TODO verify
        // suspend this task and wait for the next round (disable HF crystal during this phase!)
        LWB_SLEEP_UNTIL(t_start_ta1 + schedule.period * RTIMER_SECOND_TA1);
        
        rt->time = rtimer_now(); 
    }
    
    PT_END(&lwb_pt);
}
/*---------------------------------------------------------------------------*/
/**
 * @brief declaration of the protothread (source node)
 */
PT_THREAD(lwb_thread_src(rtimer_t *rt)) {
    
    // all variables must be static    
    static lwb_schedule_t schedule;
    static sync_state_t sync_state;
    static rtimer_clock_t t_ref, 
                          t_ref_last, 
                          t_start, 
                          t_now, 
                          t_start_ta1;                          // note: t_start != t_ref
    static glossy_payload_t glossy_payload;                     // packet buffer, used to send and receive the packets
    static uint32_t t_guard;                                    // 32-bit is enough for t_guard!
    static int16_t drift_last     = 0;
    static int32_t drift          = 0;
    static uint8_t slot_idx;
    static uint8_t payload_len;
    static uint8_t rounds_to_wait = 0; 
    static const void* callback_func = lwb_thread_src;
    
    PT_BEGIN(&lwb_pt);     // declare variables before this statement!
    
    memset(&schedule, 0, sizeof(schedule)); 
    
    // initialization specific to the source node
    lwb_stream_init();
    sync_state        = BOOTSTRAP;
    stats.period_last = LWB_SCHED_PERIOD_MIN;
    
    while (1) {
    
        if (sync_state == BOOTSTRAP) {
            DEBUG_PRINT_MSG_NOW("BOOTSTRAP ");
            stats.bootstrap_cnt++;
            drift_last = 0;
            lwb_stream_rejoin();    // rejoin all (active) streams
            // synchronize first! wait for the first schedule...
            do {
                LWB_RCV_SCHEDULE();
                if (rt->time - t_ref > LWB_T_SILENT) {
                    //DEBUG_PRINT_MSG_NOW("no communication for %ds, enabling fail-over policy..", LWB_T_SILENT);
                    // TODO: implement host fail-over
                }
            } while (!glossy_is_t_ref_updated() || !LWB_SCHED_IS_1ST(&schedule));
            // schedule received!
        } else {
            LWB_RCV_SCHEDULE();    
        }
        
        // --- COMMUNICATION ROUND STARTS ---
        
        t_start_ta1 = rtimer_now_ta1();  
#if LWB_USE_XMEM
        xmem_wakeup();      // put the external memory back into active mode (takes ~500us)
#endif
                   
        // update the sync state machine (compute new sync state and update t_guard)
        LWB_UPDATE_SYNC_STATE;  
        if (BOOTSTRAP == sync_state) {
            // something went wrong
            continue;
        } 
        LWB_SCHED_SET_AS_2ND(&schedule);
        if (glossy_is_t_ref_updated()) {
            t_ref   = glossy_get_t_ref();   
            t_start = t_ref - LWB_T_REF_OFS;
            t_start_ta1 = rtimer_now_ta1();
        } else {
            // do not update t_ref, estimate t_start
            t_start = t_ref - LWB_T_REF_OFS + schedule.period * (RTIMER_SECOND + drift_last);
        }
        
        // permission to participate in this round?
        if (sync_state == SYNCED || sync_state == UNSYNCED_1) {
                            
            slot_idx = 0;   // reset the packet counter
            stats.relay_cnt = glossy_get_relay_cnt_first_rx();         
#if LWB_SCHED_COMPRESS
            lwb_sched_uncompress((uint8_t*)schedule.slot, LWB_SCHED_N_SLOTS(&schedule));
#endif /* LWB_SCHED_COMPRESS */
            
            // --- S-ACK SLOT ---
            
            if (LWB_SCHED_HAS_SACK_SLOT(&schedule)) {   
                LWB_WAIT_UNTIL(t_start + LWB_T_SLOT_START(0) - t_guard);         // wait for the slot to start
                LWB_RCV_PACKET();                                 // receive s-ack                    
                if (LWB_DATA_RCVD) {                    
                    /* TODO: maybe forward this message to the application? or let app poll? */
                    static uint8_t i;       // must be static
                    i = 0;                  // must be a separate line of code!
                    DEBUG_PRINT_INFO("S-ACK packet received with %u stream acks in total", glossy_payload.sack_pkt.n_extra + 1);
                    do {
                        if (*(uint16_t*)(glossy_payload.raw_data + (i * 4)) == node_id) {       // potential cause of problems (BUT: structure glossy_payload should be aligned to an even address!)
                            uint8_t stream_id = *(uint8_t*)(glossy_payload.raw_data + (i * 4 + 2));
                            stats.t_slot_last = schedule.time;
                            rounds_to_wait = 0;
                            if (lwb_stream_update(stream_id)) {
                                DEBUG_PRINT_INFO("S-ACK received for stream %u (joined)", stream_id);
                            } else {
                                DEBUG_PRINT_INFO("S-ACK received for stream %u (removed)", stream_id);
                            }  
                        } 
                        i++;
                    } while (i <= glossy_payload.sack_pkt.n_extra);
                } else {
                    DEBUG_PRINT_VERBOSE("no data received in SACK SLOT");
                }
                slot_idx++;     // increment the packet counter
            }
            
            // --- DATA SLOTS ---
            
            if (LWB_SCHED_HAS_DATA_SLOT(&schedule)) {
                static uint8_t i;   // must be static because the for-loop might be interrupted and state of i would be lost
                for (i = 0; i < LWB_SCHED_N_SLOTS(&schedule); i++, slot_idx++) {
                    if (schedule.slot[i] == node_id) {
                        // this is our data slot, send a data packet
                        RTIMER_CAPTURE;         // measure the time it takes to prepare the packet
                        payload_len = 0;
                        /* is there an 'urgent' stream request? -> if so, piggyback it onto the data packet */
                        if (urgent_stream_req) {
                          urgent_stream_req &= 0x7f;    /* clear the last bit */
                          if (lwb_stream_prep_req((lwb_stream_req_t*)&glossy_payload.srq_pkt, urgent_stream_req)) {
                              payload_len = sizeof(lwb_stream_req_t);
                          }
                          glossy_payload.srq_pkt.stream_id = LWB_INVALID_STREAM_ID;
                        } else {
                          glossy_payload.data_pkt.recipient = 0;
                          glossy_payload.data_pkt.stream_id = 0;
                          payload_len = lwb_out_buffer_get(glossy_payload.raw_data);      // fetch the next 'ready-to-send' packet
                        }
                        if (payload_len) {
                            stats.t_prep_max = MAX(RTIMER_ELAPSED, stats.t_prep_max);
                            stats.t_slot_last = schedule.time;
                            LWB_WAIT_UNTIL(t_start + LWB_T_SLOT_START(slot_idx));
                            LWB_SEND_PACKET(node_id);
                            DEBUG_PRINT_INFO("message sent (l=%u)", payload_len);
                        } else {                            
                            DEBUG_PRINT_WARNING("no message to send (data slot ignored)");
                        }
                    } else {
                        // receive a data packet
                        LWB_WAIT_UNTIL(t_start + LWB_T_SLOT_START(slot_idx) - t_guard);
                        LWB_RCV_PACKET();
                        
                        // process the received data
                        if (LWB_DATA_RCVD && glossy_get_payload_len()) {
                            RTIMER_CAPTURE;         // measure the time it takes to process the received data
                            // only forward packets that are destined for this node
                            if (glossy_payload.data_pkt.recipient == node_id || 
                                glossy_payload.data_pkt.recipient == LWB_RECIPIENT_BROADCAST) {
                                DEBUG_PRINT_INFO("data received");
                                lwb_in_buffer_put(glossy_payload.raw_data, glossy_get_payload_len());
                            } else {
                                DEBUG_PRINT_VERBOSE("received packet dropped");            
                            }
                            stats.t_proc_max = MAX((uint16_t)RTIMER_ELAPSED, stats.t_proc_max); // must always be smaller than LWB_T_GAP
                        } // else: no data received                        
                    }
                    stats.data_tot += glossy_get_payload_len(); 
                    stats.pck_cnt++;
                }
            }            
            
            // --- CONTENTION SLOT ---
            
            if (LWB_SCHED_HAS_CONT_SLOT(&schedule)) {   // is there a contention slot in this round?                
                // does this node have pending stream requests?
                if (LWB_STREAM_REQ_PENDING) {
                    if (!rounds_to_wait) {      // allowed to send the request?
                        // try to join, send the stream request
                        if (lwb_stream_prep_req((lwb_stream_req_t*)glossy_payload.raw_data, LWB_INVALID_STREAM_ID)) {
                            rounds_to_wait = (random_rand() >> 1) % 8 + 1;  // wait between 1 and 8 rounds
                            payload_len = sizeof(lwb_stream_req_t);
                            LWB_WAIT_UNTIL(t_start + LWB_T_SLOT_START(slot_idx));    // wait until the contention slot starts
                            LWB_SEND_PACKET(node_id);  
                            DEBUG_PRINT_INFO("stream request sent (id=%u ipi=%u ofs=%d)", glossy_payload.srq_pkt.stream_id, glossy_payload.srq_pkt.ipi, glossy_payload.srq_pkt.t_offset);
                        } else {
                            DEBUG_PRINT_ERROR("failed to prepare stream request packet");
                        }
                    } else {
                        DEBUG_PRINT_VERBOSE("must wait %u rounds", rounds_to_wait);
                        // keep waiting and just relay incoming packets
                        rounds_to_wait--;              // decrease the number of rounds to wait
                        LWB_WAIT_UNTIL(t_start + LWB_T_SLOT_START(slot_idx) - t_guard);    // wait until the contention slot starts
                        LWB_RCV_PACKET();
                    }
                } else {
                    // no request pending -> just receive / relay packets
                    LWB_WAIT_UNTIL(t_start + LWB_T_SLOT_START(slot_idx) - t_guard);    // wait until the contention slot starts
                    LWB_RCV_PACKET();
                }
            }
        }
        
        // wait for the schedule at the end of the round
        LWB_WAIT_UNTIL(t_start + LWB_T_SCHED2_START - t_guard);
        LWB_RCV_SCHEDULE();
        
        // --- COMMUNICATION ROUND ENDS ---
        // -> time for other computations
                
        // update the state machine and the guard time
        LWB_UPDATE_SYNC_STATE;
        if (BOOTSTRAP == sync_state) {
            // something went wrong
            if (LWB_SCHED_IS_1ST(&schedule)) {
                DEBUG_PRINT_ERROR("wrong schedule received!");
            }
            // this happened probably due to a host failure
            continue;
        }
        // check joining state
        if (LWB_STREAMS_ACTIVE && (schedule.time - stats.t_slot_last) > (LWB_SCHED_PERIOD_MAX << 1)) {
            DEBUG_PRINT_WARNING("no-slot timeout, requesting new stream...");
            lwb_stream_rejoin();    // re-join (all streams)
            stats.t_slot_last = schedule.time;
        }        
        
        // update the stats and estimate the clock drift (clock cycles per second)
        drift             = (int32_t)((t_ref - t_ref_last) - ((int32_t)stats.period_last * RTIMER_SECOND)) / (int32_t)stats.period_last;
        stats.period_last = schedule.period;
        t_ref_last        = t_ref; 
        if (sync_state > ALREADY_SYNCED) {
            stats.unsynced_cnt++;
        }
        // print out some stats (note: takes approx. 2ms to compose this string)
        DEBUG_PRINT_INFO("%s %lu T=%u n=%u s=%u td=%u tp=%u d=%lu p=%u r=%u b=%u u=%u ds=%d s=%d", sync_state_to_string[sync_state], schedule.time, schedule.period, LWB_SCHED_N_SLOTS(&schedule), LWB_STREAMS_ACTIVE, stats.t_proc_max, stats.t_prep_max, stats.data_tot, stats.pck_cnt, stats.relay_cnt, stats.bootstrap_cnt, stats.unsynced_cnt, (drift_last - (int16_t)drift), drift_last);
        if (ALREADY_SYNCED == sync_state || UNSYNCED_1 == sync_state) {
            if ((drift < LWB_MAX_CLOCK_DEV) && (drift > -LWB_MAX_CLOCK_DEV)) {
                drift_last = (int16_t)drift;
            } else if (drift_last) {    // only if not zero
                // most probably a timer update overrun (or a host failure)
                // usually, the deviation per second is not higher than 50 cycles; if only one timer update is missed in 30 seconds, the deviation per second is still more than 1k cycles and therefore detectable
                DEBUG_PRINT_WARNING("Critical timing error (timer update overrun?)");
            }
        }
        
#if defined(ASYNC_INT_TIMEREQ_POLLING) && !defined(FLOCKLAB)
        if (async_int_timereq_pending(&glossy_payload.data_pkt.payload[2])) {       // check for timestamp request      
            glossy_payload.data_pkt.payload[0] = CMD_CODE_TIMESTAMP;                // command code
            SET_CTRL_MSG(&glossy_payload.data_pkt);
            glossy_payload.data_pkt.len  = 10; 
            //TODO: mm_put((message_t*)glossy_payload.raw_data);
        }
#endif // ASYNC_INT_TIMEREQ_POLLING

#if LWB_STATS_NVMEM
        lwb_stats_save();
#endif  
        // erase the schedule (slot allocations only)
        memset(&schedule.slot, 0, sizeof(schedule.slot));

        // suspend this task and let other tasks run
        debug_print_poll();
        process_poll(app_proc);
        
        // TODO verify
        // old code: LWB_WAIT_UNTIL(t_start + schedule.period * (RTIMER_SECOND + drift_last) - t_guard);
        // new code:
        LWB_SLEEP_UNTIL(t_start_ta1 + schedule.period * RTIMER_SECOND_TA1);  // NOTE: drift compensation not feasible with LF crystal
    }

    PT_END(&lwb_pt);
}
/*---------------------------------------------------------------------------*/
/* define the process control block */
PROCESS(lwb_process, "Communication Task (LWB)");
/*---------------------------------------------------------------------------*/
/* define the body (protothread) of a process */
PROCESS_THREAD(lwb_process, ev, data) {

  PROCESS_BEGIN();
  
  DEBUG_PRINT_INFO("T_HOP: %ums, T_SLOT_MIN: %ums, T_ROUND_MAX: %ums", (uint16_t)(LWB_T_HOP / 3250), (uint16_t)(LWB_T_SLOT_MIN / 3250), (uint16_t)(LWB_T_ROUND_MAX / 3250));

  lwb_stats_load();     // load the stats from the external memory   
#if !LWB_USE_XMEM
  /* pass the start addresses of the memory blocks holding the queues */
  fifo_init(&in_buffer, (uint16_t)in_buffer_mem);
  fifo_init(&out_buffer, (uint16_t)out_buffer_mem); 
#else  /* LWB_USE_XMEM */
  /* allocate memory for the message buffering (in ext. memory) */
  fifo_init(&in_buffer, xmem_alloc(LWB_IN_BUFFER_SIZE * (LWB_MAX_PACKET_LEN + 1)));
  fifo_init(&out_buffer, xmem_alloc(LWB_OUT_BUFFER_SIZE * (LWB_MAX_PACKET_LEN + 1)));   
#endif /* LWB_USE_XMEM */
  
  PT_INIT(&lwb_pt); /* initialize the protothread */
  if(node_id == HOST_ID) {
    rtimer_schedule(LWB_RTIMER_ID, rtimer_now(), 0, lwb_thread_host);
  } else {
    rtimer_schedule(LWB_RTIMER_ID, rtimer_now(), 0, lwb_thread_src);
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
void
lwb_start(void *application_proc)
{
  app_proc = (struct process*)application_proc;
  printf("Starting '%s'\r\n", lwb_process.name);
  process_start(&lwb_process, NULL);
}
/*---------------------------------------------------------------------------*/
