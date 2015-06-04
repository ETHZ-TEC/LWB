/**
 * @file
 *
 * Implementation steps:
 * - simple LWB runs (empty schedule, fixed period, 1 source and 1 host)
 * - 2nd schedule at the end of a round added
 * - pseudo-random period length added
 * - TX power reduced to -12 dBm, channel switched to 10
 * - clock drift compensation added
 * - sync state machine added (NOT yet verified)
 * - data slots added (test data sent and received successfully)
 * - macros added to reduce code size and make it more readable
 * - contention slot added (joining state machine not yet implemented)
 * - s-ack slot added
 * - simplified joining scheme added (see assumptions)
 * - FRAM memory integrated
 * - a simplified scheduler integrated (but not yet tested)
 * - slot lengths adjusted
 * - basic compression method from original lwb code added
 * - ADC sampling added to generate some 'real' data
 * - data packets buffered on FRAM
 * - fall-back to state NOT_JOINED when no slot received within a certain time
 * - host failure detection added
 * - start time offset added for requests
 * - compression algorithm modified and verified
 * - more stats added, debug output improved
 * - node ID is now 16 bits wide (-> max. number of slots limited to 59)
 * - recipient ID added for data packets
 * - union struct for payload added to avoid pointer aliasing problems (introduced by compiler optimization level -O2)
 * - buffer data packets in FRAM upon reception (i.e. between the data slots)
 * - bug fixed: pin 1.2 mustn't be configured as output! (otherwise, no data will be received from the FRAM)
 * - store the stream information on the external memory
 * - piggyback stream requests on data packets
 * - support for several streams per node added
 * - two separate functions for host and source node
 * - sack packet format changed to allow up to 31 sack's per round
 * - buffer debug print messages on external memory (Note: buffering to ext. mem takes > 2ms per msg -> increase gap time!)
 * - SMCLK now fed by XT2, SMCLK speed is 13 MHz, SPI speed for FRAM is now 13 MHz (26 does not work)
 * - port access macros simplified (note: PINx definitions changed!)
 * - support for comm board v1.0 added
 * - START_LWB macro modified: code size reduced, but a node is now either source OR host (a source node cannot become a host node)
 * - reset info print out added to contiki-cc430-main.c
 * - asynchronous data interface (ADI) integrated: source reads from AI and the host write to the ADI
 * - switching betw. SPI and UART mode reduced to a minimum of reconfiguration (reinit functions and check if sync flag is set)
 * - host can now allocate slots to itself
 * - field 'seq_no' removed from schedule (unused) to align the slots with the 16-bit word boundary
 * - control packets added (host can sent commands to source nodes)
 * - TA1 timers are now clocked by ACLK!
 * - timestamp request pin added
 * - only one software extension for both timers! (have the same clock and update speed) Reduces the overhead, only 1 update interrupt! NOTE: both timers must start at the same time and mustn't be paused!
 * - T_REF_OFFSET added to align glossy active pulses on host and source
 * - guard times now whenever a SOURCE node is receiving a packet (no guard times for the host or sending nodes); Host uses only T_GUARD (constant value) for reception of data/stream req. packets
 * - t_start introduced for source nodes
 * - occasional "unsynced" / "a_synced" problem solved (guard times before each slot added, t_start introduced)
 * - timer update interrupt disabled during the glossy flood
 * - timer update overrun detection added for the source node (how to handle it on the host node??)
 * - stream information is now updated at the end of the round (instead of searching for stream between the data slots)
 * - minor error in state machine fixed (did not enter a_synced at end of round, invalid state transitions handled -> back to bootstrap)
 * - sprintf is very inefficient (~2000 cycles to copy a string) -> use memcpy instead
 * - problem with invalid debug prints solved (characters were omitted due to SPI_FAST_READ)
 * - CRC problem and Glossy RX/TX Error fixed: execution time between slots was too long (remove debug print statements)
 * - packet format of a message and command changed, stats struct changed
 * - parts of the code restructured and cleaned up
 * - principle of token added (credit-based flow control): a processor (A or C) gets 'credits' (permission) from the other processor to write messages into the ADI queue
 * - outer glossy data_pkt removed, now simpler, smaller and more logical
 * - message manager added
 * - information about streams of a node is now stored locally
 * - pre-processing time added (before a round starts to decrease the delay)
 * - timestamp req pin: trigger on rising edge only
 * - undesired interrupts disabled during a glossy flood and only re-enabled if it was enabled before
 * - toggling the timestamp request pin at a high rate won't generate many timestamp packets; instead, only the first request is handled (interrupt re-enabled at the end of each round)
 * - credit-based message manager stored in deprecated.c
 * - FlockLAB test program integrated: source nodes allocate a stream every few rounds (randomized ipi), data is generated locally (no ADI); all data on source node is kept in SRAM (no access to FRAM)
 * - stats are now stored in non-volatile memory (FRAM)
 * - message manager added on host side
 * - timer removed from ADI driver (wait a fixed number of cycles for the ACK signal)
 * - reset stats command added
 * - nasty pointer misalignment bug fixed (memcpy() to copy node_id into pending_sack buffer)
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
 

#include "s-lwb.h"


static struct pt    slwb_pt;                // the proto-thread 
static statistics_t stats = { 0 };          // some statistics
static uint32_t     stats_addr = 0;
    

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


/** 
 * @brief returns the latest event
 */
#define GET_EVENT       ( glossy_is_t_ref_updated() ? (SCHED_IS_1ST(&schedule) ? EVT_1ST_SCHED_RCVD : EVT_2ND_SCHED_RCVD) : EVT_SCHED_MISSED )

/**
 * @brief sends the schedule packet (for host only)
 */
#define SEND_SCHEDULE(thread) \
{\
    glossy_start(node_id, (uint8_t *)&schedule, schedule_len, N_TX_MAX_SCHEDULE, GLOSSY_WITH_SYNC, GLOSSY_WITH_RF_CAL);\
    WAIT_UNTIL(rt->time + T_SCHED, thread);\
    glossy_stop();\
}
   
/**
 * @brief receives a schedule packet (for source nodes only)
 */ 
#define RCV_SCHEDULE(thread) \
{\
    glossy_start(0, (uint8_t *)&schedule, 0, N_TX_MAX_SCHEDULE, GLOSSY_WITH_SYNC, GLOSSY_WITH_RF_CAL);\
    WAIT_UNTIL(rt->time + T_SCHED + t_guard, thread);\
    glossy_stop();\
}
   
/**
 * @brief sends a data packet
 */ 
#define SEND_DATA_PACKET(ini, data, len, offset, thread) \
{\
    glossy_start(ini, (uint8_t*)data, len, N_TX_MAX_DATA, GLOSSY_WITHOUT_SYNC, GLOSSY_WITHOUT_RF_CAL);\
    WAIT_UNTIL(rt->time + T_DATA + offset, thread);\
    glossy_stop();\
}
                        
/**
 * @brief suspends the lwb proto-thread until the rtimer reaches the specified timestamp time
 */
#define WAIT_UNTIL(time, thread) \
{\
    rtimer_schedule(LWB_RTIMER_ID, time, 0, thread);\
    LWB_TASK_SUSPENDED;\
    PT_YIELD(&slwb_pt);\
    LWB_TASK_ACTIVE;\
}
    
/**
 * @brief computes the new sync state and updates the guard time
 */
#define COMPUTE_SYNC_STATE \
{\
    sync_state = next_state[GET_EVENT][sync_state];     /* get the new state based on the event */\
    if (UNSYNCED_1 == sync_state) {     /* adjust the guard time */\
        t_guard = T_GUARD_1;\
    } else if (UNSYNCED_2 == sync_state) {\
        t_guard = T_GUARD_2;\
    } else if (UNSYNCED_3 == sync_state) {\
        t_guard = T_GUARD_3;\
    } else {\
        t_guard = T_GUARD;\
    }\
} 


/**
 * @brief load statistics from the external memory
 */
void stats_load(void) {
    uint16_t crc;
    fram_init();
    stats_addr = fram_alloc(sizeof(statistics_t));
    if (FRAM_ALLOC_ERROR == stats_addr || !fram_read(stats_addr, sizeof(statistics_t), (uint8_t*)&stats)) {
        DEBUG_PRINT_WARNING("failed to load stats");
    }
    crc = stats.crc;
    stats.crc = 0;
    if (calc_crc16((uint8_t*)&stats, sizeof(statistics_t)) != crc) {
        DEBUG_PRINT_WARNING("stats corrupted, values reset");
        memset(&stats, 0, sizeof(statistics_t));
    }
    stats.reset_cnt++;
    DEBUG_PRINT_INFO("stats loaded, reset count: %d", stats.reset_cnt);
}

/**
 * @brief store the statistics in the external memory
 */
void stats_save(void) {
    stats.crc = 0;
    stats.crc = calc_crc16((uint8_t*)&stats, sizeof(statistics_t));
    if (!fram_write(stats_addr, sizeof(statistics_t), (uint8_t*)&stats)) {
        DEBUG_PRINT_WARNING("failed to write stats");
    }
}

/**
 * @brief reset the statistics in the external memory
 */
void stats_reset(void) {
    memset(&stats, 0, sizeof(statistics_t));
    stats_save();
}



/**
 * @brief thread of the host node
 */
PT_THREAD(slwb_thread_host(rtimer_t *rt)) {
    
    // all variables must be static (because this function may be interrupted at any of the WAIT_UNTIL statements)
    static schedule_t schedule;
    static rtimer_clock_t t_start, t_now;
    static uint8_t slot_idx;
    static glossy_payload_t glossy_payload;             // packet buffer, used to send and receive the packets
    static uint8_t streams_to_update[N_SLOTS_MAX];
    static uint8_t schedule_len, payload_len;
    static uint8_t rcvd_data_pkts;

    PT_BEGIN(&slwb_pt);     // declare variables before this statement!
    
    memset(&schedule, 0, sizeof(schedule));
    
    // initialization specific to the host node
    schedule_len = sched_init(&schedule);
    
    LED_ON(LED_STATUS);
          
    while (1) {
    
        // pre-processing
#ifdef T_PREPROCESS
        mm_fill(glossy_payload.raw_data);                 // read and process all the messages from the ADI
        WAIT_UNTIL(rt->time + T_PREPROCESS, slwb_thread_host);    // wait until the round starts
#endif // T_PREPROCESS
        
        t_start = rt->time;
        PREPARE_1ST_SCHED(schedule);            // prepare the schedule (update time, mark it as the first schedule)
        SEND_SCHEDULE(slwb_thread_host);        // send the previously computed schedule 
        
        stats.relay_cnt = glossy_get_relay_cnt_first_rx();
        slot_idx = 0;       // reset the packet counter

        // --- COMMUNICATION ROUND STARTS ---
        
        fram_wakeup();      // put the external memory back into active mode (takes ~500us)

        // uncompress the schedule
#ifdef COMPRESS_SCHEDULE
        sched_uncompress((uint8_t*)schedule.slot, SCHED_N_SLOTS(&schedule));
#endif // COMPRESS_SCHEDULE
        
        // --- S-ACK SLOT ---
        
        if (SCHED_HAS_SACK_SLOT(&schedule)) {
            payload_len = sched_prep_sack(&glossy_payload); 
            WAIT_UNTIL(t_start + T_SLOT_START(0), slwb_thread_host);                        // wait for the slot to start
            SEND_DATA_PACKET(node_id, &glossy_payload, payload_len, 0, slwb_thread_host);   // transmit s-ack
            DEBUG_PRINT_INFO("S-ACK sent (l=%d)", payload_len);
            slot_idx++;     // increment the packet counter
        } else {
            DEBUG_PRINT_VERBOSE("no sack slot");
        }
               
        // --- DATA SLOTS ---
        
        rcvd_data_pkts = 0;      // number of received data packets in this round
        if (SCHED_HAS_DATA_SLOT(&schedule)) {
            static uint8_t i = 0;
            for (i = 0; i < SCHED_N_SLOTS(&schedule); i++, slot_idx++) {
                streams_to_update[i] = INVALID_STREAM_ID;
                // is this our slot? Note: slots assigned to node ID 0 always belong to the host
                if (schedule.slot[i] == 0 || schedule.slot[i] == node_id) {
                    // send a data packet
                    //uint16_t msg_size = 0;    
                    //ASYNC_INT_READ(glossy_payload.raw_data, msg_size);  // load a message from the asynchronous data interface
                    payload_len = mm_get((message_t*)glossy_payload.raw_data);    // fetch the next 'ready-to-send' packet
                    if (payload_len) { 
                        // note: stream ID is irrelevant here 
                        //excess bytes = msg_size - (glossy_payload.data_pkt.len + MESSAGE_HEADER_SIZE)
                        //payload_len = ((message_t*)&glossy_payload.data_pkt)->header.len + MESSAGE_HEADER_SIZE;  // use the 'true' length                        
                        WAIT_UNTIL(t_start + T_SLOT_START(slot_idx), slwb_thread_host);    // wait until the data slot starts
                        SEND_DATA_PACKET(node_id, &glossy_payload, payload_len, 0, slwb_thread_host);
                        DEBUG_PRINT_INFO("message sent to %u (l=%u)", glossy_payload.data_pkt.recipient, payload_len);
                    }
                } else {
                    // receive a data packet
                    WAIT_UNTIL(t_start + T_SLOT_START(slot_idx) - T_GUARD, slwb_thread_host);    // wait until the data slot starts
                    SEND_DATA_PACKET(schedule.slot[i], &glossy_payload, 0, T_GUARD, slwb_thread_host);                    
                    RTIMER_CAPTURE;
                    if (DATA_RECEIVED && glossy_get_payload_len()) {
                        if (glossy_payload.data_pkt.recipient == node_id || 
                            glossy_payload.data_pkt.recipient == RECIPIENT_BROADCAST || 
                            glossy_payload.data_pkt.recipient == 0) {
                            // is it a stream request? (piggyback on data packet)
                            if (INVALID_STREAM_ID == glossy_payload.data_pkt.stream_id) {                            
                                DEBUG_PRINT_VERBOSE("piggyback stream request from node %u (ipi=%u id=%u ofs=%d)", glossy_payload.srq_pkt.node_id, glossy_payload.srq_pkt.ipi, glossy_payload.srq_pkt.stream_id, glossy_payload.srq_pkt.t_offset);
                                sched_proc_srq((const stream_request_t*)(glossy_payload.raw_data + 3));
                            } else {
                                streams_to_update[i] = glossy_payload.data_pkt.stream_id;
                                DEBUG_PRINT_INFO("data received (s=%u.%u l=%u)", schedule.slot[i], glossy_payload.data_pkt.stream_id, glossy_get_payload_len());
                                // put the received message into the async interface queue (blocking call)   TODO: think about the 'queue full' problem
                                //ASYNC_INT_WRITE(glossy_payload.raw_data, len);
#ifndef FLOCKLAB
                                mm_put((message_t*)glossy_payload.raw_data);  
#endif // FLOCKLAB
                            }
                        } else {
                            DEBUG_PRINT_VERBOSE("packet dropped, data not destined for this node");            
                        }                        
                        stats.data_tot += glossy_get_payload_len();
                        stats.pck_cnt++;
                        rcvd_data_pkts++;
                    } else {
                        DEBUG_PRINT_VERBOSE("no data received from node %u", schedule.slot[i]);
                    }
                    stats.t_proc_max = MAX((uint16_t)RTIMER_ELAPSED, stats.t_proc_max); // measure time (must always be smaller than T_GAP!)
                }
            }
        }
        
        // --- CONTENTION SLOT ---
        
        if (SCHED_HAS_CONT_SLOT(&schedule)) {
            WAIT_UNTIL(t_start + T_SLOT_START(slot_idx) - T_GUARD, slwb_thread_host);         // wait until the contention slot starts
            SEND_DATA_PACKET(GLOSSY_UNKNOWN_INITIATOR, &glossy_payload, 0, T_GUARD, slwb_thread_host);
            if (DATA_RECEIVED) {
                // check the request
                DEBUG_PRINT_INFO("stream request from node %u (ipi=%u id=%u ofs=%d)", glossy_payload.srq_pkt.node_id, glossy_payload.srq_pkt.ipi, glossy_payload.srq_pkt.stream_id, glossy_payload.srq_pkt.t_offset);
                sched_proc_srq((const stream_request_t*)&glossy_payload.srq_pkt);
            }
        }
        
        // compute the new schedule
        RTIMER_CAPTURE;
        schedule_len = sched_compute(&schedule, streams_to_update, mm_status());       // check if there are any new packets to send in the next round   
        stats.t_sched_max = MAX((uint16_t)RTIMER_ELAPSED, stats.t_sched_max);

        WAIT_UNTIL(t_start + T_SCHED2_START, slwb_thread_host);
        SEND_SCHEDULE(slwb_thread_host);      // send the schedule for the next round
        
        // --- COMMUNICATION ROUND ENDS ---

        // time for other computations
        
#ifndef T_PREPROCESS        
        mm_fill(glossy_payload.raw_data);     // process messages in ADI queue (if any)
#endif // T_PREPROCESS
#ifndef FLOCKLAB
        mm_flush(glossy_payload.raw_data);    // send the buffered messages over the ADI (if enough credit avaiable)
#endif

        // print out some stats
        DEBUG_PRINT_INFO("ts=%u td=%u dp=%u d=%lu p=%u r=%u", stats.t_sched_max, stats.t_proc_max, rcvd_data_pkts, stats.data_tot, stats.pck_cnt, stats.relay_cnt);

#ifdef STORE_STATS_XMEM
        stats_save();
#endif
        fram_sleep();           // put the external memory into LPM
        debug_process_poll();   // wake up the debug process 
        // suspend this task and wait for the next round 
#ifdef T_PREPROCESS
        WAIT_UNTIL(t_start + schedule.period * RTIMER_SECOND - T_PREPROCESS, slwb_thread_host);
#else
        WAIT_UNTIL(t_start + schedule.period * RTIMER_SECOND, slwb_thread_host);
#endif // T_PREPROCESS
    }
    
    PT_END(&slwb_pt);
}


/**
 * @brief thread of the source node
 */
PT_THREAD(slwb_thread_source(rtimer_t *rt)) {
    
    // all variables must be static    
    static schedule_t schedule;
    static sync_state_t sync_state;
    static rtimer_clock_t t_ref, t_ref_last, t_start, t_now;    // note: t_start != t_ref
    static uint32_t t_guard;                                    // 32-bit is enough for t_guard!
    static uint8_t slot_idx;
    static glossy_payload_t glossy_payload;                     // packet buffer, used to send and receive the packets
    static uint8_t payload_len;
    static uint8_t rounds_to_wait = 0; 
    static int16_t drift_last     = 0;
    static int32_t drift          = 0;
#ifdef FLOCKLAB
    static uint16_t round_count   = 0;
#endif // FLOCKLAB
    
    PT_BEGIN(&slwb_pt);     // declare variables before this statement!
    
    memset(&schedule, 0, sizeof(schedule)); 
    
    // initialization specific to the source node
    stream_init();
    sync_state        = BOOTSTRAP;
    stats.period_last = PERIOD_MIN;
    
    LED_ON(LED_STATUS);
        
    while (1) {
                
        // pre-processing
#ifdef T_PREPROCESS
        mm_fill(glossy_payload.raw_data);                               // read and process all the messages from the ADI
        WAIT_UNTIL(rt->time + T_PREPROCESS, slwb_thread_source);        // wait until the round starts
#endif // T_PREPROCESS
    
        if (sync_state == BOOTSTRAP) {
            DEBUG_PRINT_NOW("BOOTSTRAP ");
            stats.bootstrap_cnt++;
            drift_last = 0;
            stream_rejoin();    // rejoin all (active) streams
            // synchronize first! wait for the first schedule...
            do {
                RCV_SCHEDULE(slwb_thread_source);
                if (rt->time - t_ref > T_NO_COMM) {
                    //DEBUG_PRINT_NOW("no communication for %ds, enabling fail-over policy..\n", T_NO_COMM);
                    // TODO: implement host fail-over
                }
            } while (!glossy_is_t_ref_updated() || !SCHED_IS_1ST(&schedule));
            // schedule received!
            DEBUG_PRINT_NOW("\r\n");
        } else {
            RCV_SCHEDULE(slwb_thread_source);    
        }
        
        // --- COMMUNICATION ROUND STARTS ---
        
        fram_wakeup();      // put the external memory back into active mode (takes ~500us)
                   
        // update the sync state machine (compute new sync state and update t_guard)
        COMPUTE_SYNC_STATE;  
        if (BOOTSTRAP == sync_state) {
            // something went wrong
            continue;
        } 
        CLEAR_LAST_BIT(schedule.period);
        if (glossy_is_t_ref_updated()) {
            t_ref   = glossy_get_t_ref();   
            t_start = t_ref - T_REF_OFFSET;
        } else {
            // do not update t_ref, estimate t_start
            t_start = t_ref - T_REF_OFFSET + schedule.period * (RTIMER_SECOND + drift_last);
        }
        
        // do we have permission to participate in this round?
        if (sync_state == SYNCED || sync_state == UNSYNCED_1) {
                            
            slot_idx = 0;   // reset the packet counter
            stats.relay_cnt = glossy_get_relay_cnt_first_rx();         
#ifdef COMPRESS_SCHEDULE
            sched_uncompress((uint8_t*)schedule.slot, SCHED_N_SLOTS(&schedule));
#endif // COMPRESS_SCHEDULE
            
            // --- S-ACK SLOT ---
            
            if (SCHED_HAS_SACK_SLOT(&schedule)) {   
                WAIT_UNTIL(t_start + T_SLOT_START(0) - t_guard, slwb_thread_source);                    // wait for the slot to start
                SEND_DATA_PACKET(GLOSSY_UNKNOWN_INITIATOR, &glossy_payload, 0, t_guard, slwb_thread_source);   // receive s-ack                    
                if (DATA_RECEIVED) { 
                    static message_t msg;
                    static uint8_t i;       // must be static
                    i = 0;                  // must be a separate line of code!
                    DEBUG_PRINT_VERBOSE("%u additional S-ACK's", glossy_payload.sack_pkt.n_extra);
                    do {
                        if (*(uint16_t*)(glossy_payload.raw_data + (i * 4)) == node_id) {       // potential cause of problems (BUT: structure glossy_payload should be aligned to an even address!)
                            stats.t_slot_last = schedule.time;
                            rounds_to_wait = 0;
                            msg.header.recipient = node_id;
                            msg.header.stream_id = *(uint8_t*)(glossy_payload.raw_data + (i * 4 + 2));
                            msg.header.len = 1;
                            if (stream_update(msg.header.stream_id)) {
                                DEBUG_PRINT_INFO("S-ACK received for stream %u (joined)", msg.header.stream_id);
                                msg.payload[0] = CMD_CODE_SACK;
                            } else {
                                DEBUG_PRINT_INFO("S-ACK received for stream %u (removed)", msg.header.stream_id);
                                msg.payload[0] = CMD_CODE_SDEL;
                            } 
                            SET_CTRL_MSG(&msg);
#ifndef FLOCKLAB
                            mm_put(&msg);  
#endif // FLOCKLAB
                        } 
                        i++;
                    } while (i <= glossy_payload.sack_pkt.n_extra);
                } else {
                    DEBUG_PRINT_VERBOSE("no data received in SACK SLOT");
                }
                slot_idx++;     // increment the packet counter
            }
            
            // --- DATA SLOTS ---
            
            if (SCHED_HAS_DATA_SLOT(&schedule)) {
                static uint8_t i;   // must be static because the for-loop might be interrupted and state of i would be lost
                for (i = 0; i < SCHED_N_SLOTS(&schedule); i++, slot_idx++) {
                    if (schedule.slot[i] == node_id) {
                        // this is our data slot, send a data packet
                        RTIMER_CAPTURE;
#ifdef FLOCKLAB
                        // generate some dummy data
                        glossy_payload.data_pkt.recipient = 0;
                        glossy_payload.data_pkt.stream_id = 0;
                        payload_len = adc_get_data(glossy_payload.data_pkt.payload) + MESSAGE_HEADER_SIZE;
#else
                        payload_len = mm_get((message_t*)glossy_payload.raw_data);    // fetch the next 'ready-to-send' packet
#endif // FLOCKLAB
                        if (payload_len) {
                            stats.t_prep_max = MAX(RTIMER_ELAPSED, stats.t_prep_max);
                            stats.t_slot_last = schedule.time;
                            WAIT_UNTIL(t_start + T_SLOT_START(slot_idx), slwb_thread_source);
                            SEND_DATA_PACKET(node_id, &glossy_payload, payload_len, 0, slwb_thread_source);
                            DEBUG_PRINT_INFO("message sent (l=%u)", payload_len);
                        }
                    } else {
                        // receive a data packet
                        WAIT_UNTIL(t_start + T_SLOT_START(slot_idx) - t_guard, slwb_thread_source);
                        SEND_DATA_PACKET(schedule.slot[i], &glossy_payload, 0, t_guard, slwb_thread_source);
                        
                        // process the received data
                        RTIMER_CAPTURE;
                        if (DATA_RECEIVED && glossy_get_payload_len()) {
                            if (glossy_payload.data_pkt.recipient == node_id || 
                                glossy_payload.data_pkt.recipient == RECIPIENT_BROADCAST) {
                                DEBUG_PRINT_INFO("data received");
#ifndef FLOCKLAB
                                mm_put((message_t*)glossy_payload.raw_data);
#endif // FLOCKLAB
                            } else {
                                DEBUG_PRINT_VERBOSE("received packet dropped");            
                            }
                        } // else: no data received                        
                        stats.t_proc_max = MAX((uint16_t)RTIMER_ELAPSED, stats.t_proc_max); // must always be smaller than T_GAP
                    }
                    stats.data_tot += glossy_get_payload_len(); 
                    stats.pck_cnt++;
                }
            }            
            
            // --- CONTENTION SLOT ---
            
            if (SCHED_HAS_CONT_SLOT(&schedule)) {   // is there a contention slot in this round?                
                // does this node have pending stream requests?
                if (STREAM_REQ_PENDING) {
                    if (!rounds_to_wait) {      // allowed to send the request?
                        // try to join, send the stream request
                        stream_prep_req((stream_request_t*)glossy_payload.raw_data);
                        rounds_to_wait = (random_rand() >> 1) % 8 + 1;  // wait between 1 and 8 rounds
                        WAIT_UNTIL(t_start + T_SLOT_START(slot_idx), slwb_thread_source);    // wait until the contention slot starts
                        SEND_DATA_PACKET(node_id, &glossy_payload, sizeof(stream_request_t), 0, slwb_thread_source);  
                        DEBUG_PRINT_INFO("stream request sent (id=%u ipi=%u ofs=%d)", glossy_payload.srq_pkt.stream_id, glossy_payload.srq_pkt.ipi, glossy_payload.srq_pkt.t_offset);
                    } else {
                        DEBUG_PRINT_VERBOSE("must wait %u rounds", rounds_to_wait);
                        // keep waiting and just relay incoming packets
                        rounds_to_wait--;              // decrease the number of rounds to wait
                        WAIT_UNTIL(t_start + T_SLOT_START(slot_idx) - t_guard, slwb_thread_source);    // wait until the contention slot starts
                        SEND_DATA_PACKET(GLOSSY_UNKNOWN_INITIATOR, &glossy_payload, 0, t_guard, slwb_thread_source);
                    }
                } else {
                    WAIT_UNTIL(t_start + T_SLOT_START(slot_idx) - t_guard, slwb_thread_source);    // wait until the contention slot starts
                    SEND_DATA_PACKET(GLOSSY_UNKNOWN_INITIATOR, &glossy_payload, 0, t_guard, slwb_thread_source);
                }
            }
        }
        
        // wait for the schedule at the end of the round
        WAIT_UNTIL(t_start + T_SCHED2_START - t_guard, slwb_thread_source);
        RCV_SCHEDULE(slwb_thread_source);
        
        // --- COMMUNICATION ROUND ENDS ---
        
        // time for other computations
                
        // update the state machine and the guard time
        COMPUTE_SYNC_STATE;
        if (BOOTSTRAP == sync_state) {
            // something went wrong
            if (SCHED_IS_1ST(&schedule)) {
                DEBUG_PRINT_ERROR("wrong schedule received!");
            }
            // this happened probably due to a host failure
            continue;
        }
        // check joining state
        if (STREAMS_ACTIVE && (schedule.time - stats.t_slot_last) > (PERIOD_MAX << 1)) {
            DEBUG_PRINT_WARNING("no-slot timeout, requesting new stream...");
            stream_rejoin();    // re-join (all streams)
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
        DEBUG_PRINT_INFO("%s %lu T=%u n=%u s=%u td=%u tp=%u d=%lu p=%u r=%u b=%u u=%u ds=%d s=%d", sync_state_to_string[sync_state], schedule.time, schedule.period, SCHED_N_SLOTS(&schedule), STREAMS_ACTIVE, stats.t_proc_max, stats.t_prep_max, stats.data_tot, stats.pck_cnt, stats.relay_cnt, stats.bootstrap_cnt, stats.unsynced_cnt, (drift_last - (int16_t)drift), drift_last);
        if (ALREADY_SYNCED == sync_state || UNSYNCED_1 == sync_state) {
            if ((drift < MAX_CLOCK_DEV) && (drift > -MAX_CLOCK_DEV)) {
                drift_last = (int16_t)drift;
            } else if (drift_last) {    // only if not zero
                // most probably a timer update overrun (or a host failure)
                // usually, the deviation per second is not higher than 50 cycles; if only one timer update is missed in 30 seconds, the deviation per second is still more than 1k cycles and therefore detectable
                DEBUG_PRINT_WARNING("Critical timing error (timer update overrun?)");
            }
        }
        
#ifndef T_PREPROCESS        
        mm_fill(glossy_payload.raw_data);     // process messages in ADI queue (if any)
#endif // T_PREPROCESS
#ifndef FLOCKLAB
        mm_flush(glossy_payload.raw_data);    // send the buffered messages over the ADI (if enough credit avaiable)
#else
        /*if (!pending_requests && ((round_count & 7) == 0)) {        // allocate a new stream after x rounds (only if no request pending!)
            stream_add((round_count >> 3) & 127, ((random_rand() >> 1) & 127) + 10, 0);      // "randomize" IPI (btw 10 and 137), set offset to zero
        }
//         round_count++;*/
#endif // FLOCKLAB

#if defined(ASYNC_INT_TIMEREQ_POLLING) && !defined(FLOCKLAB)
        if (async_int_timereq_pending(&glossy_payload.data_pkt.payload[2])) {       // check for timestamp request      
            glossy_payload.data_pkt.payload[0] = CMD_CODE_TIMESTAMP;                // command code
            SET_CTRL_MSG(&glossy_payload.data_pkt);
            glossy_payload.data_pkt.len  = 10; 
            mm_put((message_t*)glossy_payload.raw_data);
        }
#endif // ASYNC_INT_TIMEREQ_POLLING

#ifdef STORE_STATS_XMEM
        stats_save();
#endif
        // suspend this task and let other tasks run  
        memset(&schedule.slot, 0, sizeof(schedule.slot));  // erase the schedule (slot allocations only)                        
        fram_sleep(); 
        debug_process_poll();
        
#ifdef T_PREPROCESS
        WAIT_UNTIL(t_start + schedule.period * (RTIMER_SECOND + drift_last) - t_guard - T_PREPROCESS, slwb_thread_source);
#else
        WAIT_UNTIL(t_start + schedule.period * (RTIMER_SECOND + drift_last) - t_guard, slwb_thread_source);
#endif // T_PREPROCESS
    }

    PT_END(&slwb_pt);
}


/*---------------------------------------------------------------------------*/
PROCESS(simple_lwb_process, "S-LWB (Simplified version of LWB)");
AUTOSTART_PROCESSES(&simple_lwb_process);
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(simple_lwb_process, ev, data) {

    PROCESS_BEGIN();
        
    PT_INIT(&slwb_pt);
                
    // ----- INIT -----
   
    debug_print_init();     // start the debug process (NOTE: no debug prints are accepted before this line of code, use DEBUG_PRINT_NOW() instead)
#ifdef STORE_STATS_XMEM
    stats_load();           // load the stats from the external memory   
#endif
#ifndef FLOCKLAB
    mm_init();              // message manager (only use it if not running on Flocklab)
#else
    adc_init();
#endif // FLOCKLAB
    
    //PIN_SET_AS_OUTPUT(DEBUG_PIN);
    //PIN_CLEAR(DEBUG_PIN);
    
    // set glossy parameters (antenna gain, packet length and wireless channel)
    rf1a_set_tx_power(TX_POWER);
    rf1a_set_maximum_packet_length(PACKET_LEN_MAX);
    rf1a_set_channel(TX_CHANNEL);
    
    DEBUG_PRINT_INFO("T_HOP: %ums  T_SLOT_MIN: %ums", (uint16_t)(T_HOP / 3250), (uint16_t)(T_SLOT_MIN / 3250));
        
    // ----- start the S-LWB thread in 1s -----
    START_LWB(1);

    PROCESS_END();

}
 

#ifdef PUSH_BUTTON
static volatile uint8_t push_count = 0;
ISR(PORT1, port1_interrupt) {

    if (PIN_IFG(PUSH_BUTTON)) {
        if (push_count > 4) {
            WDTCTL &= ~WDTHOLD; // trigger a watchdog password violation error and therefore a reset
        } else if (push_count > 3) {
            DEBUG_PRINT_NOW("I am warning you, one more press and I crash!\r\n");
        } else if (push_count > 2) {
            DEBUG_PRINT_NOW("You are annoying, go away!\r\n");
        } else if (push_count > 1) {
            DEBUG_PRINT_NOW("Pressing that button is fun, isn't it?\r\n");
        } else if (push_count > 0) {
            DEBUG_PRINT_NOW("Great, you did it again!\r\n");
        } else {
            DEBUG_PRINT_NOW("You just interrupted my sleep! Please don't do that again\r\n");
        }
        push_count++;
        PIN_CLEAR_IFG(PUSH_BUTTON);
    } 
}
#endif // PUSH_BUTTON
