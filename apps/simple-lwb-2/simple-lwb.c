/*
 * 
 * Implementation steps:
 * - simple LWB runs (empty schedule, fixed period, 1 source and 1 host)
 * - 2nd schedule at the end of a round added
 * - 'random' period length added
 * - TX power reduced to -12 dBm, channel switched to ...
 * - clock drift compensation added
 * - sync state machine added (NOT yet verified)
 * - data slots added (test data sent and received successfully)
 *
 *
 * To-do:
 * - add the scheduler and the compress/decompress functionality
 * - add stream requests / s-acks
 */
 

#include "simple-lwb.h"

static statistics_t stats;
static struct pt simple_lwb_pt;
static sync_state_t sync_state;
static joining_state_t joining_state;
static rtimer_clock_t T_guard;


// state machine for synchronization (note: undefined transitions do not change the state)
static const sync_state_t next_state[NUM_OF_SYNC_EVENTS][NUM_OF_SYNC_STATES] = 
{   // STATES:                                                                                                                                               // EVENTS:
    // STATE_BOOTSTRAP,   STATE_QUASI_SYNCED, STATE_SYNCED,     STATE_ALREADY_SYNCED, STATE_UNSYNCED_1,     STATE_UNSYNCED_2,     STATE_UNSYNCED_3
    { STATE_QUASI_SYNCED, STATE_SYNCED,       STATE_SYNCED,     STATE_SYNCED,         STATE_SYNCED,         STATE_SYNCED,         STATE_SYNCED         },   // 1st schedule received
    { STATE_BOOTSTRAP,    STATE_QUASI_SYNCED, STATE_SYNCED,     STATE_ALREADY_SYNCED, STATE_ALREADY_SYNCED, STATE_ALREADY_SYNCED, STATE_ALREADY_SYNCED },   // 2nd schedule received
    { STATE_BOOTSTRAP,    STATE_BOOTSTRAP,    STATE_UNSYNCED_1, STATE_UNSYNCED_1,     STATE_UNSYNCED_2,     STATE_UNSYNCED_3,     STATE_BOOTSTRAP      }    // schedule missed
};


#define GET_EVENT       ( glossy_is_t_ref_updated() ? (IS_LAST_BIT_SET(schedule.period) ? EVT_1ST_SCHED_RCVD : EVT_2ND_SCHED_RCVD) : EVT_SCHED_MISSED )

#define SEND_SCHEDULE   {\
                            LEDS_ON;\
                            glossy_start(HOST_ID, (uint8_t *)&schedule, schedule_len, N_TX_MAX_SCHEDULE, GLOSSY_WITH_SYNC, GLOSSY_WITH_RF_CAL);\
                            rtimer_schedule(0, rt->time + T_SCHED, 0, simple_lwb_thread);\
                            PT_YIELD(&simple_lwb_pt);\
                            glossy_stop();\
                            LEDS_OFF;\
                        }
            
#define RCV_SCHEDULE    {\
                            LEDS_ON;\
                            glossy_start(HOST_ID, (uint8_t *)&schedule, 0, N_TX_MAX_SCHEDULE, GLOSSY_WITH_SYNC, GLOSSY_WITH_RF_CAL);\
                            rtimer_schedule(0, rt->time + T_SCHED + T_guard, 0, simple_lwb_thread);\
                            PT_YIELD(&simple_lwb_pt);\
                            glossy_stop();\
                            LEDS_OFF;\
                        }

                        
static inline void compute_sync_state(schedule_t schedule) {
    // get the new state based on the event
    sync_state = next_state[GET_EVENT][sync_state];
    
    // adjust the guard time
    if (STATE_UNSYNCED_1 == sync_state) {
        T_guard = T_GUARD_1;
    } else if (STATE_UNSYNCED_2 == sync_state) {
        T_guard = T_GUARD_2;
    } else if (STATE_UNSYNCED_3 == sync_state) {
        T_guard = T_GUARD_3;
    } else {
        T_guard = T_GUARD;
    }
}                        

static inline uint8_t initialize_schedule(schedule_t *sched) {
	sched->time = 0;
	sched->period = T_INIT;
	sched->n_slots = 0;
	uint8_t schedule_len = SCHED_HEADER_LENGTH;
	return schedule_len;
}

static inline uint8_t compute_schedule(schedule_t *sched) {
	sched->time += T_INIT;
	sched->period = T_INIT;
    sched->seq_no++;
    sched->n_slots = sizeof(nodes);     // 1 byte per index
	uint8_t schedule_len = SCHED_HEADER_LENGTH + sizeof(nodes);
	uint8_t i;
	for (i = 0; i < sched->n_slots; i++) {
		sched->slot[i] = nodes[i];
	}
	stats.data_tot += sched->n_slots;
	return schedule_len;
}

// TODO: collect some real data
uint8_t prepare_payload(uint8_t *payload) {
	uint8_t payload_len = PACKET_LEN_MAX - SCHED_HEADER_LENGTH; // max. payload length
	uint8_t i;
	for (i = 0; i < payload_len; i++) {
		payload[i] = (uint8_t)(node_id + i);
	}
	return payload_len;
}


PT_THREAD(simple_lwb_thread(rtimer_t *rt)) {
	PT_BEGIN(&simple_lwb_pt);

	static uint8_t slot_idx;
	static uint8_t payload[PACKET_LEN_MAX];
	static schedule_t schedule;
	static uint8_t schedule_len, payload_len;
	static rtimer_clock_t t_start, t_ref;

	schedule_len = initialize_schedule(&schedule);
	memset(&stats, 0, sizeof(stats));

   if (HOST_ID == node_id) {    // is this a host or a source node?
    
        // HOST NODE
            
        while (1) {
            t_start = rt->time;
            SET_LAST_BIT(schedule.period);
            
            // SEND the previously computed schedule 
            SEND_SCHEDULE;

            stats.relay_cnt = glossy_get_relay_cnt_first_rx();
                    
            // start communication round (data slots)  -> look at g_rr_host()
            
            // TODO: if sink, deliver data packets
            
            // TODO: transmit possible S-ACKs (stream acks)
            /*if (GET_N_SACK(N_SLOTS)) {
                leds_off(LEDS_ALL);
                SCHEDULE(T_REF, T_SACK_START, g_rr_host);
                PT_YIELD(&pt_rr);
                // transmit s-ack
                prepare_stream_ack();
                save_energest_values();
                glossy_start((uint8_t *)pkt, pkt_len, GLOSSY_INITIATOR, GLOSSY_NO_SYNC, N_RR, pkt_type,
                        T_REF + T_SACK_STOP, (rtimer_callback_t)g_rr_host, &rt, NULL);
                PT_YIELD(&pt_rr);

                glossy_stop();
                update_control_dc();
                memset(stream_ack, 0, STREAM_ACK_LENGTH);
                stream_ack_idx = 0;
            }*/
            
            // data slots (receive / transmit data packets)
            DEBUG_PRINT_INFO("schedule has %u slots (%d, %d, %d, %d, ...)", schedule.n_slots, schedule.slot[0], schedule.slot[1], schedule.slot[2], schedule.slot[3]);
            for (slot_idx = 0; slot_idx < schedule.n_slots; slot_idx++) {

                // wait until the data slot starts
                rtimer_schedule(0, t_start + (T_SCHED + T_GAP) + (T_DATA + T_GAP) * slot_idx, 0, simple_lwb_thread);  // T_SLOT_START(slot_idx)
                PT_YIELD(&simple_lwb_pt);
                // TODO: wouldn't it be easier to use rt->time + T_GAP here, too?

                LEDS_ON;
                glossy_start(schedule.slot[slot_idx], payload, 0, N_TX_MAX_DATA, GLOSSY_WITHOUT_SYNC, GLOSSY_WITHOUT_RF_CAL);
                rtimer_schedule(0, rt->time + T_DATA, 0, simple_lwb_thread);    // FIXME: T_REF + T_SLOT_STOP(slot_idx)
                PT_YIELD(&simple_lwb_pt);
                glossy_stop();
                LEDS_OFF;
                
                if (glossy_get_n_rx() > 0) {
                    DEBUG_PRINT_INFO("data received!");
                    // TODO: store the received data packet in a list and/or deliver it to the application
                    // NOTE: this function mustn't take too long to complete! otherwise T_GAP is too short!
                }
            }
            
            // receive D-ACKs (data acks)
            // TODO
            
            // contention slot (receive stream requests)
            /*for (; slot_idx < GET_N_SLOTS(N_SLOTS) + GET_N_FREE(N_SLOTS); slot_idx++) {
                leds_off(LEDS_ALL);
                SCHEDULE(T_REF, T_SLOT_START(slot_idx), g_rr_host);     // wait until the data slot starts
                PT_YIELD(&pt_rr);

                leds_on(slot_idx + 1);
                save_energest_values();
                glossy_start((uint8_t *)pkt, 0, GLOSSY_RECEIVER, GLOSSY_NO_SYNC, N_RR, 0,
                        T_REF + T_SLOT_STOP(slot_idx), (rtimer_callback_t)g_rr_host, &rt, NULL);
                PT_YIELD(&pt_rr);

                if (glossy_stop() && ((get_data_len() - DATA_HEADER_LENGTH) % STREAM_REQ_LENGTH == 0)) {
                    // get the data header
                    memcpy(&data_header, pkt, DATA_HEADER_LENGTH);
                    pkt_type = get_header();
                    memcpy(stream_reqs, pkt + DATA_HEADER_LENGTH, GET_N_STREAM_REQS(pkt_type) * STREAM_REQ_LENGTH);
                    check_for_stream_requests();
                }
                update_control_dc();
            }*/
            
            // compute the new schedule
            schedule_len = compute_schedule(&schedule);     // TODO: compress the schedule
            schedule.period = (0x0f & (uint8_t)(rt->time >> 16)) + 1;      // 'random' period between 1 and 16
            //CLEAR_LAST_BIT(schedule.period);              // -> not necessary
            rtimer_schedule(0, t_start + RTIMER_SECOND / 2, 0, simple_lwb_thread);
            PT_YIELD(&simple_lwb_pt);
            SEND_SCHEDULE;      // send the schedule for the next round
            
            DEBUG_PRINT_INFO("round finished, schedule updated (T = %u)", schedule.period);

            rtimer_schedule(0, t_start + schedule.period * RTIMER_SECOND, 0, simple_lwb_thread);
            debug_process_poll();
            PT_YIELD(&simple_lwb_pt);
        }
     
    } else {
    
        // SOURCE NODE
        
        sync_state = STATE_BOOTSTRAP;
        joining_state = NOT_JOINED;
        stats.period_last = T_INIT;
        stats.skew_last   = 0;
        
        while (1) {
            
            if (sync_state == STATE_BOOTSTRAP) {
                printf("BOOTSTRAP ");
                stats.skew_last = 0;
                // synchronize first! wait for the first schedule...
                do {
                    printf(".");
                    RCV_SCHEDULE; 
                } while (!glossy_is_t_ref_updated() || !IS_LAST_BIT_SET(schedule.period));
                // schedule received!
                printf("\n");
            } else {
                RCV_SCHEDULE;            
            }
                       
            // update the sync state machine (compute new sync state)
            compute_sync_state(schedule);
            // TODO compute new joining state
                 
            CLEAR_LAST_BIT(schedule.period); 
            stats.last_t_ref = t_ref;
            t_ref = glossy_is_t_ref_updated() ? glossy_get_t_ref() : (t_ref + schedule.period);

            // do we have permission to participate in this round?
            if (sync_state == STATE_SYNCED || sync_state == STATE_UNSYNCED_1) {    // schedule received
                
                stats.relay_cnt = glossy_get_relay_cnt_first_rx();
                
                // TODO uncompress the schedule
                
                // TODO: if sink, deliver data packets
                
                // start communication round
                
                // receive S-ACKs
                
                /*if (GET_N_SACK(N_SLOTS)) {
                    leds_off(LEDS_ALL);
                    SCHEDULE(T_REF, T_SACK_START, g_rr_source); // wait for the time slot to arrive
                    PT_YIELD(&pt_rr);
                    // receive s-ack
                    save_energest_values();
                    glossy_start((uint8_t *)pkt, 0, GLOSSY_RECEIVER, GLOSSY_NO_SYNC, N_RR, pkt_type,
                            T_REF + T_SACK_STOP, (rtimer_callback_t)g_rr_source, &rt, NULL);
                    PT_YIELD(&pt_rr);

                    glossy_stop();
                    update_control_dc();
                    if (get_rx_cnt()) {     // data received?
                        // get the data header
                        memcpy(&data_header, pkt, DATA_HEADER_LENGTH);
                        pkt_type = get_header();
                        if (IS_STREAM_ACK(pkt_type)) {
                            update_control_dc();
                            memcpy(&stream_ack_idx, pkt + DATA_HEADER_LENGTH, STREAM_ACK_IDX_LENGTH);
                            memcpy(stream_ack, pkt + DATA_HEADER_LENGTH + STREAM_ACK_IDX_LENGTH, stream_ack_idx * STREAM_ACK_LENGTH);
                            for (idx = 0; idx < stream_ack_idx; idx++) {
                                if (stream_ack[idx] == node_id) {
                                    n_stream_reqs--;
                                    if (n_stream_reqs == 0) {
                                        joining_state = JOINED;
                                    }
                                    curr_ipi = stream_reqs[0].ipi;
                                }
                            }
                        }
                    }
                }*/
                
                // data slots
                if (schedule.n_slots == 0) DEBUG_PRINT_INFO("no data slots...");
                for (slot_idx = 0; slot_idx < schedule.n_slots; slot_idx++) {
                    if (schedule.slot[slot_idx] == (uint8_t)node_id) {
                        payload_len = prepare_payload(payload);
                    } else {
                        payload_len = 0;
                    }
                    rtimer_schedule(0, t_ref + (T_SCHED + T_GAP) + (T_DATA + T_GAP) * slot_idx, 0, simple_lwb_thread);  // maybe use (rt->time + T_GAP) instead
                    PT_YIELD(&simple_lwb_pt);

                    LEDS_ON;
                    glossy_start(schedule.slot[slot_idx], payload, payload_len, N_TX_MAX_DATA, GLOSSY_WITHOUT_SYNC, GLOSSY_WITHOUT_RF_CAL);
                    rtimer_schedule(0, rt->time + T_DATA, 0, simple_lwb_thread);
                    PT_YIELD(&simple_lwb_pt);
                    glossy_stop();
                    LEDS_OFF;
                    
                    //if (glossy_get_n_rx() > 0 && schedule.slot[slot_idx] != node_id) {
                        // packet received -> but nothing needs to be done with this packet
                    //}
                }
                
                // TODO: if sink, transmit data ACKs
                
                // contention slot
                /*for (; slot_idx < GET_N_SLOTS(N_SLOTS) + n_sinks + GET_N_FREE(N_SLOTS); slot_idx++) {
                    leds_off(LEDS_ALL);
                    SCHEDULE(T_REF, T_SLOT_START(slot_idx), g_rr_source); // wait until the data slot starts
                    PT_YIELD(&pt_rr);

                    leds_on(slot_idx + 1);
                    // wait until we are not a member of the view sent by the host before sending a request
                    // FIXME: works if a node has only one request to transmit (e.g, either sender with one stream or receiver)
                    if (joining_state == JOINING && (sync_state == SYNCED || sync_state == QUASI_SYNCED) && (is_source || is_sink)) {
                        n_stream_reqs = 1;
                        if (rounds_to_wait == 0) {
                            // try to join
                            joining_state = JUST_TRIED;
                            prepare_data_packet(1);
                            save_energest_values();
                            glossy_start((uint8_t *)pkt, pkt_len, GLOSSY_INITIATOR, GLOSSY_NO_SYNC, N_RR, pkt_type,
                                    T_REF + T_SLOT_STOP(slot_idx), (rtimer_callback_t)g_rr_source, &rt, NULL);
                        } else {
                            // keep waiting, decrease the number of rounds to wait
                            rounds_to_wait--;
                            save_energest_values();
                            glossy_start((uint8_t *)pkt, 0, GLOSSY_RECEIVER, GLOSSY_NO_SYNC, N_RR, 0,
                                    T_REF + T_SLOT_STOP(slot_idx), (rtimer_callback_t)g_rr_source, &rt, NULL);
                        }
                    } else {
                        // already joined or not SYNCED
                        save_energest_values();
                        glossy_start((uint8_t *)pkt, 0, GLOSSY_RECEIVER, GLOSSY_NO_SYNC, N_RR, 0,
                                T_REF + T_SLOT_STOP(slot_idx), (rtimer_callback_t)g_rr_source, &rt, NULL);
                    }
                    PT_YIELD(&pt_rr);

                    glossy_stop();
                    update_control_dc();
                }*/
            }
            
            // wait for the schedule at the end of the round
            rtimer_schedule(0, t_ref + RTIMER_SECOND / 2 - T_guard, 0, simple_lwb_thread);
            PT_YIELD(&simple_lwb_pt);
            RCV_SCHEDULE;
                                         
            // update the state machine
            sync_state = next_state[GET_EVENT][sync_state];
            
            // estimate and update the skew (clock cycles per second!)
            int32_t skew = ((t_ref - stats.last_t_ref) - ((int32_t)stats.period_last * RTIMER_SECOND));
            skew /= (int32_t)stats.period_last;
            DEBUG_PRINT_INFO("%lu %u %lu %lu %u %ld %s", schedule.time, schedule.period, stats.data_tot, stats.data_rcvd, stats.relay_cnt, (stats.skew_last - skew), sync_state_to_string[sync_state]);
            if (skew < 500 && skew > -500) {  // FIXME: this is a nasty hack
                stats.skew_last = skew;
            }
            stats.period_last = schedule.period;
            
            if (STATE_BOOTSTRAP != sync_state) {                
                memset((uint8_t *)&schedule.slot, 0, sizeof(schedule.slot));  // erase the schedule (slot allocations only)
                rtimer_schedule(0, t_ref + schedule.period * (RTIMER_SECOND + stats.skew_last) - T_guard, 0, simple_lwb_thread);
                debug_process_poll();
                PT_YIELD(&simple_lwb_pt);
            }
        }
    }

	PT_END(&simple_lwb_pt);
}

/*---------------------------------------------------------------------------*/
PROCESS(simple_lwb_process, "S-LWB (Simplified version of LWB)");
AUTOSTART_PROCESSES(&simple_lwb_process);
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(simple_lwb_process, ev, data) {

	PROCESS_BEGIN();

	PT_INIT(&simple_lwb_pt);
    
    node_id = 2; //HOST_ID;

	rf1a_set_tx_power(TX_POWER);
	rf1a_set_maximum_packet_length(PACKET_LEN_MAX);
	rf1a_set_channel(TX_CHANNEL);

	PIN_SET_AS_OUTPUT_DIRECT(1, 0); // LED1
	PIN_SET_AS_OUTPUT_DIRECT(1, 1); // LED2
	PIN_SET_AS_OUTPUT_DIRECT(1, 2); // LED3
	PIN_SET_AS_OUTPUT_DIRECT(3, 6); // INT1
	PIN_SET_AS_OUTPUT_DIRECT(3, 7); // INT2

	PIN_MAP_AS_OUTPUT_DIRECT(1, 1, PM_RFGDO2);

	// execute simple_lwb_thread() after T_SCHED, using rtimer 0
	rtimer_schedule(0, rtimer_now() + T_SCHED, 0, simple_lwb_thread);

	PROCESS_END();

}


