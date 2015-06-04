
/**
 * 
 * Changes by rdaforno:
 * - enabled_interrupts variable added, disable undesired interrupts during a glossy flood
 */

#include "glossy.h"
#include "math.h"

#define DEBUG_GLOSSY 0

// minimum and maximum number of slots after which the timeout expires, since the last transmission
// NOTE: values below 2 do not make sense, as there would be no chance to receive a packet in between
#define SLOT_TIMEOUT_MIN 2
#define SLOT_TIMEOUT_MAX 2
// number of extra ticks required by the timeout callback before starting the transmission
// (to keep synchronous transmissions and time synchronization as much accurate as possible)
#define TIMEOUT_EXTRA_TICKS 70

// maximum tolerance to accept measurements of T_slot when comparing to the theoretical value
// (value in clock ticks)
#define T_SLOT_TOLERANCE 10


typedef struct {
    rtimer_clock_t t_ref, t_tx_stop, t_rx_start, t_rx_stop, t_tx_start;
    rtimer_clock_t T_slot_sum;
    rtimer_clock_t T_slot_estimated;
    rtimer_clock_t t_timeout;
    uint8_t n_T_slot;
    glossy_header_t header;
    uint8_t *payload;
    uint8_t payload_len;
    uint8_t active;
    uint8_t n_rx;
    uint8_t n_tx;
    uint8_t relay_cnt_last_rx, relay_cnt_last_tx, relay_cnt_first_rx, relay_cnt_t_ref;
    uint8_t relay_cnt_timeout;
    uint8_t t_ref_updated;
    uint8_t header_ok;
    uint8_t enabled_interrupts;
} glossy_state_t;
static glossy_state_t g;

/************************* Glossy helper functions **************************/

static inline uint8_t process_glossy_header(uint8_t *pkt, uint8_t pkt_len, uint8_t crc_ok) {
    // extract the Glossy header from the packet
    glossy_header_t *rcvd_header = (glossy_header_t *)pkt;

    if (!g.header_ok) {
        // we have not checked the header yet, so check it now

        if (GET_COMMON_HEADER(rcvd_header->pkt_type) != GLOSSY_COMMON_HEADER) {
            // keep processing only if the common header is correct
            return FAIL;
        }

        if ((GET_SYNC(g.header.pkt_type) != GLOSSY_UNKNOWN_SYNC) &&
                (GET_SYNC(g.header.pkt_type) != GET_SYNC(rcvd_header->pkt_type))) {
            // keep processing only if the local sync value is either unknown or it matches the received one
            return FAIL;
        }

        if ((GET_N_TX_MAX(g.header.pkt_type) != GLOSSY_UNKNOWN_N_TX_MAX) &&
                GET_SYNC(g.header.pkt_type) != GET_SYNC(rcvd_header->pkt_type)) {
            // keep processing only if the local n_tx_max value is either unknown or it matches the received one
            return FAIL;
        }

        if ((g.header.initiator_id != GLOSSY_UNKNOWN_INITIATOR) &&
                (g.header.initiator_id != rcvd_header->initiator_id)) {
            // keep processing only if the local initiator_id value is either unknown or it matches the received one
            return FAIL;
        }

        if ((g.payload_len != GLOSSY_UNKNOWN_PAYLOAD_LEN) &&
                (g.payload_len != pkt_len - GLOSSY_HEADER_LEN(g.header.pkt_type))) {
            // keep processing only if the local payload_len value is either unknown or it matches the received one
            return FAIL;
        }

        // the header is ok
        g.header_ok = 1;
    }

    if (crc_ok) {
        // we have received the entire packet (and the CRC was ok)

        // store the received header (all the unknown values are also learned)
        g.header = *rcvd_header;
        // store the payload_len
        g.payload_len = pkt_len - GLOSSY_HEADER_LEN(g.header.pkt_type);
        // store the header_len
        rf1a_set_header_len_rx(GLOSSY_HEADER_LEN(g.header.pkt_type));
    }

    return SUCCESS;
}

static inline rtimer_clock_t estimate_T_slot(uint8_t pkt_len) {
    rtimer_clock_t T_tx_estim = T_TX_BYTE * (pkt_len + 3) + T_TX_OFFSET;
    return NS_TO_RTIMER_TICKS(T_tx_estim + T2R - TAU1);
}

static inline char timeout_expired(rtimer_t *rt) {
    if (!rf1a_is_busy()) {
        // we are not receiving anything: retransmit the packet
        rf1a_start_tx();
        g.header.relay_cnt = g.relay_cnt_timeout;
        rf1a_write_to_tx_fifo((uint8_t *)&g.header, GLOSSY_HEADER_LEN(g.header.pkt_type),
                (uint8_t *)g.payload, g.payload_len);
        g.t_timeout = rt->time;
    } else {
        // we are receiving a packet: postpone the timeout by one slot
        g.relay_cnt_timeout++;
        rtimer_schedule(3, rt->time + g.T_slot_estimated, 0, timeout_expired);
    }
    return 0;
}

static inline void schedule_timeout(void) {
    // number of slots after which the timeout will expire:
    // random number between SLOT_TIMEOUT_MIN and SLOT_TIMEOUT_MAX
    uint8_t slot_timeout = SLOT_TIMEOUT_MIN + (random_rand() % (SLOT_TIMEOUT_MAX - SLOT_TIMEOUT_MIN + 1));
    if (WITH_RELAY_CNT()) {
        // if the relay counter is sent, increment it by the chosen number of slots
        g.relay_cnt_timeout = g.header.relay_cnt + slot_timeout;
    }
    rtimer_schedule(3, g.t_timeout + slot_timeout * g.T_slot_estimated, 0, timeout_expired);
}

static inline void update_t_ref(rtimer_clock_t t_ref, uint8_t relay_cnt) {
    g.t_ref = t_ref;
    g.t_ref_updated = 1;
    g.relay_cnt_t_ref = relay_cnt;
}

static inline void add_T_slot_measurement(rtimer_clock_t T_slot_measured) {
    if ((T_slot_measured > g.T_slot_estimated - T_SLOT_TOLERANCE) &&
            (T_slot_measured < g.T_slot_estimated + T_SLOT_TOLERANCE)) {
        g.T_slot_sum += T_slot_measured;
        g.n_T_slot++;
    }
}

/***************************** Glossy interface *****************************/

void glossy_start(uint16_t initiator_id, 
                  uint8_t *payload, 
                  uint8_t payload_len, 
                  uint8_t n_tx_max, 
                  glossy_sync_t sync, 
                  glossy_rf_cal_t rf_cal) {
    GLOSSY_STARTED;
    DEBUG_PRINT_VERBOSE("Glossy started: in=%u, pl=%u, n=%u, s=%u", initiator_id, payload_len, n_tx_max, sync);

    // disable undesired interrupts
    g.enabled_interrupts = 0;
  #ifndef ASYNC_INT_TIMEREQ_POLLING
    g.enabled_interrupts |= ASYNC_INT_IS_TIMEREQ_ENABLED;  // 1st bit
    ASYNC_INT_TIMEREQ_DISABLE;
  #endif
  
    g.active = 1;
    g.payload = payload;
    g.payload_len = payload_len;
    g.n_rx = 0;
    g.n_tx = 0;
    g.relay_cnt_last_rx = 0;
    g.relay_cnt_last_tx = 0;
    g.t_ref_updated = 0;
    g.T_slot_sum = 0;
    g.n_T_slot = 0;

    // prepare the Glossy header, with the information known so far
    g.header.initiator_id = initiator_id;
    SET_PKT_TYPE(g.header.pkt_type, sync, n_tx_max);
    g.header.relay_cnt = 0;

    // automatically switch to TX at the end of RX
    rf1a_set_rxoff_mode(RF1A_OFF_MODE_TX);
    // automatically switch to RX at the end of TX
    rf1a_set_txoff_mode(RF1A_OFF_MODE_RX);
    // do not calibrate automatically
    rf1a_set_calibration_mode(RF1A_CALIBRATION_MODE_MANUAL);

    if (rf_cal == GLOSSY_WITH_RF_CAL) {
        // if instructed so, perform a manual calibration
        rf1a_manual_calibration();
    }

    rf1a_set_header_len_rx(GLOSSY_HEADER_LEN(g.header.pkt_type));

    rf1a_go_to_idle();

    if (IS_INITIATOR()) {
        // Glossy initiator
        if (GET_SYNC(g.header.pkt_type) == GLOSSY_UNKNOWN_SYNC) {
            // the initiator must know whether there will be synchronization or not!
            glossy_stop();
        } else {
            // start the first transmission
            g.t_timeout = rtimer_now() + TIMEOUT_EXTRA_TICKS;
            rf1a_start_tx();
            rf1a_write_to_tx_fifo((uint8_t *)&g.header, GLOSSY_HEADER_LEN(g.header.pkt_type),
                    (uint8_t *)g.payload, g.payload_len);
            g.relay_cnt_timeout = 0;
        }
    } else {
        // Glossy receiver
        rf1a_start_rx();
    }
}

uint8_t glossy_stop(void) {
    if (g.active) {
        GLOSSY_STOPPED;
        // stop the timeout
        rtimer_stop(3);
        // flush both RX FIFO and TX FIFO and go to sleep
        rf1a_flush_rx_fifo();
        rf1a_flush_tx_fifo();
        rf1a_go_to_sleep();
        GLOSSY_RX_STOPPED;
        GLOSSY_TX_STOPPED;
        g.active = 0;

        if (g.t_ref_updated) {
            if (g.n_T_slot > 0) {
                g.t_ref -= (g.relay_cnt_t_ref * g.T_slot_sum) / g.n_T_slot;
            } else {
                g.t_ref -= g.relay_cnt_t_ref * g.T_slot_estimated;
            }
        }

        if (g.n_rx > 0) {
            DEBUG_PRINT_VERBOSE("Glossy stopped: in=%u, pl=%u, n=%u, s=%u, rc_rx=%u, rc_tx=%u",
                    g.header.initiator_id, g.payload_len,
                    GET_N_TX_MAX(g.header.pkt_type), GET_SYNC(g.header.pkt_type), g.relay_cnt_last_rx, g.relay_cnt_last_tx);
            DEBUG_PRINT_VERBOSE("Glossy n_Ts=%u, rc_tref=%u, Ts=%llu, tref=%llu, Ts_est=%llu",
                    g.n_T_slot, g.relay_cnt_t_ref, (g.n_T_slot > 0) ? (g.T_slot_sum / g.n_T_slot) : 0, g.t_ref, g.T_slot_estimated);
        } else {
            DEBUG_PRINT_VERBOSE("Glossy stopped");
        }

        // re-enable interrupts
      #ifndef ASYNC_INT_TIMEREQ_POLLING
        if (g.enabled_interrupts & 1) {     // 1st bit
            ASYNC_INT_TIMEREQ_ENABLE;
        }
      #endif
    }

    return g.n_rx;
}

uint8_t glossy_is_active(void) {
    return g.active;
}

uint8_t glossy_get_n_rx(void) {
    return g.n_rx;
}

uint8_t glossy_get_n_tx(void) {
    return g.n_tx;
}

uint8_t glossy_get_payload_len(void) {
    return g.payload_len;
}

uint8_t glossy_is_t_ref_updated(void) {
    return g.t_ref_updated;
}

rtimer_clock_t glossy_get_t_ref(void) {
    return g.t_ref;
}

uint8_t glossy_get_relay_cnt_first_rx(void) {
    return g.relay_cnt_first_rx;
}

/*********************** RF1A callback implementation ***********************/

void rf1a_cb_rx_started(rtimer_clock_t *timestamp) {
    GLOSSY_RX_STARTED;
    // notify about the beginning of the reception
    DEBUG_PRINT_VERBOSE("Glossy RX started");

    RTIMER_UPDATE_DISABLE;      // timer overflow / update interrupt (must do it before every RX!

    g.t_rx_start = *timestamp;
    g.header_ok = 0;

    if (IS_INITIATOR()) {
        // we are the initiator and we have started a packet reception: stop the timeout
        rtimer_stop(3);
    }
}

void rf1a_cb_tx_started(rtimer_clock_t *timestamp) {
    GLOSSY_TX_STARTED;
    // notify about the beginning of the transmission
    DEBUG_PRINT_VERBOSE("Glossy TX started");

    g.t_tx_start = *timestamp;

    if (g.n_tx == 0) {
        // first transmission: estimate the slot length based on the packet length
        g.T_slot_estimated = estimate_T_slot(GLOSSY_HEADER_LEN(g.header.pkt_type) + g.payload_len);
    }
}

void rf1a_cb_header_received(rtimer_clock_t *timestamp, uint8_t *header, uint8_t packet_len) {
    if (process_glossy_header(header, packet_len, 0) != SUCCESS) {
        // the header is not ok: interrupt the reception and start a new attempt
        rf1a_cb_rx_failed(timestamp);
    }
}

void rf1a_cb_rx_ended(rtimer_clock_t *timestamp, uint8_t *pkt, uint8_t pkt_len) {
    GLOSSY_RX_STOPPED;
    RTIMER_UPDATE_ENABLE;    // timer overflow / update interrupt (must do it after every RX ended)
    g.t_rx_stop = *timestamp;

    if ((process_glossy_header(pkt, pkt_len, 1) == SUCCESS)) {
        // we received a correct packet, and the header has been stored into g.header
        uint8_t *payload = pkt + GLOSSY_HEADER_LEN(g.header.pkt_type);

        if (WITH_RELAY_CNT()) {
            // the relay counter is part of the header
            if (g.n_rx == 0) {
                // store the relay counter corresponding to the first reception
                g.relay_cnt_first_rx = g.header.relay_cnt;
            }
            // increment the relay counter
            g.header.relay_cnt++;
        }

        if ((GET_N_TX_MAX(g.header.pkt_type) == 0) || (g.n_tx < GET_N_TX_MAX(g.header.pkt_type))) {
            // if n_tx_max is either unknown or not yet reached, transmit the packet
            rf1a_write_to_tx_fifo((uint8_t *)&g.header, GLOSSY_HEADER_LEN(g.header.pkt_type),
                    payload, g.payload_len);
        } else {
            // otherwise, stop Glossy
            glossy_stop();
        }

        // increment the reception counter
        g.n_rx++;

        if ((!IS_INITIATOR()) && (g.n_rx == 1)) {
            // we are a receiver and this was our first packet reception:
            // store the payload for the application
            memcpy((uint8_t *)g.payload, payload, g.payload_len);
        }

        if (WITH_SYNC()) {
            // store the relay counter of this last reception
            g.relay_cnt_last_rx = g.header.relay_cnt - 1;

            if (g.t_ref_updated == 0) {
                // t_ref has not been updated yet: update it
                update_t_ref(g.t_rx_start - NS_TO_RTIMER_TICKS(TAU1), g.header.relay_cnt - 1);
            }

            if ((g.relay_cnt_last_rx == g.relay_cnt_last_tx + 1) && (g.n_tx > 0)) {
                // this reception immediately followed a transmission: measure T_slot
                add_T_slot_measurement(g.t_rx_start - g.t_tx_start - NS_TO_RTIMER_TICKS(TAU1));
            }
        }

        // notify about the successful reception
        DEBUG_PRINT_VERBOSE("Glossy RX completed. Received a %u-byte packet with initiator %u.",
                pkt_len, g.header.initiator_id);
    } else {
        // some fields in the header were not correct: discard it
        rf1a_cb_rx_failed(timestamp);
    }
}

void rf1a_cb_tx_ended(rtimer_clock_t *timestamp) {
    GLOSSY_TX_STOPPED;
    // notify about the successful transmission
    DEBUG_PRINT_VERBOSE("Glossy TX completed");

    g.t_tx_stop = *timestamp;

    if (WITH_SYNC()) {
        // store the relay counter of this last transmission
        g.relay_cnt_last_tx = g.header.relay_cnt;

        if (g.t_ref_updated == 0) {
            // t_ref has not been updated yet: update it
            update_t_ref(g.t_tx_start, g.header.relay_cnt);
        }

        if ((g.relay_cnt_last_tx == g.relay_cnt_last_rx + 1) && (g.n_rx > 0)) {
            // this transmission immediately followed a reception: measure T_slot
            add_T_slot_measurement(g.t_tx_start - g.t_rx_start + NS_TO_RTIMER_TICKS(TAU1));
        }
    }

    // increment the transmission counter
    g.n_tx++;

    if ((g.n_tx == GET_N_TX_MAX(g.header.pkt_type)) &&
            (GET_N_TX_MAX(g.header.pkt_type) > 1 || (!IS_INITIATOR()))) {
        // we have reached N_tx_max and either N_tx_max > 1 or we are a receiver: stop Glossy
        glossy_stop();
    } else {
        if ((IS_INITIATOR()) && (g.n_rx == 0)) {
            // we are the initiator and we still have not received any packet: schedule the timeout
            schedule_timeout();
        }
    }
}

void rf1a_cb_rx_failed(rtimer_clock_t *timestamp) {
    GLOSSY_RX_STOPPED;
    // notify about the failure, flush the RX FIFO and start a new reception attempt
    DEBUG_PRINT_WARNING("Glossy RX failed, corrupted packet received");

    RTIMER_UPDATE_ENABLE;    // timer overflow / update interrupt (must do it after every RX ended)
    rf1a_flush_rx_fifo();
    rf1a_start_rx();
    //PIN_TOGGLE_DIRECT(1, 2);
}

void rf1a_cb_rx_tx_error(rtimer_clock_t *timestamp) {
    GLOSSY_RX_STOPPED;
    GLOSSY_TX_STOPPED;
    // notify about the error
    DEBUG_PRINT_WARNING("Glossy RX/TX error (interference?)");

    RTIMER_UPDATE_ENABLE;    // timer overflow / update interrupt (must do it after every RX ended)

    if (g.active) {
        // if Glossy is still active, flush both RX FIFO and TX FIFO and start a new reception attempt
        rf1a_flush_rx_fifo();
        rf1a_flush_tx_fifo();
        rf1a_start_rx();
        //PIN_TOGGLE_DIRECT(1, 2);
    }
}
