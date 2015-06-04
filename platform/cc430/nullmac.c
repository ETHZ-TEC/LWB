#include "contiki.h"

#define DEBUG_NULLMAC 1

#if DEBUG_NULLMAC
#define DEBUG(t, ...)    DEBUG_PRINT_MSG(t, "NullMAC", __VA_ARGS__)
#define LEDS_ON(...)     leds_on(__VA_ARGS__)
#define LEDS_OFF(...)    leds_off(__VA_ARGS__)
#define LEDS_TOGGLE(...) leds_toggle(__VA_ARGS__)
#else
#define DEBUG(t, ...)
#define LEDS_ON(...)
#define LEDS_OFF(...)
#define LEDS_TOGGLE(...)
#endif /* DEBUG_NULLMAC */

#define NULLMAC_BROADCAST_ADDR 0xffff
#define NULLMAC_HEADER_LEN     (sizeof(nullmac_header_t))

typedef struct {
	addr_t source;
	addr_t destination;
} nullmac_header_t;

static inline nullmac_header_t prepare_header(addr_t destination) {
	nullmac_header_t header;
	header.source = node_id;
	header.destination = destination;
	return header;
}

static rtimer_clock_t now;

/***************************** NullMAC interface *****************************/

void nullmac_init(void) {
	// do not notify about a header reception
	rf1a_set_header_len_rx(0);
	// calibrate automatically when going from IDLE to either RX or TX
	rf1a_set_calibration_mode(RF1A_CALIBRATION_MODE_AUTOMATIC_FROM_IDLE);
	// turn on the radio and start a reception attempt
	rf1a_go_to_idle();
	rf1a_start_rx();
}

void unicast_send(void *payload, uint8_t payload_len, addr_t destination) {
	now = rtimer_now();
	if (rf1a_is_busy()) {
		// the radio is busy: do not proceed with the transmission
		DEBUG(&now, "Unicast not transmitted due to ongoing radio operation");
	} else {
		// the radio is not busy: proceed with the transmission
		nullmac_header_t header = prepare_header(destination);
		DEBUG(&now, "Transmitting a %u-byte unicast to %u",
				NULLMAC_HEADER_LEN + payload_len, header.destination);
		rf1a_tx_packet((uint8_t *)&header, NULLMAC_HEADER_LEN, (uint8_t *)payload, payload_len);
	}
}

void broadcast_send(void *payload, uint8_t payload_len) {
	now = rtimer_now();
	if (rf1a_is_busy()) {
		// the radio is busy: do not proceed with the transmission
		DEBUG(&now, "Broadcast not transmitted due to ongoing radio operation");
	} else {
		// the radio is not busy: proceed with the transmission
		nullmac_header_t header = prepare_header(NULLMAC_BROADCAST_ADDR);
		DEBUG(&now, "Transmitting a %u-byte broadcast", NULLMAC_HEADER_LEN + payload_len);
		rf1a_tx_packet((uint8_t *)&header, NULLMAC_HEADER_LEN, (uint8_t *)payload, payload_len);
	}
}

/*********************** RF1A callback implementation ***********************/

void rf1a_cb_rx_started(rtimer_clock_t *timestamp) {
	// notify about the beginning of the reception
	LEDS_ON(LEDS_GREEN);
}

void rf1a_cb_tx_started(rtimer_clock_t *timestamp) {
	// notify about the beginning of the transmission
	LEDS_ON(LEDS_RED);
}

void rf1a_cb_header_received(rtimer_clock_t *timestamp, uint8_t *header, uint8_t packet_len) {
	// as we set header_len to 0, this callback function will never be executed!
}

void rf1a_cb_rx_ended(rtimer_clock_t *timestamp, uint8_t *pkt, uint8_t pkt_len) {
	// notify about the successful reception
	LEDS_OFF(LEDS_GREEN);
	nullmac_header_t header = *(nullmac_header_t *)pkt;
	uint8_t *payload = pkt + NULLMAC_HEADER_LEN;

	if (header.destination == NULLMAC_BROADCAST_ADDR) {
		// broadcast packet: execute the callback function
		DEBUG(timestamp, "RX completed. Received a %u-byte broadcast from %u. RSSI: %d dBm, LQI: %u",
				pkt_len, header.source, rf1a_get_last_packet_rssi(), rf1a_get_last_packet_lqi());
		broadcast_received(timestamp, payload, pkt_len - NULLMAC_HEADER_LEN, header.source);
	} else {
		// unicast packet
		DEBUG(timestamp, "RX completed. Received a %u-byte unicast from %u to %u. RSSI: %d dBm, LQI: %u",
				pkt_len, header.source, header.destination, rf1a_get_last_packet_rssi(), rf1a_get_last_packet_lqi());
		if (header.destination == node_id) {
			// we are the intended destination of the packet: execute the callback function
			unicast_received(timestamp, payload, pkt_len - NULLMAC_HEADER_LEN, header.source);
		}
	}

	// start a new reception attempt
	rf1a_start_rx();
}

void rf1a_cb_tx_ended(rtimer_clock_t *timestamp) {
	// notify about the successful transmission and start a new reception attempt
	LEDS_OFF(LEDS_RED);
	DEBUG(timestamp, "TX completed");
	rf1a_start_rx();
}

void rf1a_cb_rx_failed(rtimer_clock_t *timestamp) {
	// notify about the failure, flush the RX FIFO and start a new reception attempt
	LEDS_OFF(LEDS_GREEN);
	DEBUG(timestamp, "RX completed. Received a corrupted packet");
	rf1a_flush_rx_fifo();
	rf1a_start_rx();
}

void rf1a_cb_rx_tx_error(rtimer_clock_t *timestamp) {
	// notify about the error, flush both RX FIFO and TX FIFO and start a new reception attempt
	LEDS_OFF(LEDS_ALL);
	DEBUG(timestamp, "RX/TX error (interference?)");
	rf1a_flush_rx_fifo();
	rf1a_flush_tx_fifo();
	rf1a_start_rx();
}

