/*
 * Copyright (c) 2015, Swiss Federal Institute of Technology (ETH Zurich).
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Author:  Federico Ferrari
 *          Marco Zimmerling
 */

#include "contiki.h"
#include "platform.h"

#if NULLMAC_CONF_ON

#define DEBUG_NULLMAC 1

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
	if (rf1a_is_busy()) {
		// the radio is busy: do not proceed with the transmission
		DEBUG_PRINT_ERROR("Unicast not transmitted due to ongoing radio operation");
	} else {
		// the radio is not busy: proceed with the transmission
		nullmac_header_t header = prepare_header(destination);
		DEBUG_PRINT_INFO("Transmitting a %u-byte unicast to %u",
				NULLMAC_HEADER_LEN + payload_len, header.destination);
		rf1a_tx_packet((uint8_t *)&header, NULLMAC_HEADER_LEN, (uint8_t *)payload, payload_len);
	}
}

void broadcast_send(void *payload, uint8_t payload_len) {
	now = rtimer_now();
	if (rf1a_is_busy()) {
		// the radio is busy: do not proceed with the transmission
		DEBUG_PRINT_ERROR("Broadcast not transmitted due to ongoing radio operation");
	} else {
		// the radio is not busy: proceed with the transmission
		nullmac_header_t header = prepare_header(NULLMAC_BROADCAST_ADDR);
		DEBUG_PRINT_INFO("Transmitting a %u-byte broadcast", NULLMAC_HEADER_LEN + payload_len);
		rf1a_tx_packet((uint8_t *)&header, NULLMAC_HEADER_LEN, (uint8_t *)payload, payload_len);
	}
}

/*********************** RF1A callback implementation ***********************/

void rf1a_cb_rx_started(rtimer_clock_t *timestamp) {
	// notify about the beginning of the reception

}

void rf1a_cb_tx_started(rtimer_clock_t *timestamp) {
	// notify about the beginning of the transmission

}

void rf1a_cb_header_received(rtimer_clock_t *timestamp, uint8_t *header, uint8_t packet_len) {
	// as we set header_len to 0, this callback function will never be executed!
}

void rf1a_cb_rx_ended(rtimer_clock_t *timestamp, uint8_t *pkt, uint8_t pkt_len) {
	// notify about the successful reception
	nullmac_header_t header = *(nullmac_header_t *)pkt;
	uint8_t *payload = pkt + NULLMAC_HEADER_LEN;

	if (header.destination == NULLMAC_BROADCAST_ADDR) {
		// broadcast packet: execute the callback function
		DEBUG_PRINT(DEBUG_LVL_INFO, timestamp, 'I', "RX completed. Received a %u-byte broadcast from %u. RSSI: %d dBm, LQI: %u",
				pkt_len, header.source, rf1a_get_last_packet_rssi(), rf1a_get_last_packet_lqi());
		broadcast_received(timestamp, payload, pkt_len - NULLMAC_HEADER_LEN, header.source);
	} else {
		// unicast packet
		DEBUG_PRINT(DEBUG_LVL_INFO, timestamp, 'I', "RX completed. Received a %u-byte unicast from %u to %u. RSSI: %d dBm, LQI: %u",
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
	DEBUG_PRINT(DEBUG_LVL_INFO, timestamp, 'I', "TX completed");
	rf1a_start_rx();
}

void rf1a_cb_rx_failed(rtimer_clock_t *timestamp) {
	// notify about the failure, flush the RX FIFO and start a new reception attempt
	DEBUG_PRINT(DEBUG_LVL_ERROR, timestamp, 'E', "RX completed. Received a corrupted packet");
	rf1a_flush_rx_fifo();
	rf1a_start_rx();
}

void rf1a_cb_rx_tx_error(rtimer_clock_t *timestamp) {
	// notify about the error, flush both RX FIFO and TX FIFO and start a new reception attempt
	DEBUG_PRINT(DEBUG_LVL_ERROR, timestamp, 'E', "RX/TX error (interference?)");
	rf1a_flush_rx_fifo();
	rf1a_flush_tx_fifo();
	rf1a_start_rx();
}

#endif /* NULLMAC_CONF_ON */