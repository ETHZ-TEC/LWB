#include "contiki.h"

// number of TX power levels
// {-30, -12, -6, 0, 10, max} dBm
#define N_TX_POWER_LEVELS     (RF1A_TX_POWER_MAX + 1)

// number of bytes to read from the RX FIFO (or to write to the TX FIFO)
// after crossing a FIFO threshold
// must be between 3 and 63, in steps of 4 (e.g., 3, 7, 11, ...)
#define FIFO_CHUNK_SIZE       15

// state of the radio core
static rf1a_rx_tx_states_t rf1a_state;

// buffer used to manage packets longer than the RX/TX FIFO size (64 bytes)
// force its address to be even in order to avoid misalignment issues
// when executing the callback functions
static uint8_t rf1a_buffer[RF1A_MAX_PACKET_LENGTH] __attribute__ ((aligned(2)));

// variables to indicate where is the starting point of the buffer (used for TX) and its length
static uint8_t rf1a_buffer_start, rf1a_buffer_len;

// length of the packet being received or transmitted
static uint8_t packet_len;

static uint8_t packet_len_max;

static uint8_t header_len_rx, header_len_notified;

// timestamp of radio events
static rtimer_clock_t timestamp;

// automatic mode switches at the end of RX and TX
static rf1a_off_modes_t rxoff_mode, txoff_mode;

static inline void read_bytes_from_rx_fifo(uint8_t n_bytes) {
	if (rf1a_buffer_len + n_bytes <= packet_len_max) {
		if (rf1a_buffer_len == 0) {
			// no bytes read from the RX FIFO yet: store the first one as the packet length
			packet_len = read_byte_from_register(RXFIFO);
			n_bytes--;
		}
		// read the bytes from the RX FIFO and append them to the buffer
		read_data_from_register(RXFIFO, &rf1a_buffer[rf1a_buffer_len], n_bytes);
		// update the buffer indexes
		rf1a_buffer_len += n_bytes;

		if ((header_len_rx != 0) && (header_len_notified == 0) && (rf1a_buffer_len >= header_len_rx)) {
			rf1a_cb_header_received(&timestamp, rf1a_buffer, packet_len);
			header_len_notified = 1;
		}

	} else {
		// too many bytes: something went wrong
		rf1a_cb_rx_tx_error(&timestamp);
	}
}

static inline void energest_off_mode(rf1a_off_modes_t off_mode) {
	SET_ENERGEST_TIME();
	switch (off_mode) {
	case RF1A_OFF_MODE_IDLE:
		ENERGEST_OFF_AT_TIME(ENERGEST_TYPE_LISTEN);
		ENERGEST_OFF_AT_TIME(ENERGEST_TYPE_TRANSMIT);
		ENERGEST_ON_AT_TIME(ENERGEST_TYPE_IDLE);
		break;
	case RF1A_OFF_MODE_RX:
		ENERGEST_OFF_AT_TIME(ENERGEST_TYPE_IDLE);
		ENERGEST_OFF_AT_TIME(ENERGEST_TYPE_TRANSMIT);
		ENERGEST_ON_AT_TIME(ENERGEST_TYPE_LISTEN);
		break;
	case RF1A_OFF_MODE_TX:
	case RF1A_OFF_MODE_FSTXON:
		ENERGEST_OFF_AT_TIME(ENERGEST_TYPE_LISTEN);
		ENERGEST_OFF_AT_TIME(ENERGEST_TYPE_IDLE);
		ENERGEST_ON_AT_TIME(ENERGEST_TYPE_TRANSMIT);
		break;
	}
}

void rf1a_init(void) {

	// reset the radio core
	rf1a_reset();

	rxoff_mode = RF1A_OFF_MODE_IDLE;
	txoff_mode = RF1A_OFF_MODE_IDLE;

	packet_len_max = RF1A_MAX_PACKET_LENGTH;

	load_SmartRF_configuration();

	// register settings corresponding to TX power levels {-30, -12, -6, 0, +10, max} dBm
	// NOTE: these values work only for 2-FSK, 2-GFSK, and MSK modulation schemes
	static const uint8_t pa_values[N_TX_POWER_LEVELS] = {0x03, 0x25, 0x2d, 0x8d, 0xc3, 0xc0};
	write_data_to_register(PATABLE, (uint8_t *)pa_values, N_TX_POWER_LEVELS);

	//-----------------------------------------------------------------------------

	// set the FIFO threshold such that FIFO_CHUNK_SIZE bytes can be written/read after an interrupt
	write_byte_to_register(FIFOTHR, (FIFO_CHUNK_SIZE - 3) / 4);

	// map the sync word signal to GDO2 (corresponding to GDO2_CFG = 0x06, see Table 25-21)
	rf1a_configure_gdo_signal(2, 6, 0);
	// timer 4: capture input CCI4B, which corresponds to GDO2, i.e., to the sync word signal
	// synchronous capture on both edges
	// NOTE: as a result timer 4 is not available for being used as a rtimer
	TA0CCTL4 = CM_3 | CCIS_1 | CAP | SCS;

	// interrupt when the number of bytes in the RX FIFO is greater than the threshold
	ENABLE_INTERRUPTS(BIT3, 1);
//	// interrupt when the number of bytes in the TX FIFO is smaller than the threshold
//	ENABLE_INTERRUPTS(BIT5, 0);
	// interrupt when RX FIFO overflows (RFIFG7)
	ENABLE_INTERRUPTS(BIT7, 1);
	// interrupt when TX FIFO underflows (RFIFG8)
	ENABLE_INTERRUPTS(BIT8, 1);
	// interrupt when sync word received or transmitted, or end of packet (RFIFG9)
	ENABLE_INTERRUPTS(BIT9, 1);
}

void rf1a_reset(void) {
	// NOTE: the radio goes to the SLEEP state (see 25.3.1)
	SET_ENERGEST_TIME();
	ENERGEST_OFF_AT_TIME(ENERGEST_TYPE_LISTEN);
	ENERGEST_OFF_AT_TIME(ENERGEST_TYPE_TRANSMIT);
	ENERGEST_OFF_AT_TIME(ENERGEST_TYPE_IDLE);
	// issue the SRES command strobe
	strobe(RF_SRES, 1);
	// issue the SNOP command strobe
	strobe(RF_SNOP, 1);
	// clear all radio interface error interrupt flags
	RF1AIFERR = 0;

	header_len_rx = 0;

	rf1a_state = NO_RX_TX;
}

uint8_t rf1a_is_busy(void) {
	return (rf1a_state == NO_RX_TX) ? 0 : 1;
}

void rf1a_set_tx_power(rf1a_tx_powers_t tx_power_level) {
	if (tx_power_level < N_TX_POWER_LEVELS) {
		set_register_field(FREND0, tx_power_level, 3, 0);
	}
}

void rf1a_configure_gdo_signal(uint8_t gdo, uint8_t signal, uint8_t invert) {
	if (gdo <= 2) {
		write_byte_to_register(IOCFG0 - gdo, (signal & 0x3f) | (invert << 6));
	}
}

uint8_t rf1a_get_status_byte(uint8_t rx) {
	// issue the SNOP command strobe in order to get the radio core status byte
	// without causing any further actions
	return strobe(RF_SNOP, 1);
}

void rf1a_go_to_sleep(void) {
	// issue the SXOFF command strobe
	strobe(RF_SXOFF, 1);
	rf1a_state = NO_RX_TX;

	SET_ENERGEST_TIME();
	ENERGEST_OFF_AT_TIME(ENERGEST_TYPE_LISTEN);
	ENERGEST_OFF_AT_TIME(ENERGEST_TYPE_TRANSMIT);
	ENERGEST_OFF_AT_TIME(ENERGEST_TYPE_IDLE);
}

void rf1a_go_to_idle(void) {
	// issue the SIDLE command strobe
	uint8_t status = strobe(RF_SIDLE, 1);
	// wait until the radio core goes to the IDLE state
	while (GET_RF_STATE(status) != RF_STATE_IDLE) {
		status = strobe(RF_SNOP, 1);
	}
	rf1a_state = NO_RX_TX;

	SET_ENERGEST_TIME();
	ENERGEST_OFF_AT_TIME(ENERGEST_TYPE_LISTEN);
	ENERGEST_OFF_AT_TIME(ENERGEST_TYPE_TRANSMIT);
	ENERGEST_ON_AT_TIME(ENERGEST_TYPE_IDLE);
}

void rf1a_manual_calibration(void) {
	// issue the SIDLE command strobe
	uint8_t status = strobe(RF_SIDLE, 1);
	// wait until the radio core goes to the IDLE state
	while (GET_RF_STATE(status) != RF_STATE_IDLE) {
		status = strobe(RF_SNOP, 1);
	}
	rf1a_state = NO_RX_TX;

	SET_ENERGEST_TIME();
	ENERGEST_OFF_AT_TIME(ENERGEST_TYPE_LISTEN);
	ENERGEST_OFF_AT_TIME(ENERGEST_TYPE_TRANSMIT);
	ENERGEST_ON_AT_TIME(ENERGEST_TYPE_IDLE);

	// then issue the SCAL command strobe
	strobe(RF_SCAL, 1);
	// wait until the calibration is finished
	while (read_byte_from_register(MARCSTATE) != 1);
}

void rf1a_flush_rx_fifo(void) {
	// issue the SIDLE command strobe
	uint8_t status = strobe(RF_SIDLE, 1);
	// wait until the radio core goes to the IDLE state
	while (GET_RF_STATE(status) != RF_STATE_IDLE) {
		status = strobe(RF_SNOP, 1);
	}
	rf1a_state = NO_RX_TX;

	SET_ENERGEST_TIME();
	ENERGEST_OFF_AT_TIME(ENERGEST_TYPE_LISTEN);
	ENERGEST_OFF_AT_TIME(ENERGEST_TYPE_TRANSMIT);
	ENERGEST_ON_AT_TIME(ENERGEST_TYPE_IDLE);

	// then issue the SFRX command strobe
	strobe(RF_SFRX, 1);
}

void rf1a_flush_tx_fifo(void) {
	// issue the SIDLE command strobe
	uint8_t status = strobe(RF_SIDLE, 1);
	// wait until the radio core goes to the IDLE state
	while (GET_RF_STATE(status) != RF_STATE_IDLE) {
		status = strobe(RF_SNOP, 1);
	}
	rf1a_state = NO_RX_TX;

	SET_ENERGEST_TIME();
	ENERGEST_OFF_AT_TIME(ENERGEST_TYPE_LISTEN);
	ENERGEST_OFF_AT_TIME(ENERGEST_TYPE_TRANSMIT);
	ENERGEST_ON_AT_TIME(ENERGEST_TYPE_IDLE);

	// then issue the SFTX command strobe
	strobe(RF_SFTX, 0);
}

void rf1a_start_rx(void) {
	SET_ENERGEST_TIME();
	ENERGEST_OFF_AT_TIME(ENERGEST_TYPE_IDLE);
	ENERGEST_OFF_AT_TIME(ENERGEST_TYPE_TRANSMIT);
	ENERGEST_ON_AT_TIME(ENERGEST_TYPE_LISTEN);

	// issue the SRX command strobe
	strobe(RF_SRX, 1);
}

void rf1a_start_tx(void) {
	SET_ENERGEST_TIME();
	ENERGEST_OFF_AT_TIME(ENERGEST_TYPE_IDLE);
	ENERGEST_OFF_AT_TIME(ENERGEST_TYPE_LISTEN);
	ENERGEST_ON_AT_TIME(ENERGEST_TYPE_TRANSMIT);

	// issue the STX command strobe
	strobe(RF_STX, 0);
	rf1a_state = TX;
}

void rf1a_write_to_tx_fifo(uint8_t *header, uint8_t header_len, uint8_t *payload, uint8_t payload_len) {
	// ensure that the TX FIFO threshold interrupt is disabled
	DISABLE_INTERRUPTS(BIT5);

	// check that the total packet length does not exceed the maximum allowed
	packet_len = header_len + payload_len;
	if (packet_len > packet_len_max) {
		return;
	}

	// write the packet length into the TX FIFO
	write_byte_to_register(TXFIFO, packet_len);

	// write the header into the TX FIFO
	// NOTE: it is assumed here that header_len is at most 63 bytes!
	write_data_to_register(TXFIFO, header, header_len);

	// compute the number of bytes still available in the TX FIFO
	uint8_t free_tx_fifo_bytes = 63 - header_len;
	// compute the number of payload bytes to write immediately into the TX FIFO
	uint8_t payload_bytes_to_tx_fifo;
	if (payload_len > free_tx_fifo_bytes) {
		// write the first free_tx_fifo_bytes bytes of the payload into the TX FIFO
		payload_bytes_to_tx_fifo = free_tx_fifo_bytes;
	} else {
		// write the whole payload into the TX FIFO
		payload_bytes_to_tx_fifo = payload_len;
	}
	write_data_to_register(TXFIFO, payload, payload_bytes_to_tx_fifo);

	// store the whole packet (header + payload) into the buffer
	memcpy(rf1a_buffer, header, header_len);
	memcpy(&rf1a_buffer[header_len], payload, payload_len);
	// mark that the header and payload_bytes_to_tx_fifo payload bytes
	// have already been written into the TX FIFO
	rf1a_buffer_start = header_len + payload_bytes_to_tx_fifo;
	rf1a_buffer_len = packet_len - rf1a_buffer_start;

	if (payload_len > free_tx_fifo_bytes) {
		// enable the TX FIFO threshold interrupt
		ENABLE_INTERRUPTS(BIT5, 0);
	}
}

void rf1a_tx_packet(uint8_t *header, uint8_t header_len, uint8_t *payload, uint8_t payload_len) {
	// flush the TX FIFO
	rf1a_flush_tx_fifo();
	// start writing the packet to the TX FIFO
	rf1a_write_to_tx_fifo(header, header_len, payload, payload_len);
	// start the transmission
	rf1a_start_tx();
}

void rf1a_set_rxoff_mode(rf1a_off_modes_t mode) {
	set_register_field(MCSM1, mode, 2, 2);
	rxoff_mode = mode;
}

void rf1a_set_txoff_mode(rf1a_off_modes_t mode) {
	set_register_field(MCSM1, mode, 2, 0);
	txoff_mode = mode;
}

int8_t rf1a_get_rssi(void) {
	// transform the RSSI value to dBm (see Section 25.3.3.6.3)
	int8_t rssi = (int8_t)read_byte_from_register(RSSI) / 2 - RSSI_OFFSET;
	return rssi;
}

uint8_t rf1a_get_lqi(void) {
	uint8_t lqi = read_byte_from_register(LQI) & LQI_MASK;
	return lqi;
}

int8_t rf1a_get_last_packet_rssi(void) {
	// extract the RSSI value appended to the packet and transform it to dBm (see Section 25.3.3.6.3)
	int8_t rssi = (int8_t)rf1a_buffer[packet_len] / 2 - RSSI_OFFSET;
	return rssi;
}

uint8_t rf1a_get_last_packet_lqi(void) {
	// extract the LQI value appended to the packet
	uint8_t lqi = rf1a_buffer[packet_len + 1] & LQI_MASK;
	return lqi;
}

void rf1a_set_maximum_packet_length(uint8_t length) {
	// ensure that we are in variable packet length mode
	set_register_field(PKTCTRL0, 0x1, 2, 0);
	// set the maximum packet length
	if (length > RF1A_MAX_PACKET_LENGTH) {
		packet_len_max = RF1A_MAX_PACKET_LENGTH;
	} else {
		packet_len_max = length;
	}
	write_byte_to_register(PKTLEN, packet_len_max);
}

void rf1a_set_channel(uint8_t channel) {
	write_byte_to_register(CHANNR, channel);
}

void rf1a_set_header_len_rx(uint8_t header_len) {
	header_len_rx = header_len;
}

void rf1a_set_calibration_mode(rf1a_calibration_modes_t mode) {
	set_register_field(MCSM0, mode, 2, 4);
}

void rf1a_clear_pending_interrupts(void) {
	// disable TX threshold interrupt
	DISABLE_INTERRUPTS(BIT5);
	// enable sync word interrupt (positive edge)
	ENABLE_INTERRUPTS(BIT9, 1);
	// clear all interrupt flags
	RF1AIFG = 0;
}

ISR(CC1101, radio_interrupt) {

	ENERGEST_ON(ENERGEST_TYPE_CPU);

	// take a timestamp
	timestamp = rtimer_now();

	switch (RF1AIV) {

	case RF1AIV_RFIFG3:
		// RX FIFO above threshold: read FIFO_CHUNK_SIZE bytes
		read_bytes_from_rx_fifo(FIFO_CHUNK_SIZE);
		break;

	case RF1AIV_RFIFG5:
		// TX FIFO below threshold
		if (rf1a_buffer_len > 0) {
			// there are still bytes to write into the TX FIFO
			if (rf1a_buffer_len > FIFO_CHUNK_SIZE) {
				// write FIFO_CHUNK_SIZE more bytes into the TX FIFO
				write_data_to_register(TXFIFO, &rf1a_buffer[rf1a_buffer_start], FIFO_CHUNK_SIZE);
				// update the buffer indexes
				rf1a_buffer_start += FIFO_CHUNK_SIZE;
				rf1a_buffer_len -= FIFO_CHUNK_SIZE;
			} else {
				// write the remaining rf1a_buffer_len bytes into the TX FIFO
				write_data_to_register(TXFIFO, &rf1a_buffer[rf1a_buffer_start], rf1a_buffer_len);
				// reset the buffer indexes
				rf1a_buffer_start = 0;
				rf1a_buffer_len = 0;
				// no more bytes left to write into the the TX FIFO:
				// disable the TX FIFO threshold interrupt
				DISABLE_INTERRUPTS(BIT5);
			}
		}
		break;

	case RF1AIV_RFIFG7:
		// RX FIFO overflowed
		rf1a_cb_rx_tx_error(&timestamp);
		rf1a_state = NO_RX_TX;
		break;

	case RF1AIV_RFIFG8:
		// TX FIFO underflowed
		rf1a_cb_rx_tx_error(&timestamp);
		rf1a_state = NO_RX_TX;
		break;

	case RF1AIV_RFIFG9:
		// sync word received or transmitted, or end of packet

		// correct the timestamp based on the time captured by timer 4
		timestamp = timestamp - ((uint16_t)(timestamp & 0xffff) - TA0CCR4);

		if (!(RF1AIES & BIT9)) {
			// sync word received or transmitted
			switch (GET_RF_STATE(rf1a_get_status_byte(1))) {
			case RF_STATE_RX:
				// sync word received
				rf1a_state = RX;
				// reset the buffer indexes
				rf1a_buffer_len = 0;
				rf1a_buffer_start = 0;
				header_len_notified = 0;
				// invert the edge for the next interrupt
				INVERT_INTERRUPT_EDGES(BIT9);
				rf1a_cb_rx_started(&timestamp);
				break;
			case RF_STATE_TX:
				// sync word transmitted
				rf1a_state = TX;
				// invert the edge for the next interrupt
				INVERT_INTERRUPT_EDGES(BIT9);
				rf1a_cb_tx_started(&timestamp);
				break;
			default:
				// RX or TX already ended, or some other error has occurred
				rf1a_state = NO_RX_TX;
				rf1a_cb_rx_tx_error(&timestamp);
			}
		} else {
			// end of packet
			switch (rf1a_state) {
			case RX:
				// RX ended
				rf1a_state = NO_RX_TX;
				energest_off_mode(rxoff_mode);

				if (read_byte_from_register(PKTSTATUS) & BIT7) {
					// CRC OK
					// read the remaining bytes from the RX FIFO
					read_bytes_from_rx_fifo(read_byte_from_register(RXBYTES));
					// invert the edge for the next interrupt
					INVERT_INTERRUPT_EDGES(BIT9);
					// execute the callback function
					rf1a_cb_rx_ended(&timestamp, rf1a_buffer, packet_len);
				} else {
					// CRC not OK
					// invert the edge for the next interrupt
					INVERT_INTERRUPT_EDGES(BIT9);
					rf1a_cb_rx_failed(&timestamp);
				}
				break;
			case TX:
				// TX ended
				rf1a_state = NO_RX_TX;
				energest_off_mode(txoff_mode);
				// invert the edge for the next interrupt
				INVERT_INTERRUPT_EDGES(BIT9);
				rf1a_cb_tx_ended(&timestamp);
				break;
			default:
				// there is no RX or TX to end, some error must have occurred
				rf1a_state = NO_RX_TX;
				// invert the edge for the next interrupt
				INVERT_INTERRUPT_EDGES(BIT9);
				rf1a_cb_rx_tx_error(&timestamp);
			}
		}
		break;
	}

	ENERGEST_OFF(ENERGEST_TYPE_CPU);

}
