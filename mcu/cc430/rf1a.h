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
 */

/**
 * @addtogroup  Platform
 * @{
 *
 * @defgroup    rf1a RF1A
 * @{
 *
 * @file
 *
 */

#ifndef __RF1A_H__
#define __RF1A_H__

#include "rf1a-core.h"
#include "rtimer.h"


/* reception started callback */
extern void rf1a_cb_rx_started(rtimer_clock_t *timestamp);
/* reception of header callback */
extern void rf1a_cb_header_received(rtimer_clock_t *timestamp,
                                    uint8_t *header,
                                    uint8_t packet_len);
/* reception ended callback */
extern void rf1a_cb_rx_ended(rtimer_clock_t *timestamp,
                             uint8_t *pkt,
                             uint8_t pkt_len);
/* transmission started callback */
extern void rf1a_cb_tx_started(rtimer_clock_t *timestamp);
/* transmission ended callback */
extern void rf1a_cb_tx_ended(rtimer_clock_t *timestamp);
/* reception failed callback */
extern void rf1a_cb_rx_failed(rtimer_clock_t *timestamp);
/* reception or transmission error callback */
extern void rf1a_cb_rx_tx_error(rtimer_clock_t *timestamp);

/* initialize radio interface and radio core */
void rf1a_init(void);

/* reset radio interface and radio core */
void rf1a_reset(void);

uint8_t rf1a_is_busy(void);

/* set the TX power to a specified level */
void rf1a_set_tx_power(rf1a_tx_powers_t tx_power_level);

/* configure one of the three GDO signals */
void rf1a_configure_gdo_signal(uint8_t gdo, uint8_t signal, uint8_t invert);

/* get the status byte of the radio core */
/* rx = 1: returns the number of bytes currently in the RXFIFO queue */
/* rx = 0: returns the number of bytes currently in the TXFIFO queue */
uint8_t rf1a_get_status_byte(uint8_t rx);

/* put the radio into the SLEEP state */
void rf1a_go_to_sleep(void);

/* put the radio into the IDLE state */
void rf1a_go_to_idle(void);

/* start a reception */
void rf1a_start_rx(void);

/* transmit a packet */
/* NOTE: header_len should be at most 63 bytes */
void rf1a_tx_packet(uint8_t *header,
                    uint8_t header_len,
                    uint8_t *payload,
                    uint8_t payload_len);

/* force a manual calibration of the frequency synthesizer */
/* NOTE: the radio will be put into the IDLE state */
void rf1a_manual_calibration(void);

/* flush the RX FIFO */
/* NOTE: the radio will be put into the IDLE state */
void rf1a_flush_rx_fifo(void);

/* flush the TX FIFO */
/* NOTE: the radio will be put into the IDLE state */
void rf1a_flush_tx_fifo(void);

/* start a manual transmission */
/* NOTE: the packet must be already in the TX FIFO */
void rf1a_start_tx(void);

/* write a packet into the TX FIFO */
/* NOTE: header_len should be at most 63 bytes */
void rf1a_write_to_tx_fifo(uint8_t *header,
                           uint8_t header_len,
                           uint8_t *payload,
                           uint8_t payload_len);

/* set into which state the radio should go after a reception */
void rf1a_set_rxoff_mode(rf1a_off_modes_t mode);

/* set into which state the radio should go after a transmission */
void rf1a_set_txoff_mode(rf1a_off_modes_t mode);

/* get the current RSSI value */
int8_t rf1a_get_rssi(void);

/* get the LQI value currently stored into the LQI register */
uint8_t rf1a_get_lqi(void);

/* get the RSSI value appended to the last received packet */
int8_t rf1a_get_last_packet_rssi(void);

/* get the LQI value appended to the last received packet */
uint8_t rf1a_get_last_packet_lqi(void);

/* set the maximum allowed packet length */
void rf1a_set_maximum_packet_length(uint8_t length);

/* set the desired wireless channel */
void rf1a_set_channel(uint8_t channel);

/* set after how many bytes the MAC/Glossy layer should be notified about a
   header reception */
/* if set to 0, rf1a_cb_header_received() will never be called */
void rf1a_set_header_len_rx(uint8_t header_len);

/* set the calibration mode */
void rf1a_set_calibration_mode(rf1a_calibration_modes_t mode);

/* clear any pending interrupts */
void rf1a_clear_pending_interrupts(void);

#endif /* __RF1A_H__ */

/**
 * @}
 * @}
 */
