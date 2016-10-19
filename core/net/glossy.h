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
 *          Reto Da Forno
 *          Marco Zimmerling
 */

/**
 * @addtogroup  Net
 * @{
 *
 * @defgroup    glossy Glossy
 * @{
 *
 * @file
 * 
 * @brief Wireless Communication Protocol 'Glossy'
 */

#ifndef __GLOSSY_H__
#define __GLOSSY_H__


#include <stdlib.h>


/* whether the initiator should retransmit the packet after a certain
 * time of no reception */
#ifndef GLOSSY_CONF_RETRANSMISSION_TIMEOUT
#define GLOSSY_CONF_RETRANSMISSION_TIMEOUT      1
#endif /* GLOSSY_CONF_RETRANSMISSION_TIMEOUT */

#ifndef GLOSSY_CONF_RTIMER_ID
#define GLOSSY_CONF_RTIMER_ID                   RTIMER_HF_3
#endif /* GLOSSY_CONF_RTIMER_ID */

#ifndef GLOSSY_CONF_COLLECT_STATS
#define GLOSSY_CONF_COLLECT_STATS               1
#endif /* GLOSSY_CONF_COLLECT_STATS */

/* max. header length (with sync) */
#define GLOSSY_MAX_HEADER_LEN                   4


enum {
  GLOSSY_UNKNOWN_INITIATOR = 0
};

enum {
  GLOSSY_UNKNOWN_N_TX_MAX = 0
};

enum {
  GLOSSY_UNKNOWN_PAYLOAD_LEN = 0
};

typedef enum {
  GLOSSY_WITHOUT_RF_CAL = 0,
  GLOSSY_WITH_RF_CAL = 1,
} glossy_rf_cal_t;

typedef enum {
  GLOSSY_UNKNOWN_SYNC = 0x00,
  GLOSSY_WITH_SYNC = 0x10,
  GLOSSY_WITHOUT_SYNC = 0x20,
  GLOSSY_ONLY_RELAY_CNT = 0x30
} glossy_sync_t;

/**
 * @brief       start Glossy
 * @param[in]   initiator_id node ID of the initiator, use
 *              GLOSSY_UNKNOWN_INITIATOR if the initiator is unknown
 * @param[in]   payload pointer to the packet data
 * @param[in]   payload_len length of the data, must not exceed PACKET_LEN_MAX
 * @param[in]   n_tx_max maximum number of retransmissions
 * @param[in]   sync synchronization mode
 * @note        n_tx_max must be at most 15!
 *
 * start Glossy, i.e. initiate a flood (if node is initiator) or switch to RX
 * mode (receive/relay packets)
 */
void glossy_start(uint16_t initiator_id,
                  uint8_t *payload,
                  uint8_t payload_len,
                  uint8_t n_tx_max,
                  glossy_sync_t sync,
                  glossy_rf_cal_t rf_cal);

/**
 * @brief stop glossy
 */
uint8_t glossy_stop(void);

/**
 * @brief query activity of glossy
 * @return the number of received bytes since glossy_start was called
 */
uint8_t glossy_is_active(void);

/**
 * @brief get the number of received packets
 * during the last flood
 */
uint8_t glossy_get_n_rx(void);

/**
 * @brief get the number of transmitted packets during the last flood
 */
uint8_t glossy_get_n_tx(void);

/**
 * @brief get the length of the payload of the received/transmitted packet
 */
uint8_t glossy_get_payload_len(void);

/**
 * @brief checks whether the reference time has been updated in the last
 * glossy flood
 */
uint8_t glossy_is_t_ref_updated(void);

/**
 * @brief get the reference time (timestamp of the reception of the first byte)
 * @return 64-bit timestamp (type rtimer_clock_t)
 */
uint64_t glossy_get_t_ref(void);


#if GLOSSY_CONF_COLLECT_STATS
/**
 * @brief get the number of started receptions
 * (preamble + sync detection) during the last flood
 */
uint8_t glossy_get_n_rx_started(void);

/**
 * @brief get the total number of received packets with CRC ok
 */
uint8_t glossy_get_n_crc_ok(void);

/**
 * @brief get the number of received packets with CRC failed during the last
 * flood
 */
uint8_t glossy_get_n_rx_fail(void);

/**
 * @brief get the signal-to-noise ratio (average RSSI value of all received 
 * packets of the last flood minus RSSI of the noise floor before the flood)
 * @note this function is only returns a value != 0 if at least one packet 
 * was successfully received in the last slot and the node was not the
 * initiator of the flood
 */
int8_t glossy_get_snr(void);

/**
 * @brief get the average RSSI value of the last flood
 * @param out rssi optional parameter, returns the individual RSSI values
 * (buffer must be able to hold 3 bytes!)
 */
int8_t glossy_get_rssi(int8_t* rssi);

/**
 * @brief get the relay count of the first received packet
 */
uint8_t glossy_get_relay_cnt_first_rx(void);

/**
 * @brief get the relay count of the last 3 packets (last flood)
 * @return 3x 4 bits (relay counter of the received packets)
 */
uint16_t glossy_get_relay_cnt(void);

/**
 * @brief get the packet error rate/ratio in percentages * 100
 */
uint16_t glossy_get_per(void);

/**
 * @brief flood success rate (percentage of successful floods), not counting
 * floods where no communication was detected (absence of preamble + sync byte)
 * and the initiated floods
 */
uint16_t glossy_get_fsr(void);

/**
 * @brief get the total number of received packets including corrupt packets
 */
uint32_t glossy_get_n_pkts(void);

/**
 * @brief get the total number of received packets (with CRC ok only)
 */
uint32_t glossy_get_n_pkts_crcok(void);

/**
 * @brief get the total number of errors which occurred (RF RX/TX error)
 */
uint16_t glossy_get_n_errors();

/**
 * @brief get the duration of the last flood, in HF clock ticks
 */
uint32_t glossy_get_flood_duration(void);

/**
 * @brief get the time to the first RX (since glossy_start), in HF clock ticks
 */
uint32_t glossy_get_t_to_first_rx(void);

#endif /* GLOSSY_CONF_COLLECT_STATS */


#endif /* __GLOSSY_H__ */

/**
 * @}
 * @}
 */
