/**
 * @file 
 * @author Federico Ferrari
 *         Reto Da Forno
 * 
 * @note
 * required files:
 * - rtimer driver
 * - radio module driver
 * - stdlib
 */


#ifndef __GLOSSY_H__
#define __GLOSSY_H__


#include "contiki.h"


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
 * @brief start Glossy
 * 
 * start Glossy, i.e. initiate a flood (if this node is the initiator) or switch to RX mode (receive/relay packets)
 * 
 * @param[in] initiator_id node ID of the initiator, use GLOSSY_UNKNOWN_INITIATOR if the initiator is unknown
 * @param[in] payload pointer to the packet data
 * @param[in] payload_len length of the data, must not exceed PACKET_LEN_MAX
 * @param[in] n_tx_max maximum number of retransmissions
 * @param[in] sync synchronization mode
 * @note  n_tx_max must be at most 15!
 */
void glossy_start(uint16_t initiator_id, uint8_t *payload, uint8_t payload_len, uint8_t n_tx_max, glossy_sync_t sync, glossy_rf_cal_t rf_cal);

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
 * @brief get the number of received bytes
 */
uint8_t glossy_get_n_rx(void);

/**
 * @brief get the number of transmitted bytes
 */
uint8_t glossy_get_n_tx(void);

/**
 * @brief get the length of the payload of the received/transmitted packet
 */
uint8_t glossy_get_payload_len(void);

/**
 * @brief checks whether the reference time has been updated in the last glossy flood
 */
uint8_t glossy_is_t_ref_updated(void);

/**
 * @brief get the reference time (timestamp of the reception of the first byte)
 */
rtimer_clock_t glossy_get_t_ref(void);

/**
 * @brief get the relay count of the first received packet
 */
uint8_t glossy_get_relay_cnt_first_rx(void);


#endif /* __GLOSSY_H__ */
