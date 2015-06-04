#ifndef __GLOSSY_H__
#define __GLOSSY_H__

#include "contiki.h"

#define GLOSSY_COMMON_HEADER 0x80


#define SET_PKT_TYPE(pkt_type, sync, n_tx_max) (pkt_type = GLOSSY_COMMON_HEADER | ((sync) & 0x30) | ((n_tx_max) & 0x0f))
#define SET_SYNC(pkt_type, sync)               (pkt_type = ((pkt_type) & 0xcf) | ((sync) & 0x30))
#define SET_N_TX_MAX(pkt_type, n_tx_max)       (pkt_type = ((pkt_type) & 0xf0) | ((n_tx_max) & 0x0f))

#define GET_COMMON_HEADER(pkt_type)            ((pkt_type) & 0xc0)
#define GET_SYNC(pkt_type)                     ((pkt_type) & 0x30)
#define GET_N_TX_MAX(pkt_type)                 ((pkt_type) & 0x0f)

#define IS_INITIATOR()   (g.header.initiator_id == node_id)
#define WITH_SYNC()      (GET_SYNC(g.header.pkt_type) == GLOSSY_WITH_SYNC)
#define WITH_RELAY_CNT() ((WITH_SYNC()) || (GET_SYNC(g.header.pkt_type) == GLOSSY_ONLY_RELAY_CNT))

#ifdef GLOSSY_START_PIN
    #define GLOSSY_STARTED      LED_ON(GLOSSY_START_PIN)
    #define GLOSSY_RX_STARTED   LED_ON(GLOSSY_RX_PIN)
    #define GLOSSY_TX_STARTED   LED_ON(GLOSSY_TX_PIN)
    #define GLOSSY_STOPPED      LED_OFF(GLOSSY_START_PIN)
    #define GLOSSY_RX_STOPPED   LED_OFF(GLOSSY_RX_PIN)
    #define GLOSSY_TX_STOPPED   LED_OFF(GLOSSY_TX_PIN)
#else
    #define GLOSSY_STARTED      
    #define GLOSSY_RX_STARTED  
    #define GLOSSY_TX_STARTED  
    #define GLOSSY_STOPPED   
    #define GLOSSY_RX_STOPPED  
    #define GLOSSY_TX_STOPPED
#endif

enum {
	GLOSSY_UNKNOWN_INITIATOR = 0
};

enum {
	GLOSSY_UNKNOWN_N_TX_MAX = 0
};

enum {
	GLOSSY_UNKNOWN_PAYLOAD_LEN = 0
};

enum {
	SUCCESS = 0,
	FAIL = 1
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

typedef struct {
	uint16_t initiator_id;
	uint8_t pkt_type;
	uint8_t relay_cnt;
} glossy_header_t;

#define GLOSSY_HEADER_LEN(pkt_type) ((GET_SYNC(pkt_type) == GLOSSY_WITHOUT_SYNC) ? 3 : 4)

// NOTE: n_tx_max must be at most 15!
void glossy_start(uint16_t initiator_id, uint8_t *payload, uint8_t payload_len,
		uint8_t n_tx_max, glossy_sync_t sync, glossy_rf_cal_t rf_cal);

uint8_t glossy_stop(void);

uint8_t glossy_is_active(void);

uint8_t glossy_get_n_rx(void);

uint8_t glossy_get_n_tx(void);

uint8_t glossy_get_payload_len(void);

uint8_t glossy_is_t_ref_updated(void);

rtimer_clock_t glossy_get_t_ref(void);

uint8_t glossy_get_relay_cnt_first_rx(void);

#endif /* __GLOSSY_H__ */
