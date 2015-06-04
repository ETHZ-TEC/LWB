
#ifndef SIMPLE_LWB_H
#define SIMPLE_LWB_H

#include "contiki.h"

#define DEBUG_LWB 1

#define DEBUG(l, t, ...)    if (DEBUG_LWB >= l) { DEBUG_PRINT_MSG(t, "S-LWB", __VA_ARGS__); }


/* CONFIGURATION */

#define HOST_ID         6
#define TX_POWER        RF1A_TX_POWER_MINUS_12_dBm
#define TX_CHANNEL      1
#define PACKET_LEN_MAX  127
#define T_INIT          5                           // initial round period: 5 seconds
#define T_SCHED         (RTIMER_SECOND / 30)        // length of a schedule slot: 33 ms
#define T_DATA          (RTIMER_SECOND / 15)        // length of a data slot: 66 ms (may be reduce this time, it's 10ms in tmote implementation)
#define T_GAP           (RTIMER_SECOND / 250)       // gap between two consecutive slots: 4 ms (can potentially be reduced)
#define T_GUARD         (RTIMER_SECOND / 2000)      // 0.5 ms
#define T_GUARD_1       (RTIMER_SECOND / 333)       //   3 ms
#define T_GUARD_2       (RTIMER_SECOND / 200)       //   5 ms
#define T_GUARD_3       (RTIMER_SECOND /  50)       //  20 ms


#define N_TX_MAX_SCHEDULE   5       // max. number of retransmissions / hops for the schedule
#define N_TX_MAX_DATA       3       // max. number of retransmissions / hops for a data packet
#define N_SLOTS_MAX         60      // max. number of data slots per round
#define N_STREAMS_MAX       120     // max. number of streams (bounds the required memory)

// modifies or checks the last bit of a byte
#define SET_LAST_BIT(b)     (b |= 0x80)
#define CLEAR_LAST_BIT(b)   (b &= ~0x80)
#define IS_LAST_BIT_SET(b)  (b & 0x80)

static uint8_t nodes[] = {1, 2, 4, 6, 7, 8};


#define SCHED_HEADER_LENGTH  9
typedef struct {	
	uint16_t host_id;
	uint32_t time;
	uint8_t period;
	uint8_t seq_no;
	uint8_t n_slots;
	uint8_t slot[PACKET_LEN_MAX - SCHED_HEADER_LENGTH];
} schedule_t;

typedef struct {
	uint8_t  relay_cnt;
	uint32_t data_tot;
	uint32_t data_rcvd;
    int32_t skew_last;
    uint8_t period_last;
	rtimer_clock_t last_t_ref;
} statistics_t;

typedef enum {
	NOT_JOINED,
	JOINING,
	JOINED,
	JUST_TRIED
} joining_state_t;

typedef enum {
	STATE_BOOTSTRAP = 0,
	STATE_QUASI_SYNCED,
	STATE_SYNCED,
	STATE_ALREADY_SYNCED,
	STATE_UNSYNCED_1,
	STATE_UNSYNCED_2,
	STATE_UNSYNCED_3,
    NUM_OF_SYNC_STATES
} sync_state_t;

const char* sync_state_to_string[NUM_OF_SYNC_STATES] = { "BOOTSTRAP", "QUASI_SYNCED", "SYNCED", "ALREADY_SYNCED", "UNSYNCED_1", "UNSYNCED_2", "UNSYNCED_3" };

typedef enum {
	EVT_1ST_SCHED_RCVD = 0,
    EVT_2ND_SCHED_RCVD,
	EVT_SCHED_MISSED,
    NUM_OF_SYNC_EVENTS
} sync_event_t;


PT_THREAD(simple_lwb_thread(rtimer_t *rt));
PROCESS_NAME(simple_lwb_process);

#endif
