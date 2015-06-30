 /**
  * Helper macros for the configuration and usage of the timer modules TA0 and TA1
  *
  * @file
  */
  
#ifndef __RTIMER_H__
#define __RTIMER_H__

/**
 * @brief the number of usable rtimers of module TA0
 * @note if the radio module is used, one CCR is reserved for it and cannot be used otherwise
 */
#ifdef WITH_RADIO
  #define N_RTIMERS       4
#else
  #define N_RTIMERS       5
#endif /* WITH_RADIO */

/**
 * @brief the number of timer ticks that (approx.) correspond to 1s
 * @note the timer modules are clock by SMCLK at 3.25 MHz
 */
#define RTIMER_SECOND           ((rtimer_clock_t)SMCLK_SPEED)  // for TA0
#define RTIMER_SECOND_TA1       (ACLK_SPEED)                   // for TA1


/**
 * @brief enable the update (overflow) interrupt of the timer TA0
 */
#define RTIMER_UPDATE_ENABLE        { TA0CTL |= TAIE; TA1CTL |= TAIE; }
/**
 * @brief disable the update (overflow) interrupt of the timer TA0
 */
#define RTIMER_UPDATE_DISABLE       { TA0CTL &= ~TAIE; TA1CTL &= ~TAIE; }
/**
 * @brief checks if the update (overflow) interrupt of the timer TA0 is enabled
 */
#define RTIMER_IS_UPDATE_ENABLED    ( (TA0CTL & TAIE) > 0 )


/**
 * @brief the rtimer IDs for timer module TA0 (number of IDs corresponds to the number of CCRs)
 */
typedef enum {
    RTIMER_TA0_0 = 0,    // TA0 CCR0
    RTIMER_TA0_1,
    RTIMER_TA0_2,
    RTIMER_TA0_3,
    RTIMER_TA0_4,
    RTIMER_TA1_0,        // TA1 CCR0
    RTIMER_TA1_1,
    RTIMER_TA1_2,
    NUM_OF_RTIMERS
} rtimer_id_t;

/**
 * @brief the rtimer states
 */
typedef enum {
	RTIMER_INACTIVE = 0,
	RTIMER_SCHEDULED = 1,
	RTIMER_JUST_EXPIRED = 2
} rtimer_state_t;

typedef uint64_t rtimer_clock_t;
typedef struct rtimer_t rtimer_t;

typedef char (*rtimer_callback_t)(rtimer_t *rt);        // prototype of a rtimer callback function
typedef void (*rtimer_ta1_callback_t)(void);


/**
 * @brief state and parameters of an rtimer
 */
typedef struct rtimer_t {
	rtimer_callback_t func; // callback function to execute when the rtimer expires
	rtimer_state_t state;   // internal state of the rtimer
	rtimer_clock_t period;  // if period = 0: one-shot timer; otherwise: timer period in clock ticks
	rtimer_clock_t time;    // if state = RTIMER_SCHEDULED: next expiration time; otherwise: last expiration time
} rtimer_t;


// initialize the rtimers
void rtimer_init(void);

// schedule rtimer timer to execute function func with a certain period, starting from start
// if period is 0, then func is executed only once
void rtimer_schedule(rtimer_id_t timer, rtimer_clock_t start, rtimer_clock_t period, rtimer_callback_t func);

// stop rtimer timer
void rtimer_stop(rtimer_id_t timer);

// get the period of rtimer timer
rtimer_clock_t rtimer_get_period(rtimer_id_t timer);

// get the expiration time of rtimer timer
rtimer_clock_t rtimer_get_expiration_time(rtimer_id_t timer);

// get the current rtimer time (TA0)
rtimer_clock_t rtimer_now(void);

// get the current rtimer time (TA1)
rtimer_clock_t rtimer_now_ta1(void);

// convert rtimer time values from clock ticks to milliseconds
#define RTIMER_TO_MS(t) ((t) / (RTIMER_SECOND/1000))

// get the current rtimer time (macro used in several Contiki core files)
#define RTIMER_NOW    (rtimer_now())

// get the current rtimer time in milliseconds
#define RTIMER_NOW_MS() (RTIMER_TO_MS(rtimer_now()))

#define NS_TO_RTIMER_TICKS(ns) (((rtimer_clock_t)(ns) * (rtimer_clock_t)RTIMER_SECOND) / (rtimer_clock_t)1000000000)

extern volatile rtimer_clock_t ta0_sw_ext;


#endif /* __RTIMER_H__ */
