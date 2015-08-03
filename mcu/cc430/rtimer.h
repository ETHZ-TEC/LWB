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
 * Authors      Federico Ferrari
 *              Reto Da Forno
 */

/**
 * @addtogroup  Platform
 * @{
 *
 * @defgroup    rtimer Realtime timer
 * @{
 *
 * @file
 * @author      
 *              Federico Ferrari
 *              Reto Da Forno
 * 
 * @brief configure TA0 and TA1 to be used to schedule tasks with high accuracy
 */

#ifndef __RTIMER_H__
#define __RTIMER_H__

/**
 * @brief the number of usable rtimers of module TA0
 * @note if the radio module is used, one CCR is reserved for it and cannot be
 * used otherwise
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
#define RTIMER_SECOND           ((rtimer_clock_t)SMCLK_SPEED)  /* for TA0 */
#define RTIMER_SECOND_TA1       (ACLK_SPEED)                   /* for TA1 */

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
#define RTIMER_IS_UPDATE_ENABLED    ((TA0CTL & TAIE) > 0)
#define RTIMER_TA1_IS_UPDATE_EN     ((TA1CTL & TAIE) > 0)

/**
 * @brief the rtimer IDs for timer module TA0 (number of IDs corresponds to the
 * number of CCRs)
 * 
 * TA0_x are high-speed timers (3.25 MHz)
 * TA1_x timers run at 32768 Hz
 */
typedef enum {
  RTIMER_TA0_0 = 0,      /* TA0 CCR0 */
  RTIMER_TA0_1,
  RTIMER_TA0_2,
  RTIMER_TA0_3,
  RTIMER_TA0_4,
  RTIMER_TA1_0,          /* TA1 CCR0 */
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
  RTIMER_JUST_EXPIRED = 2,
  RTIMER_WFE = 3,      /* wait for event */
} rtimer_state_t;

typedef uint64_t rtimer_clock_t;
typedef struct rtimer_t rtimer_t;
 /* prototype of a rtimer callback function */
typedef char (*rtimer_callback_t)(rtimer_t *rt);

/**
 * @brief state and parameters of an rtimer
 */
typedef struct rtimer_t {
  rtimer_callback_t func; /* callback function to execute when the rtimer
                             expires */
  rtimer_state_t state;   /* internal state of the rtimer */
  rtimer_clock_t period;  /* if period = 0: one-shot timer; otherwise: timer
                             period in clock ticks */
  rtimer_clock_t time;    /* if state = RTIMER_SCHEDULED: next expiration time;
                             otherwise: last expiration time */
} rtimer_t;

/**
 * @brief initialize the timers TA0 and TA1
 */
void rtimer_init(void);

/**
 * @brief schedule (start) an rtimer
 * @param[in] timer the ID of the rtimer, must by of type rtimer_id_t
 * @param[in] start the expiration time (absolute counter value)
 * @param[in] period the period; set this to zero if you don't want this timer
 * trigger an interrupt periodically
 * @param[in] func the callback function; will be executed as soon as the timer
 * has expired
 */
void rtimer_schedule(rtimer_id_t timer,
                     rtimer_clock_t start,
                     rtimer_clock_t period,
                     rtimer_callback_t func);

/**
 * @brief set a CCR of a timer to event mode (capture input)
 */
void rtimer_wait_for_event(rtimer_id_t timer, rtimer_callback_t func);

/**
 * @brief stop an rtimer
 * @param[in] timer the ID of the rtimer, must by of type rtimer_id_t
 */
void rtimer_stop(rtimer_id_t timer);

/**
 * @brief get the period of the specified rtimer
 * @param[in] timer the ID of the rtimer, must by of type rtimer_id_t
 */
rtimer_clock_t rtimer_get_period(rtimer_id_t timer);

/**
 * @brief get the expiration time of the specified rtimer
 * @param[in] timer the ID of the rtimer, must by of type rtimer_id_t
 * @return expiration time in timer clock ticks (timestamp)
 */
rtimer_clock_t rtimer_get_expiration_time(rtimer_id_t timer);

/**
 * @brief get the current timer value of TA0
 * @return timer value in timer clock ticks (timestamp)
 */
rtimer_clock_t rtimer_now(void);

/**
 * @brief get the current timer value of TA1
 * @return timer value in timer clock ticks (timestamp)
 */
rtimer_clock_t rtimer_now_ta1(void);

/* convert rtimer time values from clock ticks to milliseconds */
#define RTIMER_TO_MS(t) ((t) / (RTIMER_SECOND / 1000))

/* get the current rtimer time (macro used in several Contiki core files) */
#define RTIMER_NOW    (rtimer_now())

/* get the current rtimer time in milliseconds */
#define RTIMER_NOW_MS() (RTIMER_TO_MS(rtimer_now()))

#define NS_TO_RTIMER_TICKS(ns) (((rtimer_clock_t)(ns) * \
                                 (rtimer_clock_t)RTIMER_SECOND) / \
                                (rtimer_clock_t)1000000000)

extern volatile rtimer_clock_t ta0_sw_ext;

#endif /* __RTIMER_H__ */

/**
 * @}
 * @}
 */
