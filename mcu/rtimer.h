/*
 * Copyright (c) 2016, Swiss Federal Institute of Technology (ETH Zurich).
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
 */

/**
 * @addtogroup  Platform
 * @{
 *
 * @defgroup    rtimer Realtime timer
 * @{
 *
 * @file
 * 
 * @brief interface definition for high-speed/frequency (HF) and 
 * low-speed/frequency (LF) timers
 * 
 * supports up to 8 HF and 8 LF timers
 */

#ifndef __RTIMER_H__
#define __RTIMER_H__

#include "contiki-conf.h"     /* required for the rtimer_clock_t declaration */

/* override these default values in platform.h or config.h */

#ifndef RTIMER_CONF_NUM_HF       /* number of (usable) high-frequency timers */
#error "RTIMER_CONF_NUM_HF not defined!"
#endif /* RTIMER_CONF_NUM_HF */

#ifndef RTIMER_CONF_NUM_LF        /* number of (usable) low-frequency timers */
#error "RTIMER_CONF_NUM_LF not defined!"
#endif /* RTIMER_CONF_NUM_LF */

#ifndef RTIMER_CONF_HF_CLKSPEED
#define RTIMER_CONF_HF_CLKSPEED     SMCLK_SPEED
#endif /* RTIMER_CONF_HF_CLKSRC */

#ifndef RTIMER_CONF_LF_CLKSPEED
#define RTIMER_CONF_LF_CLKSPEED     ACLK_SPEED
#endif /* RTIMER_CONF_HF_CLKSRC */


/**
 * @brief the number of timer ticks that (approx.) correspond to 1s
 */
#define RTIMER_SECOND_HF            ((rtimer_clock_t)RTIMER_CONF_HF_CLKSPEED)
#define RTIMER_SECOND_LF            (RTIMER_CONF_LF_CLKSPEED) 
#define RTIMER_HF_LF_RATIO          (RTIMER_SECOND_HF / RTIMER_SECOND_LF)

/**
 * @brief convert rtimer values from clock ticks to milliseconds and
 * nanoseconds to rtimer ticks
 */
#define RTIMER_HF_TO_MS(t)          ((t) / (RTIMER_SECOND_HF / 1000))
#define RTIMER_LF_TO_MS(t)          ((t * 1000) / (RTIMER_SECOND_LF))
#define NS_TO_RTIMER_HF(ns)         (((rtimer_clock_t)(ns) * \
                                      (rtimer_clock_t)RTIMER_SECOND_HF) / \
                                     (rtimer_clock_t)1000000000)

/**
 * @brief the rtimer IDs
 */
typedef enum {
  RTIMER_HF_0 = 0,
  RTIMER_HF_1,
#if RTIMER_CONF_NUM_HF > 2
  RTIMER_HF_2,
#endif
#if RTIMER_CONF_NUM_HF > 3
  RTIMER_HF_3,
#endif
#if RTIMER_CONF_NUM_HF > 4
  RTIMER_HF_4,
#endif
#if RTIMER_CONF_NUM_HF > 5
  RTIMER_HF_5,
#endif 
#if RTIMER_CONF_NUM_HF > 6
  RTIMER_HF_6,
#endif 
#if RTIMER_CONF_NUM_HF > 7
  RTIMER_HF_7,
#endif 
  RTIMER_LF_0,
  RTIMER_LF_1,
#if RTIMER_CONF_NUM_LF > 2
  RTIMER_LF_2,
#endif
#if RTIMER_CONF_NUM_LF > 3
  RTIMER_LF_3,
#endif
#if RTIMER_CONF_NUM_LF > 4
  RTIMER_LF_4,
#endif
#if RTIMER_CONF_NUM_LF > 5
  RTIMER_LF_5,
#endif
#if RTIMER_CONF_NUM_LF > 6
  RTIMER_LF_6,
#endif
#if RTIMER_CONF_NUM_LF > 7
  RTIMER_LF_7,
#endif
  NUM_OF_RTIMERS
} rtimer_id_t;

/* this is necessary for the rtimer_callback_t prototype declaration */
typedef struct rtimer rtimer_t;

/* prototype of a rtimer callback function */
typedef char (*rtimer_callback_t)(rtimer_t *rt);

typedef enum {
  RTIMER_INACTIVE = 0,
  RTIMER_SCHEDULED = 1,
  RTIMER_JUST_EXPIRED = 2,
  RTIMER_WFE = 3,      /* wait for event */
} rtimer_state_t;

/**
 * @brief state and parameters of an rtimer
 */
typedef struct rtimer {
  rtimer_clock_t time;    /* if state = RTIMER_SCHEDULED: next expiration time;
                             otherwise: last expiration time */
  rtimer_clock_t period;  /* if period = 0: one-shot timer; otherwise: timer
                             period in clock ticks */
  rtimer_callback_t func; /* callback function to execute when the rtimer
                             expires */
  rtimer_state_t state;   /* internal state of the rtimer */
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
 * @brief resets the rtimer values (both LF and HF) to zero
 */
void rtimer_reset(void);

/**
 * @brief enable or disable the overflow/update interrupts for both
 * the LF and HF timer
 */
inline void rtimer_update_enable(uint8_t enable);

/**
 * @brief check whether the overflow/update interrupt (of the HF timer) is 
 * enabled
 */
inline uint8_t rtimer_update_enabled();

/**
 * @brief get the current timer value of TA0 (high frequency)
 * @return timer value in timer clock ticks (timestamp)
 */
rtimer_clock_t rtimer_now_hf(void);

/**
 * @brief get the current timer value of TA1 (low frequency)
 * @return timer value in timer clock ticks (timestamp)
 */
rtimer_clock_t rtimer_now_lf(void);

/**
 * @brief get the current timer value of both, the high and low frequency
 * timer
 * @param[in] hf_val value in timer clock ticks (timestamp)
 * @param[in] lf_val value in timer clock ticks (timestamp)
 * @remark This function will only work properly if the CPU is running on the
 * same clock source as the timer TA0 (HF) and this function can be executed 
 * within one TA0 period (i.e. ~20ms @ 3.25 MHz).
 */
void rtimer_now(rtimer_clock_t* const hf_val, rtimer_clock_t* const lf_val);

/**
 * @brief get the address of the software extensions of the timer module
 * @param[in] timer the ID of an rtimer 
 * @return the address of the software extension
 */
uint16_t rtimer_get_swext_addr(rtimer_id_t timer);


#endif /* __RTIMER_H__ */

/**
 * @}
 * @}
 */
