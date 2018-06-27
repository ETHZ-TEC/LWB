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
 * Author:  Reto Da Forno
 */

/**
 * @addtogroup  Platform
 * @{
 *
 * @defgroup    watchdog Watchdog
 * @{
 *
 * @file
 *
 * @brief configure the watchdog timer
 */

#ifndef __WATCHDOG_H__
#define __WATCHDOG_H__


/* reset the watchdog counter during a TA1 timer overflow/update */
#ifndef WATCHDOG_CONF_RESET_ON_TA1IFG
#define WATCHDOG_CONF_RESET_ON_TA1IFG     1
#endif /* WATCHDOG_CONF_RESET_ON_TA1IFG */

#ifndef WATCHDOG_CONF_TIMER_MODE
#define WATCHDOG_CONF_TIMER_MODE          0
#endif /* WATCHDOG_CONF_TIMER_MODE */

#ifndef WATCHDOG_CONF_STOP_IN_LPM
 #if WATCHDOG_CONF_RESET_ON_TA1IFG
 #define WATCHDOG_CONF_STOP_IN_LPM        0
 #else /* WATCHDOG_CONF_RESET_ON_TA1IFG */
 #define WATCHDOG_CONF_STOP_IN_LPM        1
 #endif /* WATCHDOG_CONF_RESET_ON_TA1IFG */
#endif /* WATCHDOG_CONF_STOP_IN_LPM */


/**
 * @brief configure the watchdog in interval timer mode (interrupt enabled!)
 */
static inline void
watchdog_interrupt_enable(void)
{
  WDTCTL  = WDTCTL_L + WDTPW + WDTTMSEL; /* interval timer mode */
  SFRIE1 |= WDTIE;
}

/**
 * @brief sets the clock source (ACLK) and divider
 * @note divider: WDTIS_3 = 512k, WDTIS_4 = 32k
 */
static inline void
watchdog_init(void)
{
  WDTCTL = WDTPW + WDTCNTCL + WDTHOLD + WDTSSEL_1 + WDTIS_3;
#if WATCHDOG_CONF_TIMER_MODE
  watchdog_interrupt_enable();
#endif /* WATCHDOG_CONF_TIMER_MODE */
}

/**
 * @brief hold the watchdog timer
 */
static inline void
watchdog_stop(void)
{
  WDTCTL = WDTPW + WDTHOLD;
}

/**
 * @brief start the watchdog timer
 */
static inline void
watchdog_start(void)
{
  WDTCTL = (WDTCTL_L & ~(WDTHOLD)) + WDTPW;
}

/**
 * @brief reset the watchdog timer count to 0
 */
static inline void
watchdog_reset(void)
{
  /* set counter clear bit */
  WDTCTL = (WDTCTL_L | WDTCNTCL) + WDTPW;
}

/**
 * @brief same as watchdog_reset()
 */
static inline void
watchdog_periodic(void)
{
  /* set counter clear bit */
  WDTCTL = (WDTCTL_L | WDTCNTCL) + WDTPW;
}


#endif /* __WATCHDOG_H__ */

/**
 * @}
 * @}
 */
