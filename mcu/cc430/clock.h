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
 *          Federico Ferrari
 */

/**
 * @addtogroup  Platform
 * @{
 *
 * @defgroup    clock Clock
 * @{
 *
 * @file
 *
 * @brief
 */

#ifndef __CLOCK_H__
#define __CLOCK_H__

/* FLL is only required if one of the following clock sources is used: DCOCLK,
   DCOCLKDIV, FLLREFCLK */
#ifndef CLOCK_CONF_FLL_ON
#define CLOCK_CONF_FLL_ON    0
#endif /* CLOCK_CONF_FLL_ON */

#ifndef CLOCK_CONF_XT1_ON
#define CLOCK_CONF_XT1_ON    1
#endif /* CLOCK_CONF_XT1_ON */

/* speed of XT1 (low-frequency crystal) */
#define XT1CLK_SPEED    32768

/* speed of XT2 (high-frequency crystal) */
#define XT2CLK_SPEED    26000000LU

/* source and speed of the Master Clock MCLK */
#define SELM            SELM__XT2CLK
#define DIVM            DIVM__2
#define MCLK_SPEED      (XT2CLK_SPEED / 2)      /* 13 MHz */

/* source and speed of the Auxiliary Clock ACLK */
#if CLOCK_CONF_XT1_ON
#define SELA            SELA__XT1CLK            /* 32768 Hz */
#else /* CLOCK_CONF_XT1_ON */
#define SELA            SELA__REFOCLK           /* ~32.8kHz */
#endif /* CLOCK_CONF_XT1_ON */
#define DIVA            DIVA__1
#define ACLK_SPEED      (XT1CLK_SPEED / 1)      

/* source and speed of the Sub-System Master Clock SMCLK */
#define SELS            SELS__XT2CLK
#define DIVS            DIVS__8
#define SMCLK_SPEED     (XT2CLK_SPEED / 8)      /* 3.25 MHz */

/* check whether the high-frequency crystal XT2 is permanently enabled */
#define IS_XT2_ENABLED() (!(UCSCTL6 & XT2OFF))

/* permanently enable the high-frequency crystal XT2 (i.e., even if the radio 
 * is in SLEEP mode) */
#define ENABLE_XT2()    (UCSCTL6 &= ~XT2OFF)
#define ENABLE_XT1()    (UCSCTL6 &= ~XT1OFF)

/* disable the high-frequency crystal XT2 (i.e., active only when the radio is
   active) */
#define DISABLE_XT2()   (UCSCTL6 |= XT2OFF)
#define DISABLE_XT1()   (UCSCTL6 |= XT1OFF)

/* disable automatic clock requests for ACLK */
#define DISABLE_ACLK()  { UCSCTL8 &= ~ACLKREQEN; }

/* disable automatic clock requests for ACLK */
#define DISABLE_SMCLK() { UCSCTL8 &= ~SMCLKREQEN; }

/* enable the FLL control loop */
#define ENABLE_FLL()    (__bic_status_register(SCG0))

/* disable the FLL control loop */
#define DISABLE_FLL()   (__bis_status_register(SCG0))

/* corresponds to roughly 1.008246 seconds (HF timer overflows ~50x/sec.) */
#define CLOCK_SECOND    50

/**
 * @brief wait for the oscillator fault flag to clear
 */
#define WAIT_FOR_OSC()  do \
{ \
    UCSCTL7 &= ~(XT1LFOFFG + DCOFFG + XT2OFFG); \
    SFRIFG1 &= ~OFIFG; \
} while (SFRIFG1 & OFIFG)

/**
 * @brief busy wait for ms milliseconds (delay loop) 
 */
#define WAIT_MS(ms)     __delay_cycles(MCLK_SPEED / 1000 * ms)
#define DELAY(ms)       __delay_cycles(MCLK_SPEED / 1000 * ms)


/**
 * @brief initialize the clock system 
 */
void clock_init(void);


#endif /* __CLOCK_H__ */

/**
 * @}
 * @}
 */