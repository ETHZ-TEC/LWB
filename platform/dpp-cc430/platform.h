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
 * @file
 *
 * @brief platform includes and definitions
 * 
 * @note generally, if one of the platform files is needed platform.h
 * should be included instead of the specific file to preserve the 
 * include order and prevent compiler warnings
 */

#ifndef __PLATFORM_H__
#define __PLATFORM_H__

/*
 * include standard libraries
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <isr_compat.h>

/*
 * include MCU definitions
 */
#include <cc430f5147.h>             /* or simply include <msp430.h> */

#define MCU_TYPE                    "CC430F5147"
#define COMPILER_INFO               "GCC " __VERSION__
#define GCC_VS                      __GNUC__ __GNUC_MINOR__ __GNUC_PATCHLEVEL__
#define COMPILE_DATE                __DATE__
#define SRAM_SIZE                   4096            /* starting at 0x1C00 */

/*
 * include application specific config
 */
#include "config.h"                 /* application specific configuration */

/*
 * configuration and definitions (default values, may be overwritten
 * in config.h)
 */
#ifndef WATCHDOG_CONF_ON
#define WATCHDOG_CONF_ON            0
#endif /* WATCHDOG_CONF_ON */

#ifndef LEDS_CONF_ON 
#define LEDS_CONF_ON                1
#endif /* LEDS_CONF_ON */

#ifndef FRAM_CONF_ON
#define FRAM_CONF_ON                1
#endif /* FRAM_CONF_ON */

#define CLOCK_CONF_XT1_ON           1           

/* specify the number of timer modules */
#if RF_CONF_ON
#define RTIMER_CONF_NUM_HF          4  /* number of high-frequency timers */
#else
#define RTIMER_CONF_NUM_HF          5
#endif /* RF_CONF_ON */
#define RTIMER_CONF_NUM_LF          3  /* number of low-frequency timers */     

/* specify the number of SPI modules */
#define SPI_CONF_NUM_MODULES        2

/* disable the SVS */
#define SVS_DISABLE                 { PMMCTL0_H = 0xA5;\
                                      SVSMHCTL  = 0;\
                                      SVSMLCTL  = 0;\
                                      PMMCTL0_H = 0x00; }


/*
 * pin mapping
 */
#define LED_0                       PORT3, PIN0
#define LED_STATUS                  LED_0
#define LED_ERROR                   LED_0
#define COM_MCU_INT1                PORT3, PIN5
#define COM_MCU_INT2                PORT3, PIN4    
#define FRAM_CONF_CTRL_PIN          PORT2, PIN0
#define FRAM_CONF_SPI               SPI_0

/* select multiplexer channel (high = UART, low = SPI) */
#define MUX_SEL_PIN                 PORT2, PIN7

//#define DEBUG_PRINT_TASK_ACT_PIN    PORT2, PIN0
#define GLOSSY_START_PIN            LED_0  
//#define GLOSSY_RX_PIN               COM_MCU_INT1
//#define GLOSSY_TX_PIN               COM_MCU_INT2
//#define RF_GDO0_PIN                 COM_MCU_INT1
//#define RF_GDO1_PIN                 COM_MCU_INT1
//#define RF_GDO2_PIN                 COM_MCU_INT1
//#define MCLK_PIN                    COM_MCU_INT1
//#define ACLK_PIN                    COM_MCU_INT1
//#define SMCLK_PIN                   COM_MCU_INT1

/* the following pins assignments are given by FlockLAB, do not change */
#define FLOCKLAB_INT1               COM_MCU_INT1  /* for GPIO tracing */
#define FLOCKLAB_INT2               COM_MCU_INT2  /* for GPIO tracing */

/* specify what needs to be done every time before UART is enabled */
#define UART_BEFORE_ENABLE {\
  PIN_SET(MUX_SEL_PIN);\
  uart_reinit();\
}

/*
 * The application should define the following two macros for better
 * performance (otherwise glossy will disable all active interrupts).
 */
#define GLOSSY_DISABLE_INTERRUPTS
#define GLOSSY_ENABLE_INTERRUPTS


/*#define LWB_BEFORE_DEEPSLEEP()    {\
                                    \
                                    TA0CTL  &= ~MC_3;\
                                    UCSCTL4  = SELA__XT1CLK | SELS__XT1CLK | \
                                               SELM__XT1CLK;\
                                    DISABLE_XT2();\
                                    UCSCTL7  = 0;\
                                    P1SEL    = 0;\
                                    P1DIR   |= (BIT2 | BIT5);\
                                    UCA0CTL1 |= UCSWRST;\
                                    }
*/
                                    /* make sure HF crystal is enabled */
#define LWB_AFTER_DEEPSLEEP()       {\
                                    if(UCSCTL6 & XT2OFF) {\
                                      SFRIE1  &= ~OFIE;\
                                      ENABLE_XT2();\
                                      WAIT_FOR_OSC();\
                                      UCSCTL4  = SELA | SELS | SELM;\
                                      UCSCTL7  = 0;\
                                      SFRIE1  |= OFIE;\
                                      TA0CTL  |= MC_2;\
                                      P1SEL   |= (BIT2 | BIT3 | BIT4 | BIT5 | \
                                                  BIT6);\
                                      P1DIR   &= ~(BIT2 | BIT5);\
                                    }\
                                    }
                                    /*UCSCTL4  = SELA | SELS | SELM;*/

/* specify what needs to be done every time before SPI is enabled */
#define SPI_BEFORE_ENABLE(spi) {\
  PIN_CLR(MUX_SEL_PIN); \
  if(spi == SPI_0) { spi_reinit(SPI_0); }\
}

/*
 * include MCU specific drivers
 */
#include "rf1a-SmartRF-settings/868MHz-2GFSK-250kbps.h" /* RF1A config */
#include "clock.h"
#include "dma.h"
#include "flash.h"
#include "gpio.h"
#include "pmm.h"
#include "leds.h"
#include "rf1a.h"        /* RF1A config must be included BEFORE rf1a.h */
#include "rtimer.h"
#include "spi.h"
#include "uart.h"
#include "usci.h"
#include "watchdog.h"

#endif /* __PLATFORM_H__ */

