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
#include <stdint.h>
#include <isr_compat.h>

/*
 * include MCU definitions
 */
#include <cc430f5137.h>             /* or simply include <msp430.h> */

#define MCU_TYPE                    "CC430F5137"
#define COMPILER_INFO               "GCC " __VERSION__
#define GCC_VS                      __GNUC__ __GNUC_MINOR__ __GNUC_PATCHLEVEL__
#define COMPILE_DATE                __DATE__
#define SRAM_START                  0x1c00
#define SRAM_END                    0x2bff        /* last valid byte in SRAM */
#define SRAM_SIZE                   4096            /* starting at 0x1C00 */

/*
 * include application specific config
 */
#include "config.h"                 /* application specific configuration */

/*
 * default configuration (values may be overwritten in config.h)
 */
#ifndef WATCHDOG_CONF_ON
#define WATCHDOG_CONF_ON            0
#endif /* WATCHDOG_CONF_ON */

#ifndef LED_CONF_ON 
#define LED_CONF_ON                 1
#endif /* LED_CONF_ON */

#define CLOCK_CONF_XT1_ON           1
#define CLOCK_CONF_XT1_CAP          XCAP_3

/* specify the number of timer modules */
#if RF_CONF_ON
#define RTIMER_CONF_NUM_HF          4  /* number of high-frequency timers */
#else
#define RTIMER_CONF_NUM_HF          5
#endif /* RF_CONF_ON */
#define RTIMER_CONF_NUM_LF          3  /* number of low-frequency timers */     

/* specify the number of SPI modules */
#define SPI_CONF_NUM_MODULES        2


/*
 * ERROR checks (verify parameters)
 */
#if !LWB_CONF_USE_XMEM && LWB_CONF_STATS_NVMEM
/* logging to non volatile memory is only available if the external memory is
 * used! */
#error "must enable LWB_CONF_USE_XMEM in order to use LWB_CONF_STATS_NVMEM"
#endif
#if LWB_CONF_USE_XMEM && !FRAM_CONF_ON
#error "LWB_CONF_USE_XMEM is not available if FRAM is disabled"
#endif 


/*
 * pin mapping
 */
#define LED_RED                     PORT1, PIN0
#define LED_0                       LED_RED 
#define LED_STATUS                  LED_RED
#define LED_ERROR                   LED_RED
#ifndef FLOCKLAB
#define DEBUG_SWITCH                PORT1, PIN1  /* user push-button */
#endif /* FLOCKLAB */

#if FRAM_CONF_ON
  #define FRAM_CONF_CTRL_PIN        PORT1, PIN7
  #define FRAM_CONF_SIZE            125000      /* 1 Mbit */
  #define FRAM_CONF_SPI             SPI_1
  #ifndef DEBUG_PRINT_CONF_USE_XMEM
  #define DEBUG_PRINT_CONF_USE_XMEM 1
  #ifndef DEBUG_PRINT_CONF_NUM_MSG
  #define DEBUG_PRINT_CONF_NUM_MSG  20
  #endif /* DEBUG_PRINT_CONF_NUM_MSG */
  #endif /* DEBUG_PRINT_CONF_USE_XMEM */
 #ifdef FLOCKLAB
  /* remap the SPI pins! */
  #define SPI_CONF_B0_SIMO          PORT2, PIN0         /* instead of P1.3 */ 
  #define SPI_CONF_B0_CLK           PORT2, PIN1         /* instead of P1.4 */ 
 #endif /* FLOCKLAB */
#endif /* FRAM_CONF_ON */

#if BOLT_CONF_ON
  #define BOLT_CONF_SPI             SPI_1
  #define BOLT_CONF_IND_PIN         PORT2, PIN0
  #define BOLT_CONF_MODE_PIN        PORT2, PIN1
  #define BOLT_CONF_REQ_PIN         PORT2, PIN2
  #define BOLT_CONF_ACK_PIN         PORT2, PIN3
  /* IND pin for the outgoing queue (sent messages) */
  #define BOLT_CONF_IND_OUT_PIN     PORT2, PIN4
  #define BOLT_CONF_TIMEREQ_PIN     PORT3, PIN3
  #define BOLT_CONF_FUTUREUSE_PIN   PORT2, PIN5
#endif /* BOLT_CONF_ON */

/* the following pins assignments are given by FlockLAB, do not change */
#define FLOCKLAB_LED1               PORT1, PIN0  /* for GPIO tracing */
#define FLOCKLAB_LED2               PORT1, PIN1  /* for GPIO tracing */
#define FLOCKLAB_LED3               PORT1, PIN2  /* for GPIO tracing */
#define FLOCKLAB_SIG1               PORT1, PIN3  /* target actuation */
#define FLOCKLAB_SIG2               PORT1, PIN4  /* target actuation */
#define FLOCKLAB_INT1               PORT3, PIN6  /* for GPIO tracing */
#define FLOCKLAB_INT2               PORT3, PIN7  /* for GPIO tracing */
/* Important note: P1.3 and P1.4 are used as target actuation pin on Flocklab 
 * and can therefore not be used for the SPI B0 interface! */

/*#define GLOSSY_RX_PIN             PORT2, PIN3*/
/*#define GLOSSY_TX_PIN             PORT2, PIN4*/
/*#define RF_GDO0_PIN               PORT1, PIN2 */
/*#define RF_GDO1_PIN               PORT1, PIN3 */
/* note: rf1a_init sets the GDO2 signal as follows: Asserts when sync word has
 * been sent or received, and deasserts at the end of the packet. In RX, the 
 * pin deassert when the optional address check fails or the RX FIFO 
 * overflows. In TX the pin deasserts if the TX FIFO underflows. */
//#define MCLK_PIN                  PORT2, PIN6
/*#define ACLK_PIN                  PORT3, PIN3*/
/*#define SMCLK_PIN                 PORT3, PIN1*/

#define MCU_HAS_ADC12

/* The application should define the following two macros for better
 * performance (otherwise glossy will disable all active interrupts). */
#define GLOSSY_DISABLE_INTERRUPTS
#define GLOSSY_ENABLE_INTERRUPTS

#define UART_ACTIVE                 (UCA0STAT & UCBUSY)

#if LWB_CONF_USE_LF_FOR_WAKEUP
  #define LWB_AFTER_DEEPSLEEP()     {\
                                    if(UCSCTL6 & XT2OFF) {\
                                      SFRIE1  &= ~OFIE;\
                                      ENABLE_XT2();\
                                      WAIT_FOR_OSC();\
                                      UCSCTL4  = SELA | SELS | SELM;\
	                              __delay_cycles(100); /* errata PMM12? */\
	                              UCSCTL5  = DIVA | DIVS | DIVM; \
	                              /*__delay_cycles(100);*/ /* errata PMM12? */\
	                              UCSCTL7  = 0; /* errata UCS11 */ \
                                      SFRIE1  |= OFIE;\
                                      TA0R    = 0;\
                                      TA0CTL  |= MC_2;\
                                      P1SEL   |= (BIT2 | BIT3 | BIT4 | BIT5 | \
                                                  BIT6);\
                                      P1DIR   &= ~(BIT2 | BIT5);\
                                    }}
#endif /* LWB_CONF_USE_LF_FOR_WAKEUP */


/*
 * include MCU specific files
 */
#include "rf1a-SmartRF-settings/868MHz-2GFSK-250kbps.h" /* RF1A config */
#include "adc.h"
#include "clock.h"
#include "dma.h"
#include "flash.h"
#include "gpio.h"
#include "pmm.h"
#include "rf1a.h"        /* RF1A config must be included BEFORE rf1a.h */
#include "rtimer.h"
#include "spi.h"
#include "uart.h"
#include "usci.h"
#include "watchdog.h"

#endif /* __PLATFORM_H__ */

