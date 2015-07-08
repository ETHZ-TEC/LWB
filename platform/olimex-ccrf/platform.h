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
 * @defgroup    platform Platform
 * @{
 *
 * @file
 * @author
 *              Reto Da Forno
 *
 * @brief platform includes and definitions
 */

#ifndef __PLATFORM_H__
#define __PLATFORM_H__

/*
 * include application specific config
 */
#include "config.h"     /* application specific configuration */

/*
 * configuration and definitions
 */
#define FRAM_CONF_ON        1        
#define BOLT_CONF_ON        0         /* this platform does not support bolt */

#define MCU_TYPE            "CC430F5137"
#define COMPILE_DATE        __DATE__
#define GCC_VS              __VERSION__  
                            /* __GNUC__ __GNUC_MINOR__ __GNUC_PATCHLEVEL__ */
#define SRAM_SIZE           2048

/*
 * pin mapping
 */
/* #define GLOSSY_RX_PIN       PORT3, PIN5      */
/* #define GLOSSY_TX_PIN       PORT3, PIN4      */
/* #define LWB_TASK_ACT_PIN    PORT3, PIN6      */
#define LED_RED             PORT1, PIN0   /* Note: this board has only 1 LED */
#define LED_0               LED_RED 
#define LED_STATUS          LED_0
#define DEBUG_TASK_ACT_PIN  PORT2, PIN6
#define FLOCKLAB_LED1       PORT1, PIN0
#define FLOCKLAB_LED2       PORT1, PIN1
#define FLOCKLAB_LED3       
#define FLOCKLAB_INT1       PORT3, PIN6
#define FLOCKLAB_INT2       PORT3, PIN7

/*
 * include standard libraries
 */
#include <stdio.h>
/*#include <stdlib.h>*/
#include <isr_compat.h>
#include <string.h>

/*
 * include MCU HAL
 */
#include <msp430.h>

/*
 * include MCU specific drivers
 */
#include "adc.h"
#include "clock.h"
#include "dma.h"
#include "flash.h"
#include "glossy.h"
#include "gpio.h"
#include "hal-pmm.h"
#include "leds.h"
#include "rf1a.h"
#include "rf1a-SmartRF-settings/868MHz-2GFSK-250kbps.h" /* RF1A config */
#include "rtimer.h"
#include "spi.h"
#include "uart.h"
#include "usci.h"
#include "watchdog.h"

/*
 * include plugins (PCB specific)
 */
#include "bolt.h"
#include "debug-print.h"
#include "fram.h"    /* provides the interface towards the external memory */
#include "membx.h"

#endif /* __PLATFORM_H__ */

/**
 * @}
 * @}
 */
