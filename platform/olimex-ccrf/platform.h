#ifndef __PLATFORM_H__
#define __PLATFORM_H__

 
/*
 * include config
 */

#include "config.h"     // application specific configuration


/*
 * platform specific HW configuration
 */

#define UART0_BAUDRATE      115200LU            // do not change this without modifying uart0.c
#define ASYNC_INT_SPEED     (SMCLK_SPEED / 1)   // desired SPI bit clock speed for the asynchronous interface (note: must not be higher than 4 MHz)
#define FRAM_SPEED          (SMCLK_SPEED / 1)   // desired SPI bit clock speed for the external FRAM, mustn't be higher than 13 MHz
//#define GLOSSY_RX_PIN       PORT3, PIN5       // high when glossy is receiving
//#define GLOSSY_TX_PIN       PORT3, PIN4       // high when glossy is transmitting
//#define LWB_TASK_ACT_PIN    PORT3, PIN6       // show lwb task activity on this pin (comment this line to disable it)
#define LED_STATUS          LED_0

#define HAS_FRAM
#define SPI_ASYNC_INT       SPI_A0_BASE     // base address of the SPI module used to communicate with the asynchronous interface
#define SPI_FRAM            SPI_B0_BASE     // base address of the SPI module used to communicate with the external memory (FRAM)
#define LED_RED             PORT1, PIN0     // Note: this board has only 1 LED
#define LED_0               LED_RED
#define FRAM_CTRL           PORT1, PIN7     // control line for the external FRAM (SPI chip select / enable line)
//#define RF_GDO2_PIN         1, 1
#define DEBUG_PRINT_TASK_ACT_PIN  PORT2, PIN6
// flocklab pins
#define FLOCKLAB_LED1       PORT1, PIN0
#define FLOCKLAB_LED2       PORT1, PIN1
#define FLOCKLAB_LED3       PORT1, PIN2     // -> Note: this pin is already used by the SPI
#define FLOCKLAB_INT1       PORT3, PIN6
#define FLOCKLAB_INT2       PORT3, PIN7

/*
 * Note on the SPI phase and polarity:
 * For CPOL = 0 and ...
 * ... CPHA = 0, data is captured on the clock's rising edge and changed (latched) on the falling edge.
 * ... CPHA = 1, data is captured on the clock's falling edge and changed (latched) on the rising edge.
 * For CPOL = 1 and ...
 * ... CPHA = 0, data is captured on the clock's falling edge and changed (latched) on the rising edge.
 * ... CPHA = 1, data is captured on the clock's rising edge and changed (latched) on the falling edge.
 */
#define SPI_A0_CPOL         0
#define SPI_A0_CPHA         0
#define SPI_B0_CPOL         0                   // clock polarity (default: 0 = inactive low, 1 = inactive high)
#define SPI_B0_CPHA         0                   // clock phase (see note below)

 
/*
 * include standard libraries
 */
#include <stdio.h>
#include <stdlib.h>
#include <isr_compat.h>
#include <string.h>

/*
 * include MCU HAL
 */
#include <msp430.h>
#include "hal.h"

/*
 * include MCU specific files
 */
#include "rf1a_SmartRF_settings/868MHz_2GFSK_250kbps.h" /* RF1A configuration file */
#include "leds.h"
#include "gpio.h"
#include "adc.h"
#include "clock.h"
#include "rtimer.h"
#include "hal_pmm.h"
#include "spi.h"
#include "rf1a.h"
/*#include "nullmac.h"*/
#include "glossy.h"
#include "dma.h"
#include "flash.h"
#include "watchdog.h"
#include "uart0.h"
#include "debug-print.h"

/*
 * include plugins (PCB specific)
 */
#include "membx.h"
#include "fram.h"               /* provides an interface towards the external memory */
#include "bolt.h"



#endif /* __PLATFORM_H__ */