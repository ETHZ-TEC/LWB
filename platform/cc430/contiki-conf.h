#ifndef __CONTIKI_CONF_H__
#define __CONTIKI_CONF_H__

/*
 * contiki configuration
 */
 
// standard libraries
#include <stdio.h>
#include <stdlib.h>
#include <msp430.h>
#include <isr_compat.h>
#include <string.h>

#include "config.h"     // application specific configuration
#include "hal.h"        // hardware definitions

#define CLIF
#define CCIF



typedef uint32_t clock_time_t;
#define CLOCK_SECOND    50      // corresponds to roughly 1.008246 seconds

// RF1A configuration file
#include "rf1a_SmartRF_settings/868MHz_2GFSK_250kbps.h"

typedef uint16_t addr_t;

#ifndef ENERGEST_CONF_ON
#define ENERGEST_CONF_ON 0
#endif /* ENERGEST_CONF_ON */

#ifndef WATCHDOG_CONF_ON
#define WATCHDOG_CONF_ON 0
#endif /* WATCHDOG_CONF_ON */

// globally enable debug prints
#ifndef DEBUG_PRINT_CONF_ON
#define DEBUG_PRINT_CONF_ON 1
#endif /* DEBUG_PRINT_CONF_ON */


#endif /* __CONTIKI_CONF_H__ */
