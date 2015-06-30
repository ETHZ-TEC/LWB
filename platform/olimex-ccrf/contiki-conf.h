#ifndef __CONTIKI_CONF_H__
#define __CONTIKI_CONF_H__

/*
 * contiki configuration
 */
 
// standard libraries
#include <stdio.h>
#include <stdlib.h>
#include <isr_compat.h>
#include <string.h>

#include "config.h"     // application specific configuration

#define CLIF
#define CCIF


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


extern volatile uint16_t node_id;


#endif /* __CONTIKI_CONF_H__ */
