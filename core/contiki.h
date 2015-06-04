#ifndef __CONTIKI_H__
#define __CONTIKI_H__

#ifndef CONTIKI_VERSION_STRING
#define CONTIKI_VERSION_STRING "Contiki 2.7"
#endif /* CONTIKI_VERSION_STRING */

// the configuration
#include "contiki-conf.h"

// contiki libraries
#include "sys/process.h"
#include "sys/autostart.h"
#include "sys/pt.h"
#include "sys/etimer.h"
#include "sys/energest.h"

#include "lib/ringbuf.h"
#include "lib/random.h"
#include "lib/memb.h"
#include "lib/list.h"

#include "dev/serial-line.h"
#include "dev/debug-print.h"


// platform specific files (CC430)
#include "leds.h"
#include "ports.h"
#include "adc.h"
#include "clock.h"
#include "rtimer.h"
#include "hal_pmm.h"
#include "uart0.h"
#include "spi.h"
#include "rf1a.h"
#include "nullmac.h"
#include "glossy.h"
#include "dma.h"
#include "fram.h"      // provides an interface towards the external memory
#include "async_int.h"


extern volatile uint16_t node_id;

#endif /* __CONTIKI_H__ */
