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
 *          Tonio Gsell
 */

#ifndef __CONFIG_H__
#define __CONFIG_H__

/*
 * application specific config file to override default settings
 */

//#define NODE_ID                       13
#ifndef NODE_ID
#define FLOCKLAB_SRC_NODE               1
#else /* FLOCKLAB_SRC_NODE */
#define FLOCKLAB_SRC_NODE               0
#endif /* FLOCKLAB_SRC_NODE */
#define HOST_ID                         13
#define COMPONENT_ID                    0
#define RF_CONF_TX_CH                   10    /* note: use CH/870MHz on roof */
#define RF_CONF_TX_POWER                RF1A_TX_POWER_0_dBm

/* LWB configuration */
#define LWB_CONF_SCHED_PERIOD_IDLE      30       /* define the period length */
#define LWB_SCHED_STATIC                         /* use the static scheduler */
#define LWB_VERSION                     0          /* use the custom version */
#define LWB_CONF_DATA_ACK               0                  /* use data ACKs? */
#define LWB_CONF_T_SCHED2_START         (RTIMER_SECOND_HF * 800 / 1000)
#define LWB_CONF_MAX_HOPS               3
#define LWB_CONF_OUT_BUFFER_SIZE        5
#if defined(NODE_ID) && NODE_ID == HOST_ID
  #define LWB_CONF_IN_BUFFER_SIZE       10
#else
  #define LWB_CONF_IN_BUFFER_SIZE       5  /* smaller queue for source nodes */
#endif /* NODE_ID == HOST_ID */
/* note: for better performance, PKT_LEN should be set as low as possible */
#define LWB_CONF_MAX_PKT_LEN            62
#define LWB_CONF_MAX_DATA_PKT_LEN       62   /* leave 1 byte for payload_len */
#define LWB_CONF_USE_LF_FOR_WAKEUP      1
#define LWB_CONF_MAX_N_STREAMS          10      /* to keep memory usage down */

/* BOLT config */
#define BOLT_CONF_MAX_MSG_LEN           64
#define BOLT_CONF_TIMEREQ_ENABLE        1

/* MISC */
#define SEND_HEALTH_DATA                1
#define WATCHDOG_CONF_ON                1
#define RTIMER_CONF_LF_UPDATE_LED_ON    1
#define ENERGEST_CONF_ON                0
#if !ENERGEST_CONF_ON
  #define DCSTAT_CONF_ON                1
#endif /* ENERGEST_CONF_ON */
#define LOG_CONF_ON                     1
#if NODE_ID == HOST_ID
  #define LOG_CONF_TARGET               LOG_TARGET_BOLT
#else /* LOG_CONF_ON */
  #define LOG_CONF_TARGET               LOG_TARGET_LWB
#endif /* LOG_CONF_ON */
//#define SVS_CONF_ON                   1
//#define FRAM_CONF_ON                  0

/* DEBUG config */
#define DEBUG_PRINT_CONF_LEVEL          DEBUG_PRINT_LVL_INFO
#define DEBUG_PRINT_CONF_NUM_MSG        8
#define DEBUG_ISR_TRAPS_ENABLE          0
#define DEBUG_INTERRUPT_ENABLE          0
#define DEBUG_INTERRUPT_PIN             PORT2, PIN0        /* must be port 2 */
#define DEBUG_LED                       COM_MCU_SPARE2
#define DEBUG_CONF_ISR_INDICATOR        1           /* indicate CPU activity */
#define DEBUG_CONF_ISR_IND_PIN          COM_MCU_INT2      /* show interrupts */
//#define DEBUG_PRINT_CONF_TASK_ACT_PIN COM_MCU_INT2
//#define APP_TASK_ACT_PIN              COM_MCU_INT2
//#define LWB_CONF_TASK_ACT_PIN         COM_MCU_INT2
//#define GLOSSY_TX_PIN                 COM_MCU_INT2


/* global includes */

#include "fw-version.h"


#endif /* __CONFIG_H__ */
