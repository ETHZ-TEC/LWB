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

/* --- Node and component ID --- */

//#define NODE_ID                         2
#define HOST_ID                         1
#define COMPONENT_ID                    DPP_COMPONENT_ID_CC430


/* --- Radio config --- */

#define RF_CONF_TX_CH                   10    /* note: use CH/870MHz on roof */
#define RF_CONF_TX_POWER                RF1A_TX_POWER_PLUS_10_dBm


/* --- Network parameters --- */

#define LWB_CONF_MAX_HOPS               3
#define LWB_CONF_SCHED_AE_SRC_NODE_CNT  3
#define LWB_CONF_SCHED_AE_SRC_NODE_LIST 2,3,4


/* --- LWB / eLWB config --- */

#define LWB_CONF_SCHED_PERIOD_IDLE      5            /* period T in seconds */
#define LWB_CONF_MAX_DATA_SLOTS         (LWB_CONF_SCHED_AE_SRC_NODE_CNT + 2)
#define LWB_CONF_DATA_ACK               0                  /* use data ACKs? */
/* packet and buffer size */
#define LWB_CONF_MAX_PKT_LEN            (128 - 7)  /* subtract RF&Glossy hdr */
#define LWB_CONF_IN_BUFFER_SIZE         LWB_CONF_MAX_DATA_SLOTS
#define LWB_CONF_OUT_BUFFER_SIZE        LWB_CONF_MAX_DATA_SLOTS
/* timings */
#define LWB_CONF_T_CONT                 (RTIMER_SECOND_HF / 250)      /* 4ms */
#define LWB_CONF_T_SCHED                (RTIMER_SECOND_HF / 125)      /* 8ms */
#define LWB_CONF_T_GAP                  (RTIMER_SECOND_HF / 500)      /* 2ms */
#define LWB_CONF_T_GUARD                (RTIMER_SECOND_HF / 4000)
#define LWB_CONF_T_REF_OFS              3822 /* measured with logic analyzer */
#define LWB_CONF_T_SILENT               (120 * RTIMER_SECOND_HF)     /* 2min */
#define LWB_CONF_T_DEEPSLEEP            (RTIMER_SECOND_LF * 1800)   /* 30min */
/* retransmissions */
#define LWB_CONF_TX_CNT_SCHED           3
#define LWB_CONF_TX_CNT_DATA            3
/* eLWB specific config */
#define LWB_VERSION                     0      /* override default LWB impl. */
#define LWB_SCHED_ELWB_DYN                         /* use the eLWB scheduler */
#define LWB_CONF_HEADER_LEN             0
#define LWB_CONF_MAX_N_STREAMS          LWB_CONF_MAX_DATA_SLOTS
/* preprocessing task */
#define LWB_CONF_T_PREPROCESS           (RTIMER_SECOND_LF / 20)     /* 50ms */
#define LWB_CONF_FORWARD_PKT(data)      (*(uint16_t*)(data + 4) == node_id)
/* Glossy */
#define GLOSSY_COMMON_HEADER            0xc0


/* --- BOLT config --- */

#define BOLT_CONF_MAX_MSG_LEN           LWB_CONF_MAX_PKT_LEN
#define BOLT_CONF_TIMEREQ_ENABLE        1
#define TIMESYNC_INTERRUPT_BASED        1     /* only ISR based is supported */
#define TIMESYNC_OFS                    193          /* const offset to host */
#define BOLT_CONF_TIMEREQ_HF_MODE       0 /* only low freq. mode is supported*/


/* --- MISC --- */

#define HEALTH_MSG_PERIOD               (LWB_CONF_SCHED_PERIOD_IDLE * 10)
#define WATCHDOG_CONF_ON                1
#define RTIMER_CONF_LF_UPDATE_LED_ON    1
#define ENERGEST_CONF_ON                0
#if !ENERGEST_CONF_ON
  #define DCSTAT_CONF_ON                1
#endif /* ENERGEST_CONF_ON */
#define EVENT_CONF_ON                   1
#if NODE_ID == HOST_ID
  #define EVENT_CONF_TARGET             LOG_TARGET_BOLT
#else /* LOG_CONF_ON */
  #define EVENT_CONF_TARGET             LOG_TARGET_LWB
#endif /* LOG_CONF_ON */
//#define SVS_CONF_ON                   1
//#define FRAM_CONF_ON                  0
#define RTIMER_CONF_LF_UPDATE_INT       1       /* enable LFXT OVF interrupt */


/* --- DEBUG config --- */

#define DEBUG_PRINT_CONF_ON             1
#define DEBUG_PRINT_CONF_LEVEL          DEBUG_PRINT_LVL_INFO
#define DEBUG_CONF_STACK_GUARD          (SRAM_START + 3500)
                                         /* -> bss size max 3500 */
#define DEBUG_PRINT_CONF_USE_RINGBUFFER 1
#define DEBUG_PRINT_CONF_BUFFER_SIZE    400
#define DEBUG_ISR_TRAPS_ENABLE          0
#define DEBUG_INTERRUPT_ENABLE          0
#define DEBUG_INTERRUPT_PIN             PORT2, PIN0        /* must be port 2 */
#define DEBUG_LED                       COM_MCU_SPARE2
//#define DEBUG_CONF_ISR_INDICATOR        1         /* indicate CPU activity */
//#define DEBUG_CONF_ISR_IND_PIN          COM_GPIO3   /* pin 9 on DBG header */
#define DEBUG_PRINT_CONF_TASK_ACT_PIN   COM_GPIO2     /* pin 8 on DBG header */
#define APP_TASK_ACT_PIN                COM_GPIO2
#define LWB_CONF_TASK_ACT_PIN           COM_GPIO2
#define GLOSSY_START_PIN                COM_GPIO3  /* use the default (LED0) */
#define RF_GDO2_PIN                     COM_GPIO1
//#define GLOSSY_TX_PIN                 COM_MCU_INT2
//#define MCLK_PIN                      COM_MCU_INT2


/* --- Global includes --- */

#include "fw-version.h"


#endif /* __CONFIG_H__ */
