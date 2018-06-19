/*
 * Copyright (c) 2018, Swiss Federal Institute of Technology (ETH Zurich).
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

/* important: node ID must be set accordingly if host is to be programmed (does
 * not work with objcopy in makefile for the host!) */
#define NODE_ID                         HOST_ID
#define HOST_ID                         16 //6

#define COMPONENT_ID                    DPP_COMPONENT_ID_CC430
#define IS_HOST                         (NODE_ID == HOST_ID)


/* --- External memory (FRAM) --- */

#define FRAM_CONF_ON                    1   /* enable if FRAM chip installed */


/* --- Radio config --- */

#define ELWB_CONF_RF_CH_PRIMARY         10 //8            /* CH10 = 870MHz */
#define ELWB_CONF_RF_CH_SECONDARY       10
#define RF_CONF_TX_CH                   ELWB_CONF_RF_CH_PRIMARY
#define RF_CONF_TX_POWER                RF1A_TX_POWER_0_dBm
#define RF_CONF_MAX_PKT_LEN             128


/* --- Network parameters --- */

#define ELWB_CONF_MAX_HOPS              3
//#define ELWB_CONF_SCHED_AE_SRC_NODE_LIST 2,3,4  /* predefined list of nodes */
//#define ELWB_CONF_SCHED_AE_SRC_NODE_CNT  3         /* # nodes in list above */


/* --- eLWB config --- */

#define ELWB_CONF_SCHED_PERIOD_IDLE     15            /* period T in seconds */
#define ELWB_CONF_SCHED_PERIOD_MIN      3        /* min. allowed period in s */
#define ELWB_CONF_SCHED_NODE_TIMEOUT    43200       /* remove node after x s */
#define ELWB_CONF_MAX_DATA_SLOTS        50    /* max. # data slots per round */
#define ELWB_CONF_MAX_N_NODES           50        /* max. # nodes in network */
#define ELWB_CONF_DATA_ACK              1  /* use data ACKs from src to host */
/* packet and buffer size */
#define ELWB_CONF_MAX_PKT_LEN           (128 - 2)     /* subtract Glossy hdr */
#define ELWB_CONF_USE_XMEM              FRAM_CONF_ON      /* if FRAM enabled */
#if IS_HOST
 #if ELWB_CONF_USE_XMEM
  #define ELWB_CONF_OUT_BUFFER_SIZE     10  /* = max #pkts the host can send */
 #else /* ELWB_CONF_USE_XMEM */
  #define ELWB_CONF_OUT_BUFFER_SIZE     3
 #endif /* ELWB_CONF_USE_XMEM */
 #define ELWB_CONF_IN_BUFFER_SIZE       1 /* typically ELWB_CONF_MAX_DATA_SLOTS
                                       but not required if forwarded to BOLT */
#else /* IS_HOST */
 #if ELWB_CONF_USE_XMEM
  #define ELWB_CONF_OUT_BUFFER_SIZE     5 /* = max #pkts a src node can send */
  #define ELWB_CONF_IN_BUFFER_SIZE      10 /* = max #pkts a src node can rcv */
 #else /* ELWB_CONF_USE_XMEM */
  #define ELWB_CONF_OUT_BUFFER_SIZE     3 /* = max #pkts a src node can send */
  #define ELWB_CONF_IN_BUFFER_SIZE      3  /* = max #pkts a src node can rcv */   
 #endif /* ELWB_CONF_USE_XMEM */
#endif /* IS_HOST */
/* timings */
#define ELWB_CONF_T_CONT                (RTIMER_SECOND_HF / 200)      /* 5ms */
#define ELWB_CONF_T_SCHED               (RTIMER_SECOND_HF / 50)      /* 20ms */
#define ELWB_CONF_T_GAP                 (RTIMER_SECOND_HF / 500)      /* 2ms */
#define ELWB_CONF_T_GUARD               (RTIMER_SECOND_HF / 2000)   /* 0.5ms */
#define ELWB_CONF_T_GUARD_LF            (RTIMER_SECOND_LF / 1000)     /* 1ms */
#define ELWB_CONF_T_REF_OFS             3822 /* measured with logic analyzer */
#define ELWB_CONF_T_SILENT              (120 * RTIMER_SECOND_HF)     /* 2min */
#define ELWB_CONF_T_DEEPSLEEP           (RTIMER_SECOND_LF * 1800)   /* 30min */
/* retransmissions */
#define ELWB_CONF_TX_CNT_SCHED          3
#define ELWB_CONF_TX_CNT_DATA           3
/* misc */
#if IS_HOST
 #define ELWB_CONF_WRITE_TO_BOLT        1      /* write incoming msg to BOLT */
  #define ELWB_CONF_T_PREPROCESS_LF     (RTIMER_SECOND_LF / 10)     /* 100ms */
#else /* IS_HOST */
 #define ELWB_CONF_WRITE_TO_BOLT        0
  #define ELWB_CONF_T_PREPROCESS_LF     (RTIMER_SECOND_LF / 20)      /* 50ms */
#endif /* IS_HOST */
/* override default packet filter (only keep pkts on src that match node_id) */ 
#define ELWB_CONF_SRC_PKT_FILTER(data)  (data[2] == node_id || \
                                         data[2] == 0xffff)
#define LWB_VERSION                     0       /* exclude default LWB impl. */

/* Glossy */
#define GLOSSY_COMMON_HEADER            0xc0


/* --- BOLT config --- */

#define BOLT_CONF_MAX_MSG_LEN           ELWB_CONF_MAX_PKT_LEN
#define BOLT_CONF_TIMEREQ_ENABLE        1
#define BOLT_CONF_TIMEREQ_HF_MODE       0 /* only low freq. mode is supported*/


/* --- MISC --- */

#define HEALTH_MSG_PERIOD               (ELWB_CONF_SCHED_PERIOD_IDLE * 10)
                                        /* inital value only, can be changed */
#define UTC_TIMESTAMP_MAX_DRIFT         5  /* for host only, allowed drift 
                                            before time is adjusted (= jump) */
#define WATCHDOG_CONF_ON                1
#define RTIMER_CONF_LF_UPDATE_LED_ON    0
#define DCSTAT_CONF_ON                  1  /* use DCSTAT instead of ENERGEST */
#define EVENT_CONF_ON                   1
#if IS_HOST
  #define EVENT_CONF_TARGET             EVENT_TARGET_BOLT
#else /* IS_HOST */
  #define EVENT_CONF_TARGET             EVENT_TARGET_LWB
#endif /* IS_HOST */
//#define SVS_CONF_ON                   1
#define DEBUG_PRINT_CONF_USE_XMEM       0
#define RTIMER_CONF_LF_UPDATE_INT       1       /* enable LFXT OVF interrupt */
#define NVCFG_CONF_BLOCK_SIZE           6
#define FW_UPDATE_CONF_ON               FRAM_CONF_ON


/* --- DEBUG config --- */

#define DEBUG_PRINT_CONF_ON             1
#define DEBUG_PRINT_CONF_LEVEL          DEBUG_PRINT_LVL_INFO
#define DEBUG_PRINT_CONF_USE_RINGBUFFER 1
#define DEBUG_PRINT_CONF_BUFFER_SIZE    512
#define DEBUG_PRINT_CONF_PRINT_NODEID   1
#define DEBUG_CONF_STACK_GUARD          (SRAM_START + 3588)
                                         /* -> .bss + .dec size */
//#define DEBUG_CONF_ISR_INDICATOR        1         /* indicate CPU activity */
#define DEBUG_CONF_ISR_IND_PIN          COM_GPIO3     /* pin 9 on DBG header */
#define DEBUG_PRINT_CONF_TASK_ACT_PIN   COM_GPIO2     /* pin 8 on DBG header */
#define APP_TASK_ACT_PIN                COM_GPIO2
#define ELWB_CONF_TASK_ACT_PIN           COM_GPIO3
//#define GLOSSY_START_PIN              COM_GPIO3  /* use the default (LED0) */
#define RF_GDO2_PIN                     COM_GPIO1
//#define GLOSSY_TX_PIN                 COM_MCU_INT2
//#define MCLK_PIN                      COM_MCU_INT2

#define LED_CONF_ON                     0     /* override default LED on/off */
#define LED_ON(portandpin)              if(!(cfg.dbg_flags & 0x01)) \
                                          PIN_SET_I(portandpin)
#define LED_OFF(portandpin)             if(!(cfg.dbg_flags & 0x01)) \
                                          PIN_CLR_I(portandpin)
#define LED_TOGGLE(portandpin)          if(!(cfg.dbg_flags & 0x01)) \
                                          PIN_XOR_I(portandpin)


/* --- Global includes, typedefs and variables --- */

#include "fw-version.h"

/* non-volatile configuration and stats (length must be NVCFG_CONF_BLOCK_SIZE
 * bytes) */
typedef struct {
  uint16_t  node_id;
  uint8_t   rst_cnt;
  uint8_t   dbg_flags;     /* debug flags:
                              0x01    set to turn LEDs off
                              0x02    disable eLWB ACK mechanism on source */
  uint8_t   tx_pwr;        /* transmit power for radio */ 
  uint8_t   spare;
} config_t;

extern config_t cfg;           /* most important config parameters and stats */


/* --- Compile time parameter checks --- */

#if DPP_MSG_PKT_LEN > ELWB_CONF_MAX_PKT_LEN
#error "ELWB_CONF_MAX_PKT_LEN is too small"
#endif



#endif /* __CONFIG_H__ */
