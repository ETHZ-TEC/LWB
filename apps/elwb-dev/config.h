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

//#define DEBUG                            /* uncomment to enable debug mode */


/* --- Node and component ID --- */

/* uncomment NODE_ID to compile FW image for the host node */
//#define NODE_ID                         HOST_ID
#define HOST_ID                         16

#define COMPONENT_ID                    DPP_COMPONENT_ID_CC430
#define IS_HOST                         (NODE_ID == HOST_ID)


/* --- External memory (FRAM) --- */

#define FRAM_CONF_ON                    0   /* enable if FRAM chip installed */


/* --- Radio config --- */

#define RF_CONF_TX_CH                   10                  /* CH10 = 870MHz */
#define RF_CONF_TX_POWER                RF1A_TX_POWER_0_dBm
#define RF_CONF_MAX_PKT_LEN             128


/* --- Network parameters --- */

#define ELWB_CONF_N_HOPS                3
#if 0 && IS_HOST
#define ELWB_CONF_SCHED_NODE_LIST       21001, 21002, 21003, 21004, 21005, \
                                        21006, 21007, 21008, 21009, 21010, \
                                        21011, 21012, 21013, 21014, 21015, \
                                        21016, 21017, 21018, 21019, 21020, \
                                        21021, 21022, 21023, 21024, 21025, \
                                        21026    /* predefined list of nodes */
#endif /* IS_HOST */


/* --- eLWB config --- */

#define ELWB_CONF_SCHED_PERIOD_IDLE     15            /* period T in seconds */
#define ELWB_CONF_SCHED_PERIOD_MIN      3        /* min. allowed period in s */
#define ELWB_CONF_SCHED_CRC             1          /* append CRC to schedule */
#define ELWB_CONF_SCHED_NODE_TIMEOUT    3600        /* remove node after x s */
#define ELWB_CONF_MAX_DATA_SLOTS        40    /* max. # data slots per round */
#define ELWB_CONF_MAX_N_NODES           40        /* max. # nodes in network */
#define ELWB_CONF_DATA_ACK              1  /* use data ACKs from src to host */
/* packet and buffer size */
#define ELWB_CONF_MAX_PKT_LEN           (128 - GLOSSY_MAX_HEADER_LEN)
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
  #define ELWB_CONF_OUT_BUFFER_SIZE     10 /* = max #pkts a src node can snd */
  #define ELWB_CONF_IN_BUFFER_SIZE      10 /* = max #pkts a src node can rcv */
 #else /* ELWB_CONF_USE_XMEM */
  #define ELWB_CONF_OUT_BUFFER_SIZE     6 /* = max #pkts a src node can send */
  #define ELWB_CONF_IN_BUFFER_SIZE      4  /* = max #pkts a src node can rcv */
 #endif /* ELWB_CONF_USE_XMEM */
#endif /* IS_HOST */
#define ELWB_CONF_CONT_TH               2 /* threshold for # packets in TX
                                             queue before contending */
/* timings */
#define ELWB_CONF_T_CONT                (ELWB_RTIMER_SECOND / 200)    /* 5ms */
#define ELWB_CONF_T_SCHED               (ELWB_RTIMER_SECOND / 50)    /* 20ms */
#define ELWB_CONF_T_GAP                 (ELWB_RTIMER_SECOND / 500)    /* 2ms */
#define ELWB_CONF_T_GUARD_SLOT          (ELWB_RTIMER_SECOND / 4000)/* 0.25ms */
#define ELWB_CONF_T_GUARD_ROUND         (ELWB_RTIMER_SECOND / 1000)   /* 1ms */
#define ELWB_CONF_T_SILENT              (120 * ELWB_RTIMER_SECOND)   /* 2min */
#define ELWB_CONF_T_DEEPSLEEP           (ELWB_RTIMER_SECOND * 1800) /* 30min */
/* retransmissions */
#define ELWB_CONF_N_TX_SCHED            3
#define ELWB_CONF_N_TX_DATA             3
/* misc */
#if IS_HOST
 #define ELWB_CONF_WRITE_TO_BOLT        1      /* write incoming msg to BOLT */
  #define ELWB_CONF_T_PREPROCESS        (ELWB_RTIMER_SECOND / 10)   /* 100ms */
#else /* IS_HOST */
 #define ELWB_CONF_WRITE_TO_BOLT        0
  #define ELWB_CONF_T_PREPROCESS        (ELWB_RTIMER_SECOND / 20)    /* 50ms */
#endif /* IS_HOST */
/* override default packet filter (only keep pkts on src that match node_id) */
#define ELWB_CONF_SRC_PKT_FILTER(data)  (data[2] == node_id || \
                                         data[2] == 0xffff)
#define LWB_VERSION                     0       /* exclude default LWB impl. */

/* Glossy */
#define GLOSSY_CONF_HEADER_BYTE         0xc0
#define GLOSSY_CONF_PAYLOAD_LEN         128
#define GLOSSY_CONF_SETUPTIME_WITH_SYNC 1200


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
#ifdef DEBUG
#define WATCHDOG_CONF_TIMER_MODE        1
#endif /* DEBUG */
#define DCSTAT_CONF_ON                  1  /* use DCSTAT instead of ENERGEST */
#define EVENT_CONF_ON                   1
#if IS_HOST
  #define EVENT_CONF_TARGET             EVENT_TARGET_BOLT
#else /* IS_HOST */
  #define EVENT_CONF_TARGET             EVENT_TARGET_LWB
#endif /* IS_HOST */
#define SVS_CONF_ON                     0                   /* don't use SVS */
#define RTIMER_CONF_LF_UPDATE_INT       0      /* disable LFXT OVF interrupt */
#define NVCFG_CONF_BLOCK_SIZE           6
#define FW_UPDATE_CONF_ON               FRAM_CONF_ON


/* --- DEBUG config --- */

#define DEBUG_PRINT_CONF_ON             1
#define DEBUG_PRINT_CONF_LEVEL          DEBUG_PRINT_LVL_INFO
#define DEBUG_PRINT_CONF_BUFFER_SIZE    512  /* debug print buffer size in b */
#define DEBUG_PRINT_CONF_MSG_LEN        100 /* max debug msg length per line */
#define DEBUG_PRINT_CONF_USE_XMEM       0
#define DEBUG_PRINT_CONF_PRINT_NODEID   1
#define DEBUG_CONF_STACK_GUARD          (SRAM_START + 3588)  /* bss+dec size */
#define DEBUG_CONF_P1INT_EN             0
#define DEBUG_CONF_ISR_IND_PIN          COM_GPIO3     /* pin 9 on DBG header */
#define DEBUG_PRINT_CONF_TASK_ACT_PIN   COM_GPIO2     /* pin 8 on DBG header */
#define APP_TASK_ACT_PIN                COM_GPIO2
#define ELWB_CONF_TASK_ACT_PIN          COM_GPIO2     /* runs in ISR context */
#define GLOSSY_START_PIN                LED_STATUS /* use the default (LED0) */
#define RF_GDO2_PIN                     COM_GPIO1

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
