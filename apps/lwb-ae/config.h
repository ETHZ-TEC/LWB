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

#ifndef __CONFIG_H__
#define __CONFIG_H__

/*
 * application specific config file to override default settings
 */

//#define FLOCKLAB      /* uncomment to compile for FlockLab */

/* --- ID config --- */

#define HOST_ID                         1
#ifndef FLOCKLAB
  #define NODE_ID                       6
#endif

/* --- RF config --- */

#ifndef FLOCKLAB
  #define RF_CONF_TX_CH                 5
  #define RF_CONF_TX_POWER              RF1A_TX_POWER_0_dBm
#else
  #define RF_CONF_TX_CH                 10
  #define RF_CONF_TX_POWER              RF1A_TX_POWER_PLUS_10_dBm
#endif

/* --- LWB config --- */

#define LWB_VERSION                     2  /* 1 = original LWB, 2 = modified */
#define LWB_CONF_USE_LF_FOR_WAKEUP      1  /* do NOT change */
/* scheduler */
#define LWB_CONF_SCHED_PERIOD_IDLE      5  /* define the base period length */
#if LWB_VERSION == 1
  #define LWB_SCHED_MIN_ENERGY
  #define LWB_CONF_SCHED_PERIOD_MAX     LWB_CONF_SCHED_PERIOD_IDLE
  #define LWB_CONF_SCHED_PERIOD_MIN     1
  #define LWB_CONF_SCHED_STREAM_REMOVAL_THRES  0
  #define LWB_CONF_T_SCHED2_START       (RTIMER_SECOND_HF / 2)
#else /* LWB_VERSION */
  #define LWB_SCHED_AE                             /* use the 'AE' scheduler */
#endif /* LWB_VERSION */
//#define LWB_CONF_SACK_SLOT            1           /* 1 = enable S-ACK slot */
/* buffer sizes */
#define LWB_CONF_MAX_DATA_SLOTS         6
#define LWB_CONF_MAX_PKT_LEN            63 /* incl. Glossy hdr + length byte */
#define LWB_CONF_MAX_N_STREAMS          3    /* equals max.# number of nodes */ 
#define LWB_CONF_MAX_N_STREAMS_PER_NODE 1
#if defined(FLOCKLAB) || HOST_ID == NODE_ID
  /* on the desk */
  #define LWB_CONF_IN_BUFFER_SIZE       LWB_CONF_MAX_DATA_SLOTS
  #define LWB_CONF_OUT_BUFFER_SIZE      2
#else
  #define LWB_CONF_T_PREPROCESS         20    /* ms */
  #define LWB_CONF_IN_BUFFER_SIZE       1
  #define LWB_CONF_OUT_BUFFER_SIZE      2 
#endif
/* slot durations and network parameters */
#define LWB_CONF_TX_CNT_DATA            2
#define LWB_CONF_MAX_HOPS               3
#define LWB_CONF_T_SCHED                (RTIMER_SECOND_HF / 100) /* 10ms */
#define LWB_CONF_T_CONT                 (RTIMER_SECOND_HF / 250) /* 4ms */
#define LWB_CONF_T_GAP                  (RTIMER_SECOND_HF / 500) /* 2ms */

/* --- DEBUG config --- */

#define DEBUG_PRINT_CONF_ON             0
#define DEBUG_PRINT_CONF_NUM_MSG        10
#define DEBUG_CONF_STACK_GUARD          (SRAM_END - 399)
#define DEBUG_PRINT_CONF_LEVEL          DEBUG_PRINT_LVL_INFO
#define DEBUG_TRACING_ON                0
/* pins */
#define GLOSSY_START_PIN                FLOCKLAB_LED1

//#define RF_GDO2_PIN                   FLOCKLAB_LED3
#define LWB_CONF_TASK_ACT_PIN           COM_MCU_INT1
//#define DEBUG_PRINT_TASK_ACT_PIN      FLOCKLAB_INT1
//#define APP_TASK_ACT_PIN              FLOCKLAB_INT1
//#define RF_GDO2_PIN                   FLOCKLAB_INT2

/* tracing */
#if DEBUG_TRACING_ON
  /* SOURCE node: generate a pulse before a request or data packet is sent */
  #define LWB_REQ_IND_PIN               FLOCKLAB_LED1
  #define LWB_DATA_IND_PIN              FLOCKLAB_LED1
  
  /* nasty hack, for debugging only; idea is to have 3 different pins (one for
   * each source node) which all assert when a contention (request) is detected
   * and deassert individually upon successful reception of the data packet 
   * from that node */
  /* code that will be executed...
   * ... upon 1st successful packet reception during a glossy flood (all nodes)
   * ... when a request was detected during a contention slot (host node only)
   * ... before a data slot starts (host node only) */
  #define GLOSSY_FIRST_RX       { if((slot_node_id) == 6) {\
                                    PIN_CLR(FLOCKLAB_LED1);\
                                  } else if ((slot_node_id) == 22) {\
                                    PIN_CLR(FLOCKLAB_INT1);\
                                  } else if ((slot_node_id) == 28) {\
                                    PIN_CLR(FLOCKLAB_INT2);\
                                  }\
                                  slot_node_id = 0;\
                                }  
  #define LWB_REQ_DETECTED      { PIN_SET(FLOCKLAB_LED1);\
                                  PIN_SET(FLOCKLAB_INT1);\
                                  PIN_SET(FLOCKLAB_INT2); }
  /* slot_node_id is only set when one pkt was already rcvd from that node */
  #define LWB_DATA_SLOT_STARTS  { if((i > 0) && (schedule.slot[i - 1] \
                                     == schedule.slot[i]) && LWB_DATA_RCVD) {\
                                    slot_node_id = schedule.slot[i];\
                                  } else {\
                                    slot_node_id = 0;\
                                  } }
#endif /* DEBUG_TRACING_ON */

#ifndef FLOCKLAB
  #define BOLT_CONF_ON                  1
#endif /* FLOCKLAB */

/* --- GLOBAL variables --- */

extern uint16_t slot_node_id;   /* for debugging/tracing only */

#endif /* __CONFIG_H__ */
