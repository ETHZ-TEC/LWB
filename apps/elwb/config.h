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

#define FLOCKLAB      /* uncomment to compile for FlockLab */

/* --- General config --- */

#define HOST_ID                         1
#ifndef FLOCKLAB
  #define NODE_ID                       1
#endif

//#define FRAM_CONF_ON                  1


/* --- RF config --- */

#define RF_CONF_TX_CH                   5
#define RF_CONF_TX_POWER                RF1A_TX_POWER_MINUS_6_dBm


/* --- LWB config --- */

#define LWB_VERSION                     0  /* override default LWB impl. */
#define LWB_CONF_USE_LF_FOR_WAKEUP      1
#define LWB_CONF_SCHED_PERIOD_IDLE      1   /* define the base period length */
#define LWB_SCHED_AE                               /* use the 'AE' scheduler */
#define LWB_CONF_T_CONT                 (RTIMER_SECOND_HF / 250)      /* 4ms */
#define LWB_CONF_MAX_DATA_SLOTS         6
#define LWB_CONF_MAX_PKT_LEN            56
#define LWB_CONF_MAX_N_STREAMS          3     /* equals max. number of nodes */ 
#define LWB_CONF_MAX_N_STREAMS_PER_NODE 1
#if defined(FLOCKLAB) || HOST_ID == NODE_ID
  #define LWB_CONF_IN_BUFFER_SIZE       LWB_CONF_MAX_DATA_SLOTS
  #define LWB_CONF_OUT_BUFFER_SIZE      2
#else
  #define LWB_CONF_IN_BUFFER_SIZE       1
  #define LWB_CONF_OUT_BUFFER_SIZE      2 
#endif
/* slot durations and network parameters */
#define LWB_CONF_TX_CNT_DATA            2
#define LWB_CONF_MAX_HOPS               3
#define LWB_CONF_T_SCHED                (RTIMER_SECOND_HF / 100) /* 10ms */
#define LWB_CONF_T_GAP                  (RTIMER_SECOND_HF / 500) /* 2ms */
#define LWB_CONF_T_PREPROCESS           20    /* in ms */
#define LWB_CONF_T_SILENT               (RTIMER_SECOND_HF * 20)

#if defined(NODE_ID) && NODE_ID != HOST_ID /* can't disable LFXT OVF on host */
#define RTIMER_CONF_LF_UPDATE_INT       0
#endif

#define LWB_CONF_SCHED_AE_SRC_NODE_CNT  2
#define LWB_CONF_SCHED_AE_SRC_NODE_LIST 13,25


/* --- BOLT --- */
#define BOLT_CONF_MAX_MSG_LEN           64
#define BOLT_CONF_TIMEREQ_ENABLE        1
#define BOLT_CONF_TIMEREQ_HF_MODE       0              /* low precision mode */
#define TIMESYNC_INTERRUPT_BASED        1
#define TIMESYNC_OFS                    -193         /* const offset to host */


/* --- DEBUG config --- */

#define DEBUG_PRINT_CONF_ON             1
#define DEBUG_PRINT_CONF_NUM_MSG        10
#define DEBUG_CONF_STACK_GUARD          (SRAM_END - 399)
#define DEBUG_PRINT_CONF_LEVEL          DEBUG_PRINT_LVL_INFO
#define DEBUG_TRACING_ON                1
/* pins */
//#define MCLK_PIN                        COM_MCU_INT2
//#define LWB_CONF_TASK_ACT_PIN           COM_MCU_INT2
//#define DEBUG_PRINT_TASK_ACT_PIN        COM_MCU_INT2
//#define APP_TASK_ACT_PIN                COM_MCU_INT2


#endif /* __CONFIG_H__ */
