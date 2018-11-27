/*
 * Copyright (c) 2017, Swiss Federal Institute of Technology (ETH Zurich).
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
 *          Felix Sutton
 */

#ifndef __CONFIG_H__
#define __CONFIG_H__

/*
 * application specific config file to override default settings
 */

/* --- Node ID --- */

#define FLOCKLAB
#define HOST_ID                         1
#ifndef FLOCKLAB
  #define NODE_ID                       1
#endif


/* --- RF config --- */

#define RF_CONF_TX_CH                   5
#define RF_CONF_TX_POWER                RF1A_TX_POWER_0_dBm


/* --- Network parameters --- */

#define LWB_CONF_MAX_HOPS               3
#define LWB_CONF_SCHED_AE_SRC_NODE_CNT  2
#define LWB_CONF_SCHED_AE_SRC_NODE_LIST 2,3    // on Flocklab: 13,25


/* --- LWB config --- */

/* basics */
#define LWB_CONF_SCHED_PERIOD_IDLE_MS   1000
#define LWB_CONF_MAX_DATA_SLOTS         6
#define LWB_CONF_MAX_PKT_LEN            (32 - 7) /* subtract RF + Glossy hdr */
#define LWB_CONF_IN_BUFFER_SIZE         LWB_CONF_MAX_DATA_SLOTS
#define LWB_CONF_OUT_BUFFER_SIZE        LWB_CONF_MAX_DATA_SLOTS
/* timing */
#define LWB_CONF_T_CONT                 (RTIMER_SECOND_HF / 250)      /* 4ms */
#define LWB_CONF_T_SCHED                (RTIMER_SECOND_HF / 100)     /* 10ms */
#define LWB_CONF_T_GAP                  (RTIMER_SECOND_HF / 500)      /* 2ms */
#define LWB_CONF_T_PREPROCESS           20                          /* in ms */
/* power */
#define LWB_CONF_USE_LF_FOR_WAKEUP      1
/* don't change (required for eLWB to work) */
#define LWB_VERSION                     0      /* override default LWB impl. */
#define LWB_SCHED_ELWB                             /* use the eLWB scheduler */
#define LWB_CONF_HEADER_LEN             0
#define LWB_CONF_MAX_N_STREAMS          LWB_CONF_SCHED_AE_SRC_NODE_CNT


/* --- BOLT --- */

#define BOLT_CONF_MAX_MSG_LEN           64
#define BOLT_CONF_TIMEREQ_ENABLE        1
#define BOLT_CONF_TIMEREQ_HF_MODE       0              /* low precision mode */
#define TIMESYNC_INTERRUPT_BASED        1
#define TIMESYNC_OFS                    -193         /* const offset to host */


/* --- MISC --- */

#define RTIMER_CONF_LF_UPDATE_INT       0      /* disable LFXT OVF interrupt */
//#define FRAM_CONF_ON                  1


/* --- DEBUG config --- */

#define DEBUG_PRINT_CONF_ON             1
#define DEBUG_PRINT_CONF_NUM_MSG        10
#define DEBUG_CONF_STACK_GUARD          (SRAM_END - 399)
#define DEBUG_PRINT_CONF_LEVEL          DEBUG_PRINT_LVL_INFO
#define DEBUG_TRACING_ON                1
/* pins */
//#define MCLK_PIN                        COM_MCU_INT2
#define LWB_CONF_TASK_ACT_PIN           COM_MCU_INT2
//#define DEBUG_PRINT_CONF_TASK_ACT_PIN   COM_MCU_INT2
//#define APP_TASK_ACT_PIN                COM_MCU_INT2


#endif /* __CONFIG_H__ */
