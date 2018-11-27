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

#define HOST_ID                         1
//#define NODE_ID                       1


/* --- RF config --- */

#define RF_CONF_TX_CH                   5
#define RF_CONF_TX_POWER                RF1A_TX_POWER_0_dBm


/* --- Network parameters --- */

#define LWB_CONF_MAX_HOPS               3
#define LWB_CONF_SCHED_AE_SRC_NODE_CNT  3
#define LWB_CONF_SCHED_AE_SRC_NODE_LIST 2,3,4


/* --- LWB config --- */

/* basics */
#define LWB_CONF_SCHED_PERIOD_IDLE_MS   5000
#define LWB_CONF_MAX_DATA_SLOTS         6
#define LWB_CONF_MAX_PKT_LEN            (128 - 7) /* subtract RF + Glossy hdr */
#define LWB_CONF_IN_BUFFER_SIZE         LWB_CONF_MAX_DATA_SLOTS
#define LWB_CONF_OUT_BUFFER_SIZE        LWB_CONF_MAX_DATA_SLOTS
/* timing */
#define LWB_CONF_T_CONT                 (RTIMER_SECOND_HF / 250)      /* 4ms */
#define LWB_CONF_T_SCHED                (RTIMER_SECOND_HF / 100)     /* 10ms */
#define LWB_CONF_T_GAP                  (RTIMER_SECOND_HF / 500)      /* 2ms */
/* wakeup timer */
#define LWB_CONF_USE_LF_FOR_WAKEUP      1     /* use the low frequency timer */
/* don't change (required for eLWB to work) */
#define LWB_VERSION                     0      /* override default LWB impl. */
#define LWB_SCHED_ELWB                             /* use the eLWB scheduler */
#define LWB_CONF_HEADER_LEN             0
#define LWB_CONF_MAX_N_STREAMS          LWB_CONF_SCHED_AE_SRC_NODE_CNT
/* preprocessing task */
#if LWB_CONF_USE_LF_FOR_WAKEUP          /* time in clock ticks */
#define LWB_CONF_T_PREPROCESS           (RTIMER_SECOND_LF / 50)
#else /* LWB_CONF_USE_LF_FOR_WAKEUP */
#define LWB_CONF_T_PREPROCESS           (RTIMER_SECOND_HF / 50) 
#endif /* LWB_CONF_USE_LF_FOR_WAKEUP */


/* --- BOLT --- */

#define BOLT_CONF_MAX_MSG_LEN           128
#define BOLT_CONF_TIMEREQ_ENABLE        1
#define TIMESYNC_INTERRUPT_BASED        1
#define TIMESYNC_OFS                    193          /* const offset to host */


/* --- MISC --- */

#if LWB_CONF_USE_LF_FOR_WAKEUP
#define BOLT_CONF_TIMEREQ_HF_MODE       0              /* low precision mode */
#else
#define BOLT_CONF_TIMEREQ_HF_MODE       1             /* high precision mode */
#endif /* LWB_CONF_USE_LF_FOR_WAKEUP */
#define RTIMER_CONF_LF_UPDATE_INT       1       /* enable LFXT OVF interrupt */


/* --- DEBUG config --- */

#define DEBUG_PRINT_CONF_ON             1
#define DEBUG_PRINT_CONF_NUM_MSG        5
#define DEBUG_CONF_STACK_GUARD          (SRAM_END - 399)
#define DEBUG_PRINT_CONF_LEVEL          DEBUG_PRINT_LVL_INFO
#define DEBUG_TRACING_ON                1
#define DEBUG_CONF_ISR_INDICATOR        1
#define DEBUG_CONF_ISR_IND_PIN          COM_GPIO3
/* pins */
//#define MCLK_PIN                        COM_MCU_INT2
#define LWB_CONF_TASK_ACT_PIN           COM_GPIO2
#define DEBUG_PRINT_CONF_TASK_ACT_PIN   COM_GPIO2
#define APP_TASK_ACT_PIN                COM_GPIO2


#endif /* __CONFIG_H__ */
