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

/* --- ID config --- */

#define HOST_ID    1
#ifndef FLOCKLAB
  #define NODE_ID                       1
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

#define LWB_CONF_USE_XMEM               0       /* don't use external memory */
/* scheduler */
#define LWB_VERSION                     2            /* use the modified LWB */
#define LWB_SCHED_AE                               /* use the 'AE' scheduler */
#define LWB_CONF_SCHED_PERIOD_IDLE      5   /* define the base period length */
/* buffer sizes */
#define LWB_CONF_MAX_DATA_SLOTS         10 /* equals the # nodes & # streams */
#define LWB_CONF_MAX_PACKET_LEN         64
#define LWB_CONF_MAX_DATA_PKT_LEN       (LWB_CONF_MAX_PACKET_LEN)
#define LWB_CONF_MAX_N_STREAMS          LWB_CONF_MAX_DATA_SLOTS 
#if defined(FLOCKLAB) || HOST_ID == NODE_ID
  #define LWB_CONF_IN_BUFFER_SIZE       LWB_CONF_MAX_DATA_SLOTS
  #define LWB_CONF_OUT_BUFFER_SIZE      1
#else
  #define LWB_CONF_IN_BUFFER_SIZE       1
  #define LWB_CONF_OUT_BUFFER_SIZE      1 
#endif
/* slot durations and network parameters */
#define LWB_CONF_TX_CNT_DATA            2
#define LWB_CONF_MAX_HOPS               3
#define LWB_CONF_T_SCHED                (RTIMER_SECOND_HF / 100) /* 10ms */
#define LWB_CONF_T_CONT                 (RTIMER_SECOND_HF / 250) /* 4ms */
#define LWB_CONF_T_GAP                  (RTIMER_SECOND_HF / 500) /* 2ms */

/* --- DEBUG config --- */

#define DEBUG_PRINT_CONF_NUM_MSG        10
#define DEBUG_CONF_STACK_GUARD          (SRAM_END - 0x01ff)
#define DEBUG_PRINT_CONF_LEVEL          DEBUG_PRINT_LVL_INFO
#ifndef FLOCKLAB
  #define LWB_CONF_TASK_ACT_PIN         PORT2, PIN6
  #define DEBUG_PRINT_TASK_ACT_PIN      PORT2, PIN6
  #define APP_TASK_ACT_PIN              PORT2, PIN6
#else
  #define LWB_CONF_TASK_ACT_PIN         FLOCKLAB_INT1
  #define DEBUG_PRINT_TASK_ACT_PIN      FLOCKLAB_INT1
  #define APP_TASK_ACT_PIN              FLOCKLAB_INT1
#endif

/* --- GENERAL config --- */

#define FRAM_CONF_ON                    0
#define BOLT_CONF_ON                    0
#define CLOCK_CONF_FLL_ON               1

#endif /* __CONFIG_H__ */
