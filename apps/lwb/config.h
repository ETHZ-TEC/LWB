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
 */

#ifndef __CONFIG_H__
#define __CONFIG_H__

/*
 * application specific config file to override default settings
 */

#define FLOCKLAB                             /* uncomment to run on FlockLAB */
#define HOST_ID                         1
#define NUM_NODES                       30
#define SOURCE_IPI                      30   /* seconds */

#ifdef FLOCKLAB
  /* set the highest antenna gain if the program runs on FlockLAB */
  #define GMW_CONF_RF_TX_POWER          GMW_RF_TX_POWER_PLUS_10_dBm
  #define RF_CONF_TX_CH                 5      /* approx. 869 MHz */
  #define GLOSSY_START_PIN              FLOCKLAB_LED1
  #define RF_GDO2_PIN                   FLOCKLAB_INT1
  #define LWB_CONF_TASK_ACT_PIN         FLOCKLAB_INT2
  #define DEBUG_PRINT_CONF_TASK_ACT_PIN FLOCKLAB_INT2
  #define APP_TASK_ACT_PIN              FLOCKLAB_INT2
  /* note: FLOCKLAB_LED2 should not be used */
#else
  /* only define a node ID if FlockLab is not used (FlockLab automatically
   * assigns node IDs); select an ID other than HOST_ID to compile the code 
   * for a source node */
  #define NODE_ID                       1
#endif /* FLOCKLAB */

/* LWB configuration */
#define LWB_SCHED_MIN_ENERGY             /* use the minimum energy scheduler */
//#define LWB_SCHED_STATIC                       /* use the static scheduler */
#define LWB_CONF_SCHED_PERIOD_IDLE      5        /* define the period length */
#define LWB_CONF_SCHED_PERIOD_MIN       2
#define LWB_CONF_SCHED_PERIOD_MAX       30
#define LWB_CONF_OUT_BUFFER_SIZE        4
#define LWB_CONF_IN_BUFFER_SIZE         NUM_NODES
#define LWB_CONF_USE_LF_FOR_WAKEUP      0
#define LWB_CONF_MAX_PKT_LEN            80
#define LWB_CONF_MAX_DATA_PKT_LEN       15
#define LWB_CONF_MAX_DATA_SLOTS         NUM_NODES

#define LWB_CONF_T_SCHED                (RTIMER_SECOND_HF / 40)     /* 25ms */
#define LWB_CONF_T_DATA                 (RTIMER_SECOND_HF / 50)     /* 20ms */
#define LWB_CONF_T_GUARD_1              (RTIMER_SECOND_HF / 1000)   /* 1ms */
#define LWB_CONF_T_GUARD_2              (RTIMER_SECOND_HF / 1000)
#define LWB_CONF_T_GUARD_3              (RTIMER_SECOND_HF / 1000)
#define LWB_CONF_T_GUARD                (RTIMER_SECOND_HF / 2000)   /* 0.5ms */
#define LWB_CONF_T_GAP                  (RTIMER_SECOND_HF / 200)    /* 5ms */
#define LWB_CONF_T_CONT                 (RTIMER_SECOND_HF / 125)    /* 8ms */
#define LWB_CONF_TX_CNT_SCHED           3
#define LWB_CONF_TX_CNT_DATA            3
#define LWB_CONF_T_SCHED2_START         RTIMER_SECOND_HF

#define DCSTAT_CONF_ON                  1

/* debug config */
#define DEBUG_PRINT_CONF_LEVEL          DEBUG_PRINT_LVL_INFO
#define DEBUG_PRINT_CONF_USE_RINGBUFFER 1
#define DEBUG_PRINT_CONF_BUFFER_SIZE    512  /* debug print buffer size in b */
#define DEBUG_PRINT_CONF_MSG_LEN        100 /* max debug msg length per line */
//#define DEBUG_CONF_ISR_INDICATOR      1           /* indicate CPU activity */
//#define DEBUG_CONF_ISR_IND_PIN        COM_MCU_INT2      /* show interrupts */


#endif /* __CONFIG_H__ */
