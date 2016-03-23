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

#define FLOCKLAB
#define HOST_ID    22

#ifndef FLOCKLAB
  //#define NODE_ID                       200
  #define LWB_CONF_TASK_ACT_PIN         PORT2, PIN5
  #define DEBUG_PRINT_TASK_ACT_PIN      PORT2, PIN5
  #define APP_TASK_ACT_PIN              PORT2, PIN5
  #define RF_CONF_TX_POWER              RF1A_TX_POWER_MINUS_12_dBm
#else
  #define LWB_CONF_TASK_ACT_PIN         FLOCKLAB_LED3
  #define DEBUG_PRINT_TASK_ACT_PIN      FLOCKLAB_LED3
  #define APP_TASK_ACT_PIN              FLOCKLAB_LED3
  #define RF_CONF_TX_POWER              RF1A_TX_POWER_PLUS_10_dBm  
#endif /* FLOCKLAB */

#define FRAM_CONF_ON                    0
#define BOLT_CONF_ON                    0

/* LWB configuration */

#define LWB_VERSION                     0       /* use custom version */

//#define LWB_CONF_RANDOM_DELAY_MAX       400
#define LWB_CONF_OFFSET_NODE_ID         16
#define GLOSSY_CONF_RETRANSMISSION_TIMEOUT  0

#define LWB_CONF_STREAM_EXTRA_DATA_LEN  0
#define LWB_CONF_TX_CNT_DATA            1
#define LWB_CONF_MAX_HOPS               3
#define LWB_CONF_T_SCHED                (RTIMER_SECOND_HF / 100) /* 10ms */
#define LWB_CONF_T_CONT                 (RTIMER_SECOND_HF / 200) /* 5ms */
#define LWB_CONF_T_GAP                  (RTIMER_SECOND_HF / 500) /* 2ms */
#define LWB_CONF_MAX_DATA_PKT_LEN       180
#define LWB_CONF_MAX_PKT_LEN            180

#define RF_CONF_TX_CH                   10

/* debug config */
#define DEBUG_CONF_STACK_GUARD          (SRAM_END - 0x01ff)
#define DEBUG_PRINT_CONF_LEVEL          DEBUG_PRINT_LVL_INFO

#endif /* __CONFIG_H__ */
