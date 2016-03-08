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

#define HOST_ID    1

#define NODE_ID                         2
#define LWB_CONF_TASK_ACT_PIN           PORT2, PIN6
#define DEBUG_PRINT_TASK_ACT_PIN        PORT2, PIN6
#define APP_TASK_ACT_PIN                PORT2, PIN6

#define FRAM_CONF_ON                    1
#define BOLT_CONF_ON                    0

/* LWB configuration */
#define LWB_VERSION                     2    /* use the modified LWB */
#define LWB_SCHED_BURST                 /* use the 'burst' scheduler */
#define LWB_CONF_RELAY_ONLY             0
#define LWB_CONF_USE_XMEM               1
#define LWB_CONF_USE_LF_FOR_WAKEUP      1
#define LWB_CONF_SCHED_PERIOD_IDLE      5        /* define the period length */
#define LWB_CONF_MAX_DATA_SLOTS         20
#define LWB_CONF_MAX_N_STREAMS          5
#define LWB_CONF_MAX_PACKET_LEN         128
#define LWB_CONF_MAX_DATA_PKT_LEN       (LWB_CONF_MAX_PACKET_LEN)
#define LWB_CONF_SCHED_SACK_BUFFER_SIZE LWB_CONF_MAX_N_STREAMS
#define LWB_CONF_T_SCHED2_START         0       /* omit the 2nd schedule */
#if HOST_ID == NODE_ID
  #define LWB_CONF_IN_BUFFER_SIZE       LWB_CONF_MAX_DATA_SLOTS
  #define LWB_CONF_OUT_BUFFER_SIZE      1
#else
  #define LWB_CONF_IN_BUFFER_SIZE       1
  #define LWB_CONF_OUT_BUFFER_SIZE      LWB_CONF_MAX_DATA_SLOTS
#endif
#define LWB_CONF_TX_CNT_DATA            2
#define LWB_CONF_MAX_HOPS               3
#define LWB_CONF_T_SCHED                (RTIMER_SECOND_HF / 100) /* 10ms */
#define LWB_CONF_T_CONT                 (RTIMER_SECOND_HF / 200) /* 5ms */
#define LWB_CONF_T_GAP                  (RTIMER_SECOND_HF / 500) /* 2ms */
#define LWB_CONF_SCHED_STREAM_REMOVAL_THRES     5

/* debug config */
/* adjust this accordingly! */
#define DEBUG_CONF_STACK_GUARD          (SRAM_END - 0x01ff)
#define DEBUG_PRINT_CONF_LEVEL          DEBUG_PRINT_LVL_INFO

#endif /* __CONFIG_H__ */
