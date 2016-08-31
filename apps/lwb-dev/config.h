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

#define NODE_ID                         1  //20050
#define HOST_ID                         1

#define SEND_HEALTH_DATA                1
/* rooftop: dozer is on 868.875 MHz (CH12) -> choose CH10 */
#define RF_CONF_TX_CH                   2
#define ENERGEST_CONF_ON                1
#define LWB_CONF_SCHED_PERIOD_IDLE      10       /* define the period length */
#define RF_CONF_TX_POWER                RF1A_TX_POWER_0_dBm
#define LWB_CONF_DATA_ACK               0                  /* use data ACKs? */

/* LWB configuration */
#define LWB_SCHED_STATIC                         /* use the static scheduler */
#define LWB_VERSION                     0          /* use the custom version */
#define LWB_CONF_OUT_BUFFER_SIZE        4
#define LWB_CONF_IN_BUFFER_SIZE         10
#define LWB_CONF_MAX_PKT_LEN            63
#define LWB_CONF_MAX_DATA_PKT_LEN       62
#define LWB_CONF_USE_LF_FOR_WAKEUP      1
#define LWB_CONF_TASK_ACT_PIN           COM_MCU_INT2
#define LWB_CONF_MAX_N_STREAMS          10      /* to keep memory usage down */

/* constant clock offset for timesync */
#define LWB_CLOCK_OFS                   -1200

/* BOLT config */
#define BOLT_CONF_MAX_MSG_LEN           64
#define BOLT_CONF_TIMEREQ_ENABLE        1


/* debug config */
#define DEBUG_PRINT_CONF_LEVEL          DEBUG_PRINT_LVL_INFO
#ifndef DEBUG_PRINT_CONF_NUM_MSG
#define DEBUG_PRINT_CONF_NUM_MSG        8
#endif /* DEBUG_PRINT_CONF_NUM_MSG */
//#define DEBUG_PRINT_CONF_TASK_ACT_PIN   COM_MCU_INT2
//#define APP_TASK_ACT_PIN                COM_MCU_INT2

/* use port2 interrupt to print out debug information? */
#define DEBUG_PORT2_INT                 0



#endif /* __CONFIG_H__ */
