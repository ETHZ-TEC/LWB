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

#define HOST_ID    1

#define NODE_ID                         20050
#define RF_CONF_TX_CH                   5       /* approx. 869 MHz */   
#define RF_CONF_TX_POWER                RF1A_TX_POWER_0_dBm 

/* Contiki config */
#define ENERGEST_CONF_ON                1
                                                       
/* LWB configuration */
#define LWB_SCHED_STATIC                         /* use the static scheduler */
#define LWB_CONF_SCHED_PERIOD_IDLE      10       /* define the period length */
#define LWB_CONF_OUT_BUFFER_SIZE        10
#define LWB_CONF_IN_BUFFER_SIZE         10
#define LWB_CONF_MAX_PKT_LEN            63
#define LWB_CONF_MAX_DATA_PKT_LEN       (31 + LWB_DATA_PKT_HEADER_LEN)
#define LWB_CONF_USE_LF_FOR_WAKEUP      1

#define LWB_STREAM_ID_STATUS_MSG        1
/* constant clock offset for timesync */
#define LWB_CLOCK_OFS                   -1200       

#define BOLT_CONF_MAX_MSG_LEN           48
#define BOLT_CONF_TIMEREQ_ENABLE        1

/* debug config */
#define DEBUG_PRINT_CONF_LEVEL          DEBUG_PRINT_LVL_INFO

/* 
 * INCLUDES
 */
#include "packet.h"                    /* packet structure and message types */

/* 
 * GLOBALS
 */
extern void send_msg(message_type_t type, uint8_t* data, uint8_t len);
/* the static scheduler implements the following function: */
extern void lwb_sched_set_period(uint16_t period);

#endif /* __CONFIG_H__ */
