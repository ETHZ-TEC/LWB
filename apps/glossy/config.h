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
#define HOST_ID    1

#ifdef FLOCKLAB
  /* set the highest antenna gain if the program runs on FlockLAB */
  #define RF_CONF_TX_POWER              RF1A_TX_POWER_PLUS_10_dBm
  #define RF_CONF_TX_CH                 5                 /* approx. 869 MHz */
  #define RF_CONF_MAX_PKT_LEN           128
  //#define DEBUG_PRINT_TASK_ACT_PIN      FLOCKLAB_LED3
  //#define APP_TASK_ACT_PIN              FLOCKLAB_LED3
#else
  /* only define a node ID if FlockLAB is not used (FlockLAB automatically 
   * assigns node IDs); select an ID other than HOST_ID to compile the code 
   * for a source node */
  #define RF_CONF_TX_CH                 3               /* approx. 868.6 MHz */
  #define NODE_ID                       1
  #define RF_CONF_MAX_PKT_LEN           128
  #define APP_TASK_ACT_PIN              PORT2, PIN1
  #define RF_CONF_TX_POWER              RF1A_TX_POWER_0_dBm
#endif /* FLOCKLAB */

/* LWB configuration */
#define GLOSSY_PERIOD                   (RTIMER_SECOND_HF / 10)
#define GLOSSY_RTIMER_ID                RTIMER_HF_0
#define GLOSSY_T_SLOT                   (RTIMER_SECOND_HF / 20)
#define GLOSSY_T_GUARD                  (RTIMER_SECOND_HF / 1000)     /* 1ms */
#define GLOSSY_PAYLOAD_LEN              8
#define GLOSSY_N_TX                     3
#define GLOSSY_CONF_RETRANSMISSION_TIMEOUT      1

/* to align the host and source nodes */
#define GLOSSY_REF_OFS                  (RTIMER_SECOND_HF / 800)

/* debug config */
#define DEBUG_PRINT_CONF_LEVEL          DEBUG_PRINT_LVL_INFO

#endif /* __CONFIG_H__ */
