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

//#define FLOCKLAB
#define HOST_ID    2                              


#ifdef FLOCKLAB
  #define RF_CONF_TX_POWER              RF1A_TX_POWER_PLUS_10_dBm   // set the antenna gain
  #define FRAM_CONF_ON                  0                           // make sure external FRAM is disabled when using FlockLAB
  #define BOLT_CONF_ON                  0
#else
  #define NODE_ID                       2                           // only define a node ID if FlockLAB is not used
#endif // FLOCKLAB

// LWB
#define LWB_SCHED_STATIC                                            // use the static scheduler
#define LWB_CONF_STREAM_EXTRA_DATA_LEN  0                           // set the length (in bytes) of the additional stream information

#define RF_CONF_TX_CH                   5               // select the channel that is best suited for the test/deployment environment

#define DEBUG_PRINT_CONF_NUM_MSG        5
#define DEBUG_PRINT_CONF_LEVEL          DEBUG_PRINT_LVL_INFO  // select the debug level, must be of type debug_level_t


#endif /* __CONFIG_H__ */
