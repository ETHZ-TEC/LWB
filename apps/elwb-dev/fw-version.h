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

#ifndef __FW_VERSION_H__
#define __FW_VERSION_H__

/* current FW version (8 bits for major version, 8 bits for minor) */
#define FW_VERSION      0x0007
#define FW_NAME         "elwb-dev"     /* name of the application (8 bytes) */

/*

Revision History
----------------

Version 0.7 (--- IN DEVELOPMENT | bugfixes only ---):
- bugfix: on the source node during a data round, there was no checked whether
          a packet was actually received before applying the node_id filter
- bugfix: TX power was set to the minimum after a reset if it hasn't been set 
          before via a command
- change: debug prints optimized
- change: etimer_process disabled and print_processes removed from
          contiki-cc430-main.c
- bugfix: order of defines in config.h is critical, FRAM_CONF_ON used before
          defining will compile but yield unexpected results
- bugfix: packet RX count was still not just over the last health period
- bugfix: TX power in com health was constant instead of the actual value
- change: event_write() optimized to use the global struct for the message
          (reduces stack usage)
- bugfix: first event message ('time adjusted') had an invalid timestamp
- change: NODE_ID is now defined differently in contiki-conf.h, node_id now 
          exists as a global variable in any case (host and source)
- change: preprocess time reverted back to 50ms (is sufficient)
- bugfix: when changing the round period, the timestamp was unnecessarily 
          'adjusted' (i.e. a difference of more than 5s was detected)
- change: defines TIMESYNC_HOST_RCV_UTC and TIMESYNC_INTERRUPT_BASED removed
          (deprecated), IS_HOST macro added

Version 0.6 (works with FRAM chip only, 2018-06-04):
- bugfix: timing issue on the host node resolved where the time to compute the 
          new schedule was sometimes too short
- change: schedule recomputation at beginning of round removed
- bugfix: in scheduler time management (period added twice, when schedule
          was recomputed at the beginning of a round)
- feature: firmware updater added, enables OTA updates if an FRAM chip is 
           installed (still in experimental stage!)
- change: stack usage reduced by utilizing global packet buffers
- change: preprocess task set to 100ms before round
- feature: node ID is now stored in NVMEM config
- feature: new message type dpp_com_response_t and command added to read out
           an arbitrary memory location (for debugging)
- feature: new command and persistent config option added to enable/disable
           LEDs (CMD_CC430_SET_DBG_FLAGS)
- feature: new command CMD_CC430_SET_TX_POWER added + stored in NVMEM config
- feature: data ack mechanism added, allows source nodes to retransmit lost
           packets to the host (only works for pkts from SRC -> HOST)
- change: to reduce the memory usage, it has to be specified at compile time 
          whether the node is a host or source node
- bugfix: radio SNR value in dpp_com_health_t for host node corrected

Version 0.5 (2018-05-09):
- feature: event msg is generated whenever the local time was adjusted
- feature: separate duty cycle stats for radio RX and TX
- change: dpp_com_health_t adjusted and new type DPP_MSG_TYPE_LWB_HEALTH added

Version 0.4 (2018-05-02):
- feature: automatically switch RF channel if bootstrap times out
- feature: host prints list of host nodes after a data dissemination round
- change: now only 2 guard times, one for HF and one for LF timer
- change: timestamp is only adjusted if the jump is > 5 seconds
- change: schedule slot, contention slot and guard times increased
- change: LWB_PERIOD_T_DATA is now calculated dynamically in the scheduler

Version 0.3 (2018-04-30):
- bugfix: host would not reset streams to inactive after a data round

Version 0.2 (2018-04-27):
- feature: UTC timestamp now distributed over network (LWB schedule)
- bugfix: node info msg from host sometimes had an invalid timestamp (1970)

Version 0.1 (2018-04-06):
- initial version (based on code in lwb-dev)

*/

#endif /* __FW_VERSION_H__ */
