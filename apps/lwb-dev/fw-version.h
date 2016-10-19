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

#ifndef __FW_VERSION_H__
#define __FW_VERSION_H__

/* current FW version (8 bits for major version, 8 bits for minor) */
#define FW_VERSION      0x0105
#define FW_NAME         "lwb-dev"     /* name of the application (8 bytes) */

/*

Feature Suggestions:
- include an accuracy indicator with the generation_time (depending on the
  time that has elapsed since the last sync)



Revision History
----------------

Changes in v1.05 (2016-10-19):
- change:  debug trap removed in lwb-custom.c
- change:  glossy start/stop indication changed from PIN_SET/CLR to PIN_XOR
- change:  RF_CONF_ON default value added to platform.h
- change:  Glossy constants for slot estimation in rf1a config changed
- change:  LWB HF timer interrupt is now disabled when RX starts in Glossy
- change:  rf1a_go_to_idle() now executed before reconfiguring the registers in
           glossy_start()
- change:  INVERT_INTERRUPT_EDGES() in rf1a moved to beginning for IFG9 ISR
- change:  rf1a_reconfig_after_sleep() introduced to reconfigure the lost reg.
           contents when radio awakes from sleep mode (used in glossy_start)
- change:  glossy_set_tx_pwr() removed
- feature: host node sends a node info message after reset
- bugfix:  in source node code: message length for packet forwarding from
           BOLT to LWB corrected + target_id is now used
- bugfix:  errata RF1A5 workaround removed, seems to cause more problems
           (except for falling edge of RFIFG9, if not in RX or TX)
- bugfix:  endless loop (wait for RF_RDY bit) fixed in glossy_start()

Changes in v1.04 (2016-10-10):
- change:  wait for RF_RDY added to glossy_start()
- change:  message_t structure size increased by 2 bytes to include the CRC;
           consequently, the sizeof operator will now return MSG_PKT_LEN
- bugfix:  TEST0 register configuration fixed in rf1a-core.h (typo)
- bugfix:  vacant memory access violation due to inconsistent message_t size
           handling fixed; mainly concerned lwb_rcv_pkt() and lwb_send_pkt()

Changes in v1.03 (2016-10-07):
- change:  component ID introduced (added to log events and node info packets)
- change:  limiter added for command values for TXPWR and round period
- change:  more debugging info added to UNMI ISR
- change:  workaround for errata RF1A5 applied to radio core ISR
- change:  default RF_CONF_MAX_PKT_LEN changed in lwb.h
- feature: SVS_CONF_ON define added (default is 0)
- bugfix:  delay added to wake-up routine after deepsleep to increase stability

Changes in v1.02 (2016-09-30):
- change:  software POR added to DEBUG_PRINT_FATAL()
- change:  node info packet is only generated once the node is synced
- feature: lwb_get_timestamp() added, delivers the current time or an estimate
           in case the node is not synchronized
- feature: LOG_POS() added to log the current position (line, file) in the code
- feature: SVS_ENABLE macro added
- feature: reset command added to comm_cmd_type_t
- feature: glossy error counter added
- bugfix:  in log_generic
- bugfix:  LWB_BEFORE_DEEPSLEEP() and LWB_AFTER_DEEPSLEEP() adjusted to account
           for various errata (PMM11, PMM12, UCS11)

Changes in v1.01 (2016-09-29):
- change:  watchdog is now configured at the very beginning of the program
- change:  node_info_t struct adjusted
- change:  event 'reset' replaced by a node_info_t packet
- cleanup: utils.c created, contains general helper functions
- cleanup: print_device_info() simplified
- feature: makefile extended to include the compiler macros COMPILE_TIME
           (UNIX timestamp) and GIT_HEADREV_SHA (git head revision SHA hash)
- feature: RTIMER_CONF_LF_UPDATE_LED_ON added for debugging
- bugfix:  send node health on host (wrong pointer)

Changes in v1.0 (2016-09-27):
- change:  LWB_CONF_T_GUARD is now only used for ack/data/contention slots,
           whereas LWB_CONF_T_GUARD_1 to 3 are only used for schedule reception
- change:  LWB bootstrap loop adjusted + permission to participate in round
           changed to state LWB_STATE_SYNCED only
- cleanup: send_pkt and send_msg consolidated (same for src and host node)
- cleanup: everything regarding node health outsourced into node-health.c
- cleanup: LWB states renamed and lwb_update_sync_state() function added
- feature: separate sequence number for BOLT and LWB packets
- feature: logging system added (log.c, log.h, log-events.h)
- feature: host health message period now adjustable
- feature: new message type (struct node_info_t) added
- feature: event 'reset' with reset cause is not generated
- feature: nvcfg added to keep a few bytes in non-volatile memory (flash)
- feature: node replies with "config changed" message to a command
- bugfix:  host did not forward broadcast packets
- bugfix:  t_guard was still used for timeout instead of LWB_CONF_T_GUARD

*/



#endif /* __FW_VERSION_H__ */
