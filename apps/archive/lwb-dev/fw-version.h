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
#define FW_VERSION      0x010c
#define FW_NAME         "lwb-dev"     /* name of the application (8 bytes) */

/*

Revision History
----------------

Changes in v1.12 (2017-04-12):
- changes: see git changelog of core, mcu and platform files

Changes in v1.11 (2016-12-22):
- change:  clock dividers in LWB_BEFORE_DEEPSLEEP now unchanged (yields a
           faster MCLK speed of ~6.2MHz between rounds for the ISR execution
           and still doesn't exceed the max. allowed speed due to errata PMM11)
- change:  __delay_cycles(100) removed from LWB_AFTER_DEEPSLEEP since the DIVx
           is not changed anymore
- change:  increasing number of slots (if no packet received from a node)
           removed from sched-static.c
- change:  SEND_HEALTH_DATA #ifdef region in source-node.c changed + stream
           allocation slightly adjusted
- bugfix:  pointer aliasing problem in lwb_in_buffer_put() fixed

Changes in v1.10 (2016-12-09):
- change:  PIN_XOR replaced by PIN_SET/CLR for RTIMER_CONF_LF_UPDATE_LED_ON and
           GLOSSY_START_PIN
- change:  glossy_snr in lwb.c moved into the lwb_statistics_t struct
- change:  guard time for LWB state QUASI_SYNCED changed
- change:  in glossy.c, disabling of LWB timer during RX removed (causes
           slot overruns)
- change:  LWB_CONF_DATA_ACK disabled
- change:  watchdog re-enabled
- feature: FLOCKLAB_SRC_NODE added to compile code for flocklab
- bugfix:  correction in the drift compensation for lwb
- bugfix:  errata CPU46 fix added to main()
- bugfix:  schedule.period replaced by period_last in lwb_thread_src() when
           1st schedule missed

Changes in v1.09 (2016-11-29):
- change:  severity level of some ERROR debug prints changed in several files
           (+ comment added to debug-prints.h)
- change:  nvcfg moved into core folder to make it available for all apps
- change:  t_proc_max check added to lwb-custom.c for source node
- change:  BOLT_CONF_TIMEREQ_HF_MODE and BOLT_CONF_TIMEREQ_CCR definitions
           added to bolt.h, PIN_SEL replaced by PIN_MAP in bolt_init()
- change:  LWB_CONF_MAX_CLOCK_DEV in lwb.h is now in ppm

Changes in v1.08 (2016-11-24):
- change:  condition for IES toggling removed for falling edge of IFG9 in rf
           core ISR
- change:  SVS is now disabled by default
- change:  default value for LWB_CONF_T_SCHED2_START increased by 20ms
- change:  GLOSSY_STARTED indicator moved to the end of glossy_start() routine
- change:  GDO1 changed to RSSI_VALID signal
- change:  in glossy_start: wait for RSSI_VALID signal instead of const delay
- change:  for the falling edge of IFG9 in rf1a core isr, the default case is
           not handled with rf1a_cb_rx_tx_error() anymore
- change:  error handling for rf core IFG9 (rising) removed for default case
- change:  calculations in estimate_T_slot() in glossy.c optimized, using
           32-bit calculations for slot measurements; reduces tx_started ISR
           execution time from ~210us to ~45us
- change:  RF1A5 errata workaround re-added to IFG7 and IFG8 in rf core ISR
- feature: DEBUG_CONF_ISR_INDICATOR added to show interrupt activity
- feature: dcstat added, a minimal implementation of ENERGEST for CPU and RF
           duty cycle (uses 2.9kB less ROM), replaces ENERGEST by default
- feature: DEBUG_ISR_TRAPS_ENABLE define added to (de-)activate interrupt traps
- bugfix:  LED indicator in rtimer ISR moved into update/OVF case
- bugfix:  rf1a_buffer size increased by 3 bytes to account for length + status

Changes in v1.07 (2016-11-18):
- change:  default duration of contention slot increased from 5 to 8ms
- change:  falling edge of IFG9 in rf core ISR adjusted
- change:  code in lwb-custom.c/lwb.c for src node slightly rearranged
- change:  drift variable added to statistics struct; not reset when entering
           BOOTSTRAP mode
- change:  lwb_get_timestamp() adjusted
- bugfix:  estimation of t_ref (in case the schedule is missed) corrected

Changes in v1.06 (2016-11-15):
- change:  in clock.c, all debug code removed from UNMI ISR
- change:  default target in Makefile.include set to DPP
- change:  elapsed time for sending health packets is now based on local clock
- change:  #dropped packets (queue full) is now continuously increasing, not
           a delta
- change:  LWB_CONF_T_SCHED2_START set in config.h to 800ms
- change:  LWB data ACKs activated
- change:  LWB_CONF_MAX_HOPS set to 3 to reduce data slot length
- change:  INVERT_INTERRUPT_EDGES in RF interrupt for BIT9 moved down to the
           case statements
- change:  message types for APP_FW consolidated

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
