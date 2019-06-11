/*
 * Copyright (c) 2019, Swiss Federal Institute of Technology (ETH Zurich).
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

/* current FW version (A = major, B = minor, C = patchlevel)  */
#define FW_VERSION      10012           /* in decimal ABBCC */
#define FW_NAME         "elwb-dev"      /* name of the application (8 bytes) */

/*

Revision History
----------------

Version 1.0013 (development):
- bugfix: minor bugfix in Glossy (slot estimation was skipped in some cases)

Version 1.0012 (2019-03-25):
- change: minor adjustment to load_config()
- feature: new command added to set the node ID (to 'recover' nodes that loose
           their configuration)

Version 1.0011 (2019-02-26):
- change: CRC check added to HOST eLWB task before passing message to BOLT when
          ELWB_CONF_WRITE_TO_BOLT is enabled (no DACK => source will resend)

Version 1.0010 (2019-02-12):
- bugfix: issue fixed where the network traffic stalls for some time when at
          least two nodes with comparable links to the host node send (contend)
          during the contention slot

Version 1.0009 (2019-02-08):
- feature: new define ELWB_CONF_CONT_TH added to accumulate / hold back data
           packets in the TX queue for more efficient use of the eLWB
- feature: random backoff mechanism added to improve contention slot
           performance in case many nodes contend
- change: Glossy initiator retransmits if RX fails (e.g. due to CRC error).
- change: max. relay count added to Glossy interface, is now reset together
          with other stats
- change: Unit for Glossy flood duration and time to first RX changed from
          ticks to us.
- bugfix: Glossy retransmission timeout didn't work as expected + parameters
          tuned to better align pulses.
- bugfix: sequence number always zero, was not defined as static
- change: TX queue for source nodes increased from 6 to 8 messages for the case
          when the FRAM is not used

Version 1.0008 (2018-12-21):
- feature: if node ID cannot be retrieved from flash memory, a 'random' ID will
           be generated based on the unique identifier
- change: eLWB code simplified, now uses only 1 timer for all wakeups (LF)
- change: feature with primary and secondary RF channel removed (unused)
- change: Glossy stats adjusted, now uses lf_hw only for timestamping
- change: in Glossy, NS_TO_RTIMER_HF replaced by NS_TO_RTIMER_HF_32

Version 1.0007 (2018-11-30):
- change: debug print task simplified
- change: undefined ISRs and new define 'DEBUG' added
- change: minor improvements to error handling

Version 1.0006 (2018-09-28):
- feature: new events EVENT_CC430_CORRUPTED_SCHEDULE and
           EVENT_CC430_CORRUPTED_CONFIG added
- change: check added for TI device ID (ensures that the code has been compiled
          for the target platform)
- bugfix: fixed two potential issues, one where a CRC value of 0xffff would be
          regarded as invalid in nvcfg_load() and another where an "odd" block
          size would lead to a corrupted block at the flash segment boundary
- change: flash content verification added to nvcfg_save

Version 1.0005 (2018-09-26):
- bugfix: error in DC stats fixed
- change: defines for Glossy adjusted (enums replaced by defines, now only
          with_sync or without_sync) + parameter for constant setup time added
          (GLOSSY_CONF_SETUPTIME_WITH_SYNC)
- change: RTIMER_CONF_LF_UPDATE_INT disabled
- change: perform a SW POR if reset source unknown (bugfix for CC430F5137)
- change: issue with ADC on CC430F5137 fixed
- change: include files and platform.h adjusted to support both the CC430F5137
          and CC430F5147 (use MCU=... flag for compilation)

Version 1.0004 (2018-09-11):
- bugfix: value for drift in LWB health was always zero
- change: in rf1a, timeouts added to WAIT_UNTIL macros to avoid potential
          lockup conditions + cleanup (rf1a-core.h removed)

Version 1.0003 (2018-07-23):
- change: max #packets that are read from the BOLT queue per round limited
- change: more checks added for xmem_read() in elwb.c
- change: limit for max #slots per request added to scheduler
- change: jump into BSL if reset flag SWBOR detected

Version 1.0002 (2018-07-03);
- change: rtimer period changed to 32-bit to reduce runtime and memory usage
- bugfix: potential invalid timestamp issue fixed, elwb_get_time() now returns
          the LF timestamp, last_synced_hf removed
- change: clear pending radio interrupts after module has entered sleep state
- change: DMA_CONF_ENABLE define added to explicitly exclude DMA code; in BOLT
          code for timereq callback, DMA is only used if BOLT_CONF_USE_DMA is 1
- bugfix: wrong guard time used for slot duration for schedule reception
- change: calculation of t_to_rx stats value improved
- change: eLWB buffer size increased
- change: FRAM wakeup removed from eLWB, moved into platform.h (FRAM_WAKEUP)

Version 1.0001 (2018-06-27):
- feature: new flag ELWB_CONF_PREEMPTION added to enable task preemption
- change: debug.c added
- change: xmem_wakeup() replaced by a simple control pin toggling
- change: sanitiy check for n_slots added on source nodes to prevent potential
          memory overflow
- change: new defines/config added to watchdog.h
- feature: ELWB_CONF_SCHED_CRC define added, by default a CRC is appended to 
           the schedule, source nodes check the CRC
- change: minor changes on how the stats about Glossy are collected and new
          define GLOSSY_CONF_ALWAYS_SAMPLE_NOISE introduced in glossy.h
- bugfix: due to invalid pin configuration in BEFORE_DEEPSLEEP, an extra zero
          character was transmitted over UART and the output low configuration
          of the RXD/TXD pins could cause a higher current drain

Version 1.0 (release candidate, 2018-06-25):
- change: elwb_sched_register_nodes() added to make better use of the define
          ELWB_CONF_SCHED_NODE_LIST
- bugfix: issue with D-ACK payload misalignment solved
- change: handling of t_ref on source node when schedule missed
- change: elwb_process removed (not needed as a separate process)
- bugfix: xmem_task removed, reading/writing from/to the FRAM is now handled
          within the interrupt routine (will not work with FRAM_CONF_USE_DMA!)
- bugfix: issue with rt->time for SEND_SCHEDULE on host node fixed
- bugfix: check for node ID in D-ACK slot removed and requeue all packets
          if D-ACK missed (function fifo_elem_addr_rel introduced)
- change: elwb_stats_t struct adjusted, pkt_snd and pkt_ack added, rx_total and
          req_cnt removed
- change: ELWB_CONF_T_DACK introduced to increase the slot size for data ACKs
- change: default parameters moved from config.h into elwb.h, unused defines
          removed and more comments added / minor cleanup

Version 0.9 (bugfix update, 2018-06-20):
- change: broadcast reset command doesn't affect the host node anymore
- change: rt->time not anymore overwritten with HF timestamp
- bugfix: max. number of nodes and data slots reduced to 40 to be compatible 
          with v0.7 source nodes (prevent overflow of schedule struct)
- bugfix: several buffer overflows fixed: in elwb_sched_compute(),
          lwb_schedule_t (length of slot field), lwb_sched_compress() and
          lwb_sched_uncompress(), mostly because the default value for
          LWB_CONF_MAX_DATA_SLOTS was used
- change: guard time (HF) reduced to 250us
- change: D-ACK slot only if there is no contention slot (no D-ACK if host node
          sends data!)
- change: DEBUG_CONF_ISR_INDICATOR define removed
- change: in glossy.c only read noise floor in schedule slot (with sync)

Version 0.8 (2018-06-19):
- change: debug buffer size increased to 512 bytes
- change: new fields added to LWB health message (load, drift and unsynced_cnt)
- change: UART_CONF_RX_INTERRUPT define added, serial_line process and UART
          receive interrupt disabled by default
- change: scheduler adjusted: new flag ELWB_CONF_SCHED_FAIR added, enables
          'fair' slot allocation (before it was first come, first serve =>
          lower node IDs had higher priority)
- feature: scheduler: nodes that have not transmitted any data for
           ELWB_CONF_SCHED_NODE_TIMEOUT seconds are now removed from the list
- bugfix: potential divide by zero fixed for health_msg_period
- change: minimal allowed round period changed to ELWB_CONF_SCHED_PERIOD_MIN
          seconds, max. period changed to ELWB_SCHED_MAX_PERIOD, max. number of
          allowed slots per round and nodes increased to 50
- feature: events EVENT_CC430_NODE_ADDED and EVENT_CC430_NODE_REMOVE and
           command CMD_CC430_ADD_NODE added
- change: code restucturing and some cleanup in elwb.c and sched-elwb-dyn.c, 
          variable renaming for consistency, prefix of all defines and function
          names renamed from lwb_/LWB_ to elwb_/ELWB_
- bugfix: event value for event_write() increased from 16 to 32 bits
- change: parameter LWB_CONF_MAX_N_STREAMS increased to 40 and
          LWB_CONF_OUT_BUFFER_SIZE for source increased to 5
- change: debug_print_buffer_put() added, allows scheduler to print all node
          IDs
- change: preprocess for host set to 100ms (50 seems too tight when reading
          many packets from BOLT)

Version 0.7 (bugfix update, 2018-06-06):
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

Version 0.6 (2018-06-04):
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
