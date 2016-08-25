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

/**
 * @brief Low-Power Wireless Bus Test Application
 * 
 * A simple range test application. Each source node sends some status data
 * (RSSI, battery voltage, temperature, ...) to the host in each round.
 */


#include "contiki.h"
#include "platform.h"
#include "log.h"
#include "flash-logger.h"
#include "fw.h"

/*---------------------------------------------------------------------------*/
#ifdef FLOCKLAB
#warning "---------------------- COMPILED FOR FLOCKLAB ----------------------"
#endif /* FLOCKLAB */
/*---------------------------------------------------------------------------*/
#ifdef APP_TASK_ACT_PIN
#define TASK_ACTIVE             PIN_SET(APP_TASK_ACT_PIN)
#define TASK_SUSPENDED          PIN_CLR(APP_TASK_ACT_PIN)
#else
#define TASK_ACTIVE
#define TASK_SUSPENDED
#endif /* APP_TASK_ACT_PIN */
/*---------------------------------------------------------------------------*/
uint16_t program_state  = 0;
static uint16_t  seq_no = 0;
static message_t msg_buffer;
static uint8_t   bolt_buffer[BOLT_CONF_MAX_MSG_LEN];
/*---------------------------------------------------------------------------*/
/* FUNCTIONS */
void
send_msg(message_type_t type, uint8_t* data, uint8_t len) 
{
  /* send the message over the LWB */  
  rtimer_clock_t round_start; 
  uint64_t       lwb_time_seconds = lwb_get_time(&round_start);
  // convert to microseconds
  round_start = (rtimer_now_hf() - round_start) * 1000000 / SMCLK_SPEED;
  msg_buffer.header.device_id       = node_id;
  msg_buffer.header.type            = type;
  msg_buffer.header.generation_time = lwb_time_seconds * 1000000 +
                                      round_start;
  msg_buffer.header.payload_len     = len;
  msg_buffer.header.seqnr           = seq_no++;
  
  if(data) {
    memcpy((char*)msg_buffer.payload, data, len);
  }

  if(!lwb_send_pkt(LWB_RECIPIENT_SINKS, LWB_STREAM_ID_STATUS_MSG,
                   (uint8_t*)&msg_buffer + 2, 
                   MSG_HDR_LEN + msg_buffer.header.payload_len - 2)) {
    DEBUG_PRINT_WARNING("message #%u dropped (queue full?)", seq_no - 1);
  } else {
    DEBUG_PRINT_INFO("message #%u added to TX queue", seq_no - 1);   
  }
}
/*---------------------------------------------------------------------------*/
health_msg_t* 
get_node_health(void) 
{
  static int16_t temp = 0;
  static health_msg_t health;
  static rtimer_clock_t last_energest_rst = 0;
  
  while(REFCTL0 & REFGENBUSY);
  REFCTL0 |= REFON;
  while(REFCTL0 & REFGENBUSY);
  //__delay_cycles(MCLK_SPEED / 25000);                /* let REF settle */
  
  temp = (temp + adc_get_temp()) / 2;    /* moving average (LP filter) */
  health.temp = temp;
  health.vcc  = adc_get_vcc();
  REFCTL0 &= ~REFON;             /* shut down REF module to save power */

  glossy_get_rssi(health.rssi);
  health.snr       = glossy_get_snr();
  health.rx_cnt    = glossy_get_n_pkts_crcok();
  health.per       = glossy_get_per();
  health.n_rx_hops = glossy_get_n_rx() | (glossy_get_relay_cnt() << 4);
  rtimer_clock_t now = rtimer_now_lf();
  health.cpu_dc    = (uint16_t)(energest_type_time(ENERGEST_TYPE_CPU) *
                                1000 / (now - last_energest_rst));
  health.rf_dc     = (uint16_t)((energest_type_time(ENERGEST_TYPE_TRANSMIT) + 
                                energest_type_time(ENERGEST_TYPE_LISTEN)) * 
                                1000 / (now - last_energest_rst));
  // reset values
  last_energest_rst  = now;
  energest_type_set(ENERGEST_TYPE_CPU, 0);        
  energest_type_set(ENERGEST_TYPE_TRANSMIT, 0);
  energest_type_set(ENERGEST_TYPE_LISTEN, 0);
  
  return &health;
}
/*---------------------------------------------------------------------------*/
PROCESS(app_process, "Application Task");
AUTOSTART_PROCESSES(&app_process);
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(app_process, ev, data) 
{   
  static flash_logger_block_t stats;
  
  PROCESS_BEGIN();
            
#if LWB_CONF_USE_LF_FOR_WAKEUP
  SVS_DISABLE;
#endif /* LWB_CONF_USE_LF_FOR_WAKEUP */
  /* all other necessary initialization is done in contiki-cc430-main.c */
    
#if HOST_ID != NODE_ID
  /* SOURCE node only */
  /* init the ADC */
  adc_init();
  REFCTL0 &= ~REFON;             /* shut down REF module to save power */
  
 #if DEBUG_PORT2_INT
  /* enable ISR for debugging! */
  PIN_IES_RISING(BOLT_CONF_IND_PIN);
  PIN_CLR_IFG(BOLT_CONF_IND_PIN);
  PIN_INT_EN(BOLT_CONF_IND_PIN);
 #endif /* DEBUG_PORT2_INT */
#endif /* HOST_ID != NODE_ID */

 #if FW_CONF_ON
  if(fw_init() != 0) {
    DEBUG_PRINT_MSG_NOW("WARNING: FW data in xmem is corrupted");
  }
 #endif /* FW_CONF_ON */
 
  if(flash_logger_load(&stats)) {
    DEBUG_PRINT_MSG_NOW("Reset count: %u", stats.reset_cnt);
  } else {
    DEBUG_PRINT_MSG_NOW("WARNING: stats corrupted");
  }
  stats.reset_cnt++;
  flash_logger_save(&stats);

#if LOG_CONF_ON
  log_init();
  log_print(0);         /* print out all log messages */
#endif /* LOG_CONF_ON */
      
  /* start the LWB thread */
  lwb_start(0, &app_process);
  
  /* --- start of application main loop --- */
  while(1) {
    /* the app task should not do anything until it is explicitly granted 
     * permission (by receiving a poll event) by the LWB task */
    PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_POLL);
    TASK_ACTIVE;      /* application task runs now */
    
    LOG(LOG_EVENT_CONTEXT_SWITCH, 0x01, 0x00);

    if(HOST_ID == node_id) {    /* --------- HOST NODE --------- */  

  #if FW_CONF_ON
      /* send a big chunk of data */
      static uint16_t sent_pkts = 0, crc = 0, state = 0;
      if(state == 0 && lwb_get_time(0) > 30) {
        fw_info_t fw;
        msg_buffer.header.type        = MSG_TYPE_FW_INFO;
        msg_buffer.header.seqnr       = seq_no++;
        msg_buffer.header.payload_len = sizeof(fw_info_t);
        fw.version = 1001;
        fw.len = 230;
        fw.block_size = sizeof(data_t) - 2;
        fw.data_crc = 0xf9c5;
        fw.crc = crc16((uint8_t*)&fw, sizeof(fw_info_t) - 2, 0);
        memcpy(msg_buffer.payload, &fw, sizeof(fw_info_t));
        lwb_send_pkt(LWB_RECIPIENT_BROADCAST, 0, (uint8_t*)&msg_buffer + 2,
                     MSG_HDR_LEN + msg_buffer.header.payload_len - 2);
        state = 1;
      } else if(state == 1) {
        while(sent_pkts < 10) {
          if(sent_pkts == 5) { sent_pkts++; }   /* skip no. 5 */
          /* fill the output buffer */
          message_ext_t* msg_data      = (message_ext_t*)&msg_buffer;
          msg_data->header.type        = MSG_TYPE_FW_DATA;
          msg_data->header.seqnr       = seq_no++;
          msg_data->data.pktnr         = sent_pkts++;
          msg_data->header.payload_len = MSG_EXT_PAYLOAD_LEN;      
          memset(msg_data->data.payload, 'a', MSG_EXT_PAYLOAD_LEN - 2);
          if(lwb_send_pkt(LWB_RECIPIENT_BROADCAST, 0, (uint8_t*)msg_data + 2,
                         MSG_EXT_HDR_LEN + msg_data->header.payload_len - 2)) {
            crc = crc16(msg_data->data.payload, MSG_EXT_PAYLOAD_LEN - 2, crc);
            DEBUG_PRINT_MSG_NOW("pkt %u, crc 0x%04x", sent_pkts - 1, crc);
          } else {
            seq_no--;
            sent_pkts--;
            break;
          }
        }
        if(sent_pkts >= 10) {
          state = 2; 
        }
      } else if(state == 2) {
        msg_buffer.header.type        = MSG_TYPE_FW_VALIDATE;
        msg_buffer.header.seqnr       = seq_no++;
        msg_buffer.header.payload_len = 0;      
        lwb_send_pkt(LWB_RECIPIENT_BROADCAST, 0, (uint8_t*)&msg_buffer + 2,
                     MSG_HDR_LEN + msg_buffer.header.payload_len - 2);
        state = 3;
      }
  #endif /* FW_CONF_ON */
      
      /* analyze and print the received data */
      while(1) {
        uint16_t sender_id;
        uint8_t pkt_len = lwb_rcv_pkt((uint8_t*)&msg_buffer + 2,
                                      &sender_id, 0);
        if(pkt_len) {
          /* use DEBUG_PRINT_MSG_NOW to prevent a queue overflow */
          DEBUG_PRINT_MSG_NOW("data packet #%u received from node %u",
                              msg_buffer.header.seqnr,
                              sender_id);
          /* append the ID to the message */
          msg_buffer.header.device_id = sender_id;
        
          if(msg_buffer.header.type == MSG_TYPE_CC430_HEALTH) {
            /* forward the packet: write it to BOLT */
            BOLT_WRITE((uint8_t*)&msg_buffer, MSG_HDR_LEN + 
                       msg_buffer.header.payload_len);
          } else if(msg_buffer.header.type == MSG_TYPE_ERROR) {
            DEBUG_PRINT_MSG_NOW("error message rcvd from node %u (code 0x%x)",
                                sender_id, msg_buffer.payload16[0]);
          }
  #if FW_CONF_ON
          else if(msg_buffer.header.type == MSG_TYPE_FW_REQ_DATA) {
            /* copy the data into a local buffer */
            uint16_t pkt_ids[MSG_PAYLOAD_LEN / 2];
            uint8_t  n_pkts = msg_buffer.header.payload_len / 2;
            memcpy(pkt_ids, msg_buffer.payload, 
                   msg_buffer.header.payload_len);
            DEBUG_PRINT_MSG_NOW("node %u requested %u FW pkts",
                                sender_id, n_pkts);
            uint8_t i;
            for(i = 0; i < n_pkts; i++) {
              /* a node requests data blocks of the FW */
              message_ext_t* msg_data      = (message_ext_t*)&msg_buffer;
              msg_data->header.type        = MSG_TYPE_FW_DATA;
              msg_data->header.seqnr       = seq_no++;
              msg_data->data.pktnr         = pkt_ids[i];
              msg_data->header.payload_len = MSG_EXT_PAYLOAD_LEN;      
              memset(msg_data->data.payload, 'a', MSG_EXT_PAYLOAD_LEN - 2);
              if(lwb_send_pkt(LWB_RECIPIENT_BROADCAST, 0,
                              (uint8_t*)msg_data + 2, MSG_EXT_HDR_LEN +
                              msg_data->header.payload_len - 2)) {
                DEBUG_PRINT_MSG_NOW("pkt %u resent", pkt_ids[i]);
              } else {
                break;  /* stop here, the output buffer is full */
              }
            }
            state = 2;  /* resend validation request */
          }
  #endif /* FW_CONF_ON */
        } else {
          break;
        }
      }
      /* handle timestamp requests */
      uint64_t time_last_req = bolt_handle_timereq();
      if(time_last_req) {
        /* write the timestamp to BOLT (convert to us) */
        msg_buffer.header.type            = MSG_TYPE_TIMESTAMP;
        msg_buffer.header.generation_time = time_last_req * 1000000 /
                                            ACLK_SPEED + LWB_CLOCK_OFS;
        msg_buffer.header.payload_len     = 0;
        msg_buffer.header.seqnr           = seq_no++;
        BOLT_WRITE((uint8_t*)&msg_buffer, MSG_HDR_LEN);
        //DEBUG_PRINT_MSG_NOW("time request handled");
      }
      /* msg available from BOLT? */
      uint16_t msg_cnt = 0;
      while(BOLT_DATA_AVAILABLE) {
        uint8_t msg_len = 0;
        BOLT_READ(bolt_buffer, msg_len);
        if(msg_len) {
          msg_cnt++;
          memcpy(&msg_buffer, bolt_buffer, MSG_MAX_LEN);
          if(msg_buffer.header.type == MSG_TYPE_LWB_CMD) {
            if(msg_buffer.payload16[0] == LWB_CMD_SET_SCHED_PERIOD) {
              /* adjust the period */
              lwb_sched_set_period(msg_buffer.payload16[1]);
              DEBUG_PRINT_INFO("LWB period set to %us", 
                               msg_buffer.payload16[1]);
            } else if(msg_buffer.payload16[0] == LWB_CMD_SET_STATUS_PERIOD) {
              /* broadcast the message */
              uint16_t recipient = LWB_RECIPIENT_BROADCAST;
              if(msg_buffer.header.payload_len > 4) {
                recipient = msg_buffer.payload16[2];
              }
              DEBUG_PRINT_INFO("sending message to node %u", recipient);
              lwb_send_pkt(recipient, 0,
                           (uint8_t*)&msg_buffer + 2, 
                           MSG_HDR_LEN + msg_buffer.header.payload_len - 2);              
            } else if(msg_buffer.payload16[0] == LWB_CMD_PAUSE) {
              /* stop */            
              while(BOLT_DATA_AVAILABLE) {        /* flush the queue */
                BOLT_READ(bolt_buffer, msg_len);
              }
              /* configure a port interrupt for the IND pin */
              __dint();
              PIN_IES_RISING(BOLT_CONF_IND_PIN);
              PIN_CLR_IFG(BOLT_CONF_IND_PIN);
              PIN_INT_EN(BOLT_CONF_IND_PIN);
              if(BOLT_DATA_AVAILABLE) {
                DEBUG_PRINT_MSG_NOW("failed to stop LWB");   
                __eint();
              } else {
                lwb_pause();
                DEBUG_PRINT_MSG_NOW("LWB paused");
                //LWB_BEFORE_DEEPSLEEP();
                TASK_SUSPENDED;
                __bis_status_register(GIE | SCG0 | SCG1 | CPUOFF);
                __no_operation();
                DEBUG_PRINT_MSG_NOW("LWB resumed");
                lwb_resume();
                continue;
              }
            }
          }
    #if FW_CONF_ON
          /* FW packets received over BOLT */
          else if(msg_buffer.header.type == MSG_TYPE_FW_INFO) {
            fw_info_t* ptr = (fw_info_t*)&msg_buffer.payload;
            if(fw_reset(ptr) == 0) {
              DEBUG_PRINT_INFO("FW info updated (vs: %u, len: %u, crc: %04x)",
                               ptr->version, ptr->len, ptr->data_crc);
            } else {
              DEBUG_PRINT_ERROR("failed to update FW info");
            }
          } else if(msg_buffer.header.type == MSG_TYPE_FW_DATA) {
            fw_store_data(&(((message_ext_t*)&msg_buffer)->data));              
          } else if(msg_buffer.header.type == MSG_TYPE_FW_VALIDATE) {
            uint64_t start_time = rtimer_now_lf();
            uint8_t res = fw_validate();
            if(res == 0) {
              res = fw_backup();  /* now backup the current FW */
              if(res == 0) {
                DEBUG_PRINT_INFO("FW validation successful (took %lums)",
                                 (uint32_t)((rtimer_now_lf() - start_time) * 
                                  1000 / RTIMER_SECOND_LF));
                /* reply to the APP processor */
                msg_buffer.header.type        = MSG_TYPE_FW_READY;
                msg_buffer.header.payload_len = 0;
                msg_buffer.header.seqnr       = seq_no++;
                BOLT_WRITE((uint8_t*)&msg_buffer, MSG_HDR_LEN);
              } else {
                DEBUG_PRINT_ERROR("FW backup failed (code 0x%02x)", res);
              }
            } else if(res == 2) {
              /* some data blocks are still missing, request them */
              uint16_t tmp[MSG_PAYLOAD_LEN / 2];
              res = fw_get_missing_block_ids((uint16_t*)tmp, 
                                             MSG_PAYLOAD_LEN / 2);
              if(res) {
                /* should not happen, communication over BOLT is reliable */
                msg_buffer.header.type        = MSG_TYPE_FW_REQ_DATA;
                msg_buffer.header.payload_len = 0;
                msg_buffer.header.seqnr       = seq_no++;
                memcpy(msg_buffer.payload, (uint8_t*)tmp, res * 2);
                BOLT_WRITE((uint8_t*)&msg_buffer, MSG_HDR_LEN);
                DEBUG_PRINT_INFO("requesting %u missing FW data blocks", res);
              } // else: no missing blocks found
            } else {
              DEBUG_PRINT_ERROR("FW validation failed (code 0x%02x)", res);
            }              
          } else if(msg_buffer.header.type == MSG_TYPE_FW_UPDATE) {
            DEBUG_PRINT_MSG_NOW("firmware update in progress...");
            fw_update();
            DEBUG_PRINT_MSG_NOW("ERROR: update failed");
          }          
    #endif /* FW_CONF_ON */
        }
      }
      if(msg_cnt) {
        DEBUG_PRINT_MSG_NOW("%u messages read from BOLT", msg_cnt);
      }
      
    } else {                    /* --------- SOURCE NODE --------- */
      
      SET_PROGRAM_STATE(11);

    #if SEND_HEALTH_DATA
      static uint8_t stream_state = 0;
    
      if(stream_state != LWB_STREAM_STATE_ACTIVE) {
        stream_state = lwb_stream_get_state(1);
        if(stream_state == LWB_STREAM_STATE_INACTIVE) {
          /* request a stream with ID LWB_STREAM_ID_STATUS_MSG and an IPI
          * (inter packet interval) of LWB_CONF_SCHED_PERIOD_IDLE seconds */
          lwb_stream_req_t my_stream = { node_id, 0, LWB_STREAM_ID_STATUS_MSG,
                                         LWB_CONF_SCHED_PERIOD_IDLE };
          if(!lwb_request_stream(&my_stream, 0)) {
            DEBUG_PRINT_ERROR("stream request failed");
          }
        }
      } else {
        /* generate a node health packet */
        send_msg(MSG_TYPE_CC430_HEALTH, 
                 (uint8_t*)get_node_health(), sizeof(health_msg_t));
        /* is there a packet to read? */      
        while(1) {
          uint8_t pkt_len = lwb_rcv_pkt((uint8_t*)&msg_buffer + 2, 0, 0);
          if(pkt_len) {
            DEBUG_PRINT_INFO("packet received (%ub)", pkt_len);
            if(msg_buffer.header.type == MSG_TYPE_LWB_CMD &&
               msg_buffer.payload16[0] == LWB_CMD_SET_STATUS_PERIOD) {
              // change health/status report interval
              lwb_stream_req_t my_stream = { node_id, 0, 
                                             LWB_STREAM_ID_STATUS_MSG, 
                                             msg_buffer.payload16[1] };
              lwb_request_stream(&my_stream, 0);
            }            
  #if FW_CONF_ON
            else if(msg_buffer.header.type == MSG_TYPE_FW_INFO) {
              fw_info_t* ptr = (fw_info_t*)&msg_buffer.payload;
              if(fw_reset(ptr) == 0) {
                DEBUG_PRINT_INFO("FW info updated (vs: %u, len: %u)",
                                 ptr->version, ptr->len);
              } else {
                DEBUG_PRINT_ERROR("failed to update FW info");
              }
            } else if(msg_buffer.header.type == MSG_TYPE_FW_DATA) {
              fw_store_data(&(((message_ext_t*)&msg_buffer)->data));              
            } else if(msg_buffer.header.type == MSG_TYPE_FW_VALIDATE) {
              uint64_t start_time = rtimer_now_lf();
              uint8_t res = fw_validate();
              if(res == 0) {
                res = fw_backup();  /* now backup the current FW */
                if(res == 0) {
                  DEBUG_PRINT_INFO("FW validation successful (took %lums)",
                                   (uint32_t)((rtimer_now_lf() - start_time) * 
                                    1000 / RTIMER_SECOND_LF));
                  /* send the confirmation to the host */
                  send_msg(MSG_TYPE_FW_READY, 0, 0);
                } else {  
                  DEBUG_PRINT_ERROR("FW backup failed (code 0x%02x)", res);
                }
              } else if(res == 2) {
                /* some data blocks are still missing, request them */
                uint16_t tmp[MSG_PAYLOAD_LEN / 2];
                res = fw_get_missing_block_ids((uint16_t*)tmp, 
                                               MSG_PAYLOAD_LEN / 2);
                if(res) {
                  send_msg(MSG_TYPE_FW_REQ_DATA, (uint8_t*)tmp, res * 2);
                  DEBUG_PRINT_INFO("requesting %u missing FW data blocks", 
                                   res);
                } // else: no missing blocks found
              } else {
                DEBUG_PRINT_ERROR("FW validation failed (code 0x%02x)", res);
              }              
            }
  #endif /* FW_CONF_ON */
          } else {
            /* invalid packet length: abort */
            break;
          }
        }
      }        
      SET_PROGRAM_STATE(12);
  #endif /* SEND_HEALTH_DATA */
    }
  #if QUICK_CONFIG == 4
    /* print out some useful stats */
    int8_t rssi[3] = { 0 };
    glossy_get_rssi(rssi);
    DEBUG_PRINT_INFO("n_rx=%u relay=0x%x rssi1=%d rssi2=%d rssi3=%d", 
                     glossy_get_n_rx(), glossy_get_relay_cnt(), 
                     rssi[0], rssi[1], rssi[2]);
  #endif
    
    /* since this process is only executed at the end of an LWB round, we 
     * can now configure the MCU for minimal power dissipation for the idle
     * period until the next round starts */
#if LWB_CONF_USE_LF_FOR_WAKEUP
    LWB_BEFORE_DEEPSLEEP();
#endif /* LWB_CONF_USE_LF_FOR_WAKEUP */
    
    TASK_SUSPENDED;
  } /* --- end of application main loop --- */

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
#if DEBUG_PORT2_INT
ISR(PORT2, port2_interrupt) 
{    
  PIN_XOR(LED_STATUS);  /* toggle LED */
  
  /* 
   * collect and print debugging info:
   * - stack address / size
   * - return address and status register before ISR (last 32 bits on stack)
   * - whether or not timers are still running and CCR interrupts enabled
   * - state of the global/static variables
   * - some registers, e.g. enabled peripherals
   */
  #define REGISTER_BYTES_ON_STACK       16       /* see lwb.dis file! */
  #define MAX_BSS_SIZE                  3072
    
  uint16_t stack_addr;
  uint16_t stack_size = SRAM_END - (uint16_t)&stack_addr + 1;
  uint8_t peripherals = ((UCA0CTL1 & UCSWRST) << 7) | 
                        ((UCB0CTL1 & UCSWRST) << 6) | 
                        (TA0CTL & (MC_3 | TAIE))    |
                        ((TA1CTL & MC_3) >> 2)      |
                        ((TA1CTL & TAIE) >> 1);

  /* look into the assembly code to find out how many registers have been 
   * put onto the stack since this function has been entered */
  uint16_t sr_addr  = (uint16_t)&stack_addr + REGISTER_BYTES_ON_STACK;
  
  /* status register bits:
   * 8 = arithmetic overflow
   * 7 = SCG1 (system clock generator 1 off)
   * 6 = SCG0
   * 5 = OSCOFF (oscillator off, turns off LFXT)
   * 4 = CPUOFF
   * 3 = GIE
   * 2 = N (result of last operation was negative)
   * 1 = Z (set if result of last operation was zero)
   * 0 = C (carry bit, set if result of last operation produced a carry) */
  
  /* print out the information */
  uart_enable(1);
  printf("stack: %u, ret: 0x%04x, sr: 0x%04x, peri: 0x%02x, lwb_ccr: 0x%x, "
         "state: %u\r\n",
         stack_size, 
         *(volatile uint16_t*)sr_addr, 
         *(volatile uint16_t*)(sr_addr + 2), 
         peripherals,
         ((*(&TA0CCTL0 + LWB_CONF_RTIMER_ID) & CCIE) >> 3) |
           ((*(&TA1CCTL0 + LWB_CONF_LF_RTIMER_ID - RTIMER_LF_0) & CCIE) >> 4),
         program_state);
  
  /* print out the content of the bss section (global & static variables),
   * use objdump -t lwb.exe | grep "\.bss" to map addresses to variables! */
  uint16_t i;
  for(i = SRAM_START; i < MAX_BSS_SIZE + SRAM_START; i++) {
    if((i & 0x000f) == 0) {
      printf("\r\n0x%04x:", i);
    }
    printf(" %02x", *(uint8_t*)i);
  }
  
  
  PIN_CLR_IFG(BOLT_CONF_IND_PIN);
    
  /*
  ENERGEST_ON(ENERGEST_TYPE_CPU);
  
  if(PIN_IFG(BOLT_CONF_IND_PIN)) {
    while(BOLT_DATA_AVAILABLE) {
      uint8_t msg_len = 0;
      BOLT_READ(bolt_buffer, msg_len);
      if(msg_len && 
         msg_buffer.header.type == MSG_TYPE_LWB_CMD &&
         msg_buffer.payload16[0] == LWB_CMD_RESUME) {
        PIN_INT_OFF(BOLT_CONF_IND_PIN);
        __bic_status_register_on_exit(SCG0 | SCG1 | CPUOFF);
        break;
      }
    }
    PIN_CLR_IFG(BOLT_CONF_IND_PIN);
  }
  
  ENERGEST_OFF(ENERGEST_TYPE_CPU);
  */
}
#endif /* DEBUG_PORT2_INT */
/*---------------------------------------------------------------------------*/
/* for debugging: define all unused ISRs */
ISR(SYSNMI, sysnmi_interrupt)
{
  PIN_SET(LED_0);
  switch (SYSSNIV) {
    case SYSSNIV_VMAIFG:
      while(1) { PIN_XOR(COM_MCU_INT1); __delay_cycles(MCLK_SPEED / 15); }
      break;
    default:
        break;
  }
  while(1) { PIN_XOR(COM_MCU_INT1); __delay_cycles(MCLK_SPEED / 10); }
}
ISR(AES, aes_interrupt)
{
  PIN_SET(LED_0); 
  while(1) { PIN_XOR(COM_MCU_INT1); __delay_cycles(MCLK_SPEED / 20); }    
}
ISR(RTC, rtc_interrupt)
{
  PIN_SET(LED_0); 
  while(1) { PIN_XOR(COM_MCU_INT1); __delay_cycles(MCLK_SPEED / 30); }   
}
ISR(PORT1, p1_interrupt)
{
  PIN_SET(LED_0); 
  while(1) { PIN_XOR(COM_MCU_INT1); __delay_cycles(MCLK_SPEED / 40); }   
}
ISR(ADC10, adc_interrupt)
{
  PIN_SET(LED_0); 
  while(1) { PIN_XOR(COM_MCU_INT1); __delay_cycles(MCLK_SPEED / 50); }    
}
ISR(USCI_B0, ucb0_interrupt)
{
  PIN_SET(LED_0); 
  while(1) { PIN_XOR(COM_MCU_INT1); __delay_cycles(MCLK_SPEED / 60); }    
}
ISR(WDT, wdt_interrupt)
{
  PIN_SET(LED_0); 
  while(1) { PIN_XOR(COM_MCU_INT1); __delay_cycles(MCLK_SPEED / 70); }    
}
ISR(COMP_B, comp_interrupt)
{
  PIN_SET(LED_0); 
  while(1) { PIN_XOR(COM_MCU_INT1); __delay_cycles(MCLK_SPEED / 80); }    
}
/*---------------------------------------------------------------------------*/
