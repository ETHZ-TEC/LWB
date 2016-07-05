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
static uint16_t  seq_no = 0;
static message_t msg_buffer;
static uint8_t   bolt_buffer[BOLT_CONF_MAX_MSG_LEN];
#if FW_CONF_ON
static fw_info_t fw = { 0 };
static uint8_t fw_block_info[FW_BLOCK_INFO_SIZE] = { 0 };
#endif /* FW_CONF_ON */
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

  if(!lwb_put_data(LWB_RECIPIENT_SINKS, LWB_STREAM_ID_STATUS_MSG, 
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

  REFCTL0 |= REFON;
  __delay_cycles(MCLK_SPEED / 25000);                /* let REF settle */
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
uint8_t
fw_validate(void)
{
  /* verify the received FW data */
  /* first, check the info struct */
  if(crc16((uint8_t*)&fw, sizeof(fw_info_t) - 2, 0) != fw.crc) {
    return 1;
  }
  uint16_t n_blocks = 0, i = 0;
  while(i < FW_BLOCK_INFO_SIZE) {
    if(fw_block_info[i] == 0xff) {
      n_blocks += 8;
      i++;
    } else {
      /* these are the last blocks */
      for(i = 0; i < 8; i++) {
        if(fw_block_info[i] & (1 << i)) {
          n_blocks++;
        }
      }
      break;  /* stop here */
    }                    
  }
  if(((fw.len + (fw.block_size - 1)) / fw.block_size) != 
     n_blocks) {
    return 2;
  }
  /* received all blocks! -> now check CRC */
  uint16_t crc = 0, len = 0;
  uint8_t buffer[128];
  while(len < fw.len) {
    uint16_t block_size = 128;
    if(!xmem_read(FW_DATA_START + len, block_size, buffer)) {
      return 3;
    }
    if((fw.len - len) < 128) {
      block_size = fw.len - len;
    }
    crc = crc16(buffer, block_size, crc);  
    len += 128;
  }
  if(crc != fw.data_crc) {
      return 4;
  }
 
  return 0;     /* success! */
}
/*---------------------------------------------------------------------------*/
uint8_t
fw_backup(void)
{
  /* start at address 0, copy the whole flash content into the external 
   * memory */
  if(!xmem_write(FW_BACKUP_ADDR_XMEM, FW_SIZE_XMEM, (uint8_t*)FLASH_START)) {
    return 1;
  }  
  /* verify the content */
  uint32_t xmem_addr = FW_BACKUP_ADDR_XMEM;
  uint16_t flash_addr = CODE_START,
           read_bytes = 0;
  uint8_t buffer[256];  
  while(read_bytes < FW_SIZE_XMEM) { 
    /* load a block of data into the RAM */
    if(!xmem_read(xmem_addr, 256, buffer)) {
      return 2;
    }
    uint16_t i;
    for(i = 0; i < 256; i++) {
      if(buffer[i] != *((uint8_t*)flash_addr + i)) {
        return 3;
      }
    }
    flash_addr += 256;
    xmem_addr += 256;
    read_bytes += 256;
  }
  return 0;
}
/*---------------------------------------------------------------------------*/
PROCESS(app_process, "Application Task");
AUTOSTART_PROCESSES(&app_process);
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(app_process, ev, data) 
{   
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

  /* erase an info memory segment */
  /*uint8_t* info_addr = (uint8_t*)(INFO_START + 128);
  flash_erase_segment(info_addr);
  if(flash_erase_check(info_addr, 128)) {
    flash_write([data], info_addr, [num_bytes]);
  }*/
  
 #if FW_CONF_ON
  /* quick error check */
  if((FW_BACKUP_ADDR_XMEM + FW_SIZE_XMEM) > FRAM_CONF_SIZE) {
    DEBUG_PRINT_MSG_NOW("ERROR: data exceeds FRAM size!");
    while(1);
  }
  /* TODO: move this code to the very beginning of the main() */
  /* verify integrity of the FW info block */
  if(xmem_read(FW_ADDR_XMEM, sizeof(fw_info_t), (uint8_t*)&fw)) {
    if(fw.crc != crc16((uint8_t*)&fw, sizeof(fw_info_t) - 2, 0)) {
      DEBUG_PRINT_MSG_NOW("ERROR: FW info block integrity test failed");
      /* clear the data */
      memset(&fw, 0, sizeof(fw_info_t));
      fw.crc = crc16((uint8_t*)&fw, sizeof(fw_info_t) - 2, 0);
      xmem_write(FW_ADDR_XMEM, sizeof(fw_info_t), (uint8_t*)&fw);
    } else {
      DEBUG_PRINT_MSG_NOW("FW OK (status: %u)", fw.status);
    }
  } else {
    DEBUG_PRINT_MSG_NOW("ERROR: failed to read FW info block");
  }
 #endif /* FW_CONF_ON */
#endif /* HOST_ID != NODE_ID */

#if LOG_CONF_ON
  log_init();
  log_print(0);         /* print out all log messages */
#endif /* LOG_CONF_ON */
      
  /* start the LWB thread */
  lwb_start(0, &app_process);
  
  /* main loop of this application task */
  while(1) {
    /* the app task should not do anything until it is explicitly granted 
     * permission (by receiving a poll event) by the LWB task */
    PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_POLL);
    TASK_ACTIVE;      /* application task runs now */
    
    LOG(LOG_EVENT_CONTEXT_SWITCH, 0x01, 0x00);

    if(HOST_ID == node_id) {    /* HOST NODE */
      /* we are the host */
      /* print out the received data */
      uint16_t sender_id;
      while(1) {
        uint8_t pkt_len = lwb_get_data((uint8_t*)&msg_buffer + 2,
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
        } else {
          break;
        }
      }
      
  #if FW_CONF_ON
      /* send a big chunk of data */
      static uint16_t sent_pkts = 0, crc = 0, state = 0;
      if(state == 0 && lwb_get_time(0) > 30) {
        msg_buffer.header.type        = MSG_TYPE_FW_INFO;
        msg_buffer.header.seqnr       = seq_no++;
        msg_buffer.header.payload_len = sizeof(fw_info_t);
        fw.version = 1001;
        fw.len = 230;
        fw.block_size = FW_BLOCK_SIZE;
        fw.data_crc = 0xf9c5;
        fw.crc = crc16((uint8_t*)&fw, sizeof(fw_info_t) - 2, 0);
        memcpy(msg_buffer.payload, &fw, sizeof(fw_info_t));
        lwb_put_data(LWB_RECIPIENT_BROADCAST, 0, (uint8_t*)&msg_buffer + 2, 
                     MSG_HDR_LEN + msg_buffer.header.payload_len - 2);
        state = 1;
      } else if(state == 1) {
        while(sent_pkts < 10) {
          /* fill the output buffer */
          message_ext_t* msg_data      = (message_ext_t*)&msg_buffer;
          msg_data->header.type        = MSG_TYPE_FW_DATA;
          msg_data->header.seqnr       = seq_no++;
          msg_data->data.pktnr         = sent_pkts++;
          msg_data->header.payload_len = MSG_EXT_PAYLOAD_LEN;      
          memset(msg_data->data.payload, 'a', MSG_EXT_PAYLOAD_LEN - 2);
          if(lwb_put_data(LWB_RECIPIENT_BROADCAST, 0, (uint8_t*)msg_data + 2, 
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
        lwb_put_data(LWB_RECIPIENT_BROADCAST, 0, (uint8_t*)&msg_buffer + 2, 
                     MSG_HDR_LEN + msg_buffer.header.payload_len - 2);
        state = 3;
      }
         
  #endif /* FW_CONF_ON */

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
      while(BOLT_DATA_AVAILABLE) {
        DEBUG_PRINT_INFO("BOLT: data is available");
        uint8_t msg_len = 0;
        BOLT_READ(bolt_buffer, msg_len);
        if(msg_len) {
          memcpy(&msg_buffer, bolt_buffer, MSG_MAX_LEN);
          DEBUG_PRINT_INFO("BOLT message rcvd");
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
              lwb_put_data(recipient, 0,
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
        }
      }
    } else {  /* SOURCE NODE */

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
          uint8_t pkt_len = lwb_get_data((uint8_t*)&msg_buffer + 2, 0, 0);
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
              /* make sure the block size is valid */
              memcpy(&fw, msg_buffer.payload, sizeof(fw_info_t));
              if(FW_BLOCK_SIZE != fw.block_size ||
                 crc16((uint8_t*)&fw, sizeof(fw_info_t) - 2, 0) != fw.crc) {
                xmem_read(FW_ADDR_XMEM, sizeof(fw_info_t), (uint8_t*)&fw);
                DEBUG_PRINT_ERROR("FW info validation failed");
              } else {
                /* store the info */
                fw.status = FW_STATUS_RECEIVING;
                fw.crc    = crc16((uint8_t*)&fw, sizeof(fw_info_t) - 2, 0);
                if(!xmem_write(FW_ADDR_XMEM, sizeof(fw_info_t),
                               (uint8_t*)&fw)) {
                  DEBUG_PRINT_ERROR("failed to store FW info in XMEM");
                }
                DEBUG_PRINT_INFO("FW info updated (vs: %u, len: %u, block: %u)",
                                 fw.version, fw.len, fw.block_size);
                /* erase the FW block info */
                memset(fw_block_info, 0, FW_BLOCK_INFO_SIZE);
                xmem_erase(FW_ADDR_XMEM + sizeof(fw_info_t),
                           FW_BLOCK_INFO_SIZE);                
              }  
            } else if(msg_buffer.header.type == MSG_TYPE_FW_DATA) {
              message_ext_t* msg_data = (message_ext_t*)&msg_buffer;
              xmem_write(FW_DATA_START + msg_data->data.pktnr * FW_BLOCK_SIZE,
                         FW_BLOCK_SIZE,
                         msg_data->data.payload);
              /* mark this block as received */
              fw_block_info[msg_data->data.pktnr / 8] |= 
                                          (1 << (msg_data->data.pktnr & 0x07));
              xmem_write(FW_ADDR_XMEM + sizeof(fw_info_t), 1, 
                         &fw_block_info[msg_data->data.pktnr / 8]);
              
            } else if(msg_buffer.header.type == MSG_TYPE_FW_VALIDATE) {
              uint64_t start_time = rtimer_now_lf();
              uint8_t res = fw_validate();
              if(res == 0) {
                /* now backup the current FW */
                res = fw_backup();
                if(res == 0) {
                  /* validation/preparation successful, update the state */
                  fw.status = FW_STATUS_VALIDATED;
                  fw.crc    = crc16((uint8_t*)&fw, sizeof(fw_info_t) - 2, 0);
                  xmem_write(FW_ADDR_XMEM, sizeof(fw_info_t), (uint8_t*)&fw);
                  DEBUG_PRINT_INFO("FW validation successful (took %lums)", (uint32_t)((rtimer_now_lf() - start_time) * 1000 / RTIMER_SECOND_LF));
                } else {  
                  DEBUG_PRINT_ERROR("FW backup failed (code 0x%02x)", res);
                }
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
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
ISR(PORT2, port2_interrupt) 
{
  ENERGEST_ON(ENERGEST_TYPE_CPU);
  
  if(PIN_IFG(BOLT_CONF_IND_PIN)) {
    while(BOLT_DATA_AVAILABLE) {        /* flush the queue */
      uint8_t msg_len = 0;
      BOLT_READ(bolt_buffer, msg_len);
      if(msg_len && 
         msg_buffer.header.type == MSG_TYPE_LWB_CMD &&
         msg_buffer.payload16[0] == LWB_CMD_RESUME) {
        /* resume the LWB */
        PIN_INT_OFF(BOLT_CONF_IND_PIN);
        __bic_status_register_on_exit(SCG0 | SCG1 | CPUOFF);
        break;
      }
    }
    PIN_CLR_IFG(BOLT_CONF_IND_PIN);
  }
  
  ENERGEST_OFF(ENERGEST_TYPE_CPU);
}
/*---------------------------------------------------------------------------*/
/* for debugging: define all unused ISRs */
ISR(SYSNMI, sysnmi_interrupt)
{
  PIN_SET(LED_0); 
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
