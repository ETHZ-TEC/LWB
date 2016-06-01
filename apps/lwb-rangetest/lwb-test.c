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
    DEBUG_PRINT_WARNING("lwb_put_data() failed (queue full?)");
  } /* else: data packet successfully passed to the LWB */
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
#endif 
  
  /* start the LWB thread */
  lwb_start(0, &app_process);
  
  /* main loop of this application task */
  while(1) {
    /* the app task should not do anything until it is explicitly granted 
     * permission (by receiving a poll event) by the LWB task */
    PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_POLL);
    TASK_ACTIVE;      /* application task runs now */

#if HOST_ID == NODE_ID
    /* we are the host */
    /* print out the received data */
    uint16_t sender_id;
    while(1) {
      uint8_t pkt_len = lwb_get_data((uint8_t*)&msg_buffer + 2, &sender_id, 0);
      if(pkt_len) {
        /* use DEBUG_PRINT_MSG_NOW to prevent a queue overflow */
        DEBUG_PRINT_MSG_NOW("data packet %u received from node %u",
                            msg_buffer.header.seqnr,
                            sender_id);
        /* append the ID to the message */
        msg_buffer.header.device_id = sender_id;
        
        if(msg_buffer.header.type == MSG_TYPE_CC430_HEALTH) {
          /* forward the packet: write it to BOLT */
          BOLT_WRITE((uint8_t*)&msg_buffer, MSG_HDR_LEN + 
                     msg_buffer.header.payload_len);
        } else {
          if(msg_buffer.header.type == MSG_TYPE_ERROR) {
            DEBUG_PRINT_MSG_NOW("error message rcvd from node %u (code 0x%x)",
                                sender_id, msg_buffer.payload16[0]);
          }
        }
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
    while(BOLT_DATA_AVAILABLE) {
      uint8_t msg_len = 0;
      BOLT_READ((uint8_t*)&msg_buffer, msg_len);
      if(msg_len) {
        DEBUG_PRINT_INFO("message received over BOLT");
        if(msg_buffer.header.type == MSG_TYPE_LWB_CMD) {
          if(msg_buffer.payload16[0] == LWB_CMD_SET_SCHED_PERIOD) {
            /* adjust the period */
            lwb_sched_set_period(msg_buffer.payload16[1]);
            DEBUG_PRINT_INFO("LWB period set to %us", msg_buffer.payload16[1]);
          } else if (msg_buffer.payload16[0] == LWB_CMD_SET_STATUS_PERIOD) {
            /* broadcast the message */
            DEBUG_PRINT_INFO("broadcasting message");
            lwb_put_data(LWB_RECIPIENT_BROADCAST, 0, 
                         (uint8_t*)&msg_buffer + 2, 
                         MSG_HDR_LEN + msg_buffer.header.payload_len - 2);              
          } else if(msg_buffer.payload16[0] == LWB_CMD_PAUSE) {
            /* stop */            
            while(BOLT_DATA_AVAILABLE) {        /* flush the queue */
              BOLT_READ((uint8_t*)&msg_buffer, msg_len);
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
#else 
    /* we are a source node */
    static uint8_t stream_state = 0;
    
    if(stream_state != LWB_STREAM_STATE_ACTIVE) {
      stream_state = lwb_stream_get_state(1);
      if(stream_state == LWB_STREAM_STATE_INACTIVE) {
        /* request a stream with ID LWB_STREAM_ID_STATUS_MSG and an IPI
        * (inter packet interval) of LWB_CONF_SCHED_PERIOD_IDLE seconds */
        lwb_stream_req_t my_stream =
          { node_id, 0, LWB_STREAM_ID_STATUS_MSG, LWB_CONF_SCHED_PERIOD_IDLE };
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
            lwb_stream_req_t my_stream =
             { node_id, 0, LWB_STREAM_ID_STATUS_MSG, msg_buffer.payload16[1] };
            lwb_request_stream(&my_stream, 0);
          }          
        } else {
          break;
        }
      }
    }
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
  if(PIN_IFG(BOLT_CONF_IND_PIN)) {
    while(BOLT_DATA_AVAILABLE) {        /* flush the queue */
      uint8_t msg_len = 0;
      BOLT_READ((uint8_t*)&msg_buffer, msg_len);
      if(msg_buffer.header.type == MSG_TYPE_LWB_CMD &&
         msg_buffer.payload16[0] == LWB_CMD_RESUME) {
        /* resume the LWB */
        PIN_INT_OFF(BOLT_CONF_IND_PIN);
        __bic_status_register_on_exit(SCG0 | SCG1 | CPUOFF);
        break;
      }
    }
    PIN_CLR_IFG(BOLT_CONF_IND_PIN);
  }
}