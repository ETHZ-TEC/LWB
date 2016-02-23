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
 *          Marco Zimmerling
 */
 
/**
 * @brief Low-Power Wireless Bus Test Application
 * 
 * The burst scheduler is used in this example. It is designed for scenarios
 * where most of the time no data is transmitted, but occasionally (upon an
 * event) a node needs to send a large amount of data to the host.
 * 
 * This demo application is not designed to run on Flocklab. 
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
PROCESS(app_process, "Application Task");
AUTOSTART_PROCESSES(&app_process);
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(app_process, ev, data) 
{   
  static uint16_t round_cnt = 0;
  
  PROCESS_BEGIN();
          
#if LWB_CONF_USE_LF_FOR_WAKEUP
  SVS_DISABLE;
#endif /* LWB_CONF_USE_LF_FOR_WAKEUP */
  /* all other necessary initialization is done in contiki-cc430-main.c */
    
  /* start the LWB thread */
  lwb_start(0, &app_process);
  
  /* INIT code */
#if HOST_ID != NODE_ID
  /* configure port interrupt */
  PIN_CFG_INT(DEBUG_SWITCH);
  PIN_PULLUP_EN(DEBUG_SWITCH); 
#endif
  
  /* MAIN LOOP of this application task */
  while(1) {
    /* the app task should not do anything until it is explicitly granted 
     * permission (by receiving a poll event) by the LWB task */
    PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_POLL);
    TASK_ACTIVE;      /* application task runs now */
        
    if(HOST_ID == NODE_ID) {
      /* we are the host */
      /* print out the received data */
      uint8_t pkt_buffer[LWB_CONF_MAX_DATA_PKT_LEN], stream_id;
      uint16_t sender_id;
      uint8_t pkt_cnt = 0;
      uint16_t num_bytes = 0;
      while(1) {
        uint8_t pkt_len = lwb_get_data(pkt_buffer, &sender_id, &stream_id);
        if(pkt_len) {
          pkt_cnt++;
          num_bytes += pkt_len;
        } else {
          break;
        }
      } 
      DEBUG_PRINT_INFO("%u data packets (%ub) received",
                       pkt_cnt, num_bytes);
    } else {
      if(round_cnt == 2) {
        /* request a stream */
        lwb_stream_req_t my_stream = 
          { node_id, 0, 1, LWB_CONF_SCHED_PERIOD_IDLE };
        if(!lwb_request_stream(&my_stream, 0)) {
          DEBUG_PRINT_ERROR("low stream request failed");
        }
        /* request a stream once and send a dummy packet with the node ID */
        lwb_put_data(0, 1, (uint8_t*)&node_id, 2);  
      } else if(round_cnt == 5) {
        /* request a stream */
        lwb_stream_req_t my_stream = 
          { node_id, 0, 1, 0 }; /* remove this stream */
        if(!lwb_request_stream(&my_stream, 0)) {
          DEBUG_PRINT_ERROR("low stream request failed");
        }
        /* request a stream once and send a dummy packet with the node ID */
        lwb_put_data(0, 1, (uint8_t*)&node_id, 2);  
      }
        
      if(round_cnt > 8) {
        /* we are a source node */
        static uint8_t data_pkt[LWB_CONF_MAX_DATA_PKT_LEN - 3];
        /* generate some data, keep the buffer filled */
        while(lwb_put_data(0, 2, (uint8_t*)data_pkt,
                           LWB_CONF_MAX_DATA_PKT_LEN - 3));
      }
    }
    round_cnt++;
                
    /* IMPORTANT: This process must not run for more than a few hundred
     * milliseconds in order to enable proper operation of the LWB */
    
    /* since this process is only executed at the end of an LWB round, we 
     * can now configure the MCU for minimal power dissipation for the idle
     * period until the next round starts */
#if LWB_CONF_USE_LF_FOR_WAKEUP
  #if FRAM_CONF_ON
    fram_sleep();
  #endif /* FRAM_CONF_ON */
    /* disable all peripherals, reconfigure the GPIOs and disable XT2 */
    TA0CTL   &= ~MC_3; /* stop TA0 */
    DISABLE_XT2();
  #ifdef MUX_SEL_PIN
    PIN_CLR(MUX_SEL_PIN);
  #endif /* MUX_SEL_PIN */
    P1SEL = 0; /* reconfigure GPIOs */
    P1DIR = 0xff;
#endif /* LWB_CONF_USE_LF_FOR_WAKEUP */
    
    TASK_SUSPENDED;
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
#if HOST_ID != NODE_ID
ISR(PORT1, port1_interrupt) 
{
  if(PIN_IFG(DEBUG_SWITCH)) {
    //PIN_XOR(LED_0);
    lwb_stream_req_t my_stream = { node_id, 0, 2, 1 };
    if(!lwb_request_stream(&my_stream, 0)) {
      DEBUG_PRINT_ERROR("stream request failed");
    } else {
      DEBUG_PRINT_INFO("stream requested");   
    }
    PIN_CLR_IFG(DEBUG_SWITCH);
  } 
}
#endif
/*---------------------------------------------------------------------------*/