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
 *          Marco Zimmerling
 */
 
/**
 * @brief Low-Power Wireless Bus Test Application
 * 
 * All source nodes sample the temperature and supply voltage and send this
 * information to the host node.
 * The used scheduler is static, i.e. the period is constant. A source node
 * may send a stream request in each round.
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
  static uint8_t stream_state = 0;
    
  PROCESS_BEGIN();  
  
  /* application specific initialization code */
  if(HOST_ID != node_id) {
    adc_init();
  }
    
  /* all other necessary initialization is done in contiki-cc430-main.c */
  
  /* start the LWB thread */
  lwb_start(0, &app_process);
  
  /* main loop of this application task */
  while(1) {
    /* the app task should not do anything until it is explicitly granted 
     * permission (by receiving a poll event) by the LWB task */
    PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_POLL);
    TASK_ACTIVE;      /* application task runs now */
    
    if(HOST_ID == node_id) {
      /* we are the host */
      /* print out the received data */
      uint8_t pkt_buffer[LWB_CONF_MAX_DATA_PKT_LEN];
      uint16_t sender_id;
      while(1) {
        uint8_t pkt_len = lwb_get_data(pkt_buffer, &sender_id, 0);
        if(pkt_len) {
          /* use DEBUG_PRINT_MSG_NOW to prevent a queue overflow */
          DEBUG_PRINT_MSG_NOW("data packet received from node %u: "
                              "%u°C, %umV", 
                              sender_id, pkt_buffer[0], 
                              (uint16_t)pkt_buffer[1] * 4 + 2000);
        } else {
          break;
        }
      } 
    } else {
      /* we are a source node */
      if(stream_state != LWB_STREAM_STATE_ACTIVE) {
        stream_state = lwb_stream_get_state(1);
        if(stream_state == LWB_STREAM_STATE_INACTIVE) {
          /* request a stream with ID 1 and IPI 5 */
          lwb_stream_req_t my_stream = { node_id, 0, 1, 10 };
          if(!lwb_request_stream(&my_stream, 0)) {
            DEBUG_PRINT_ERROR("stream request failed");
          }
        }
      } else {
        /* collect ADC samples and create a data packet */
        uint8_t data[2];
        adc_get_data(data);
        if(!lwb_put_data(0, 1, data, 2)) {
          DEBUG_PRINT_WARNING("out queue full, packet dropped");
        } /* else: data packet successfully passed to the LWB */
      }
    }
    /* IMPORTANT: This process must not run for more than a few hundred
     * milliseconds in order to enable proper operation of the LWB */
    TASK_SUSPENDED;
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
