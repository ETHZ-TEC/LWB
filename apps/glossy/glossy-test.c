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
 *          Federico Ferrari
 */
 
/**
 * @brief Glossy Test Application
 * 
 * A simple test app, no drift compensation or state machine. All static.
 * The host node sends a sync packet every 100ms.
 */


#include "contiki.h"
#include "platform.h"

static struct pt glossy_pt; /* glossy protothread */
static uint8_t   glossy_payload[RF_CONF_MAX_PKT_LEN];
/*---------------------------------------------------------------------------*/
#ifdef APP_TASK_ACT_PIN
#define TASK_ACTIVE             PIN_SET(APP_TASK_ACT_PIN)
#define TASK_SUSPENDED          PIN_CLR(APP_TASK_ACT_PIN)
#else
#define TASK_ACTIVE
#define TASK_SUSPENDED
#endif /* APP_TASK_ACT_PIN */

#define WAIT_UNTIL(time) \
{\
  rtimer_schedule(GLOSSY_RTIMER_ID, (time), 0, glossy_thread);\
  TASK_SUSPENDED;\
  PT_YIELD(&glossy_pt);\
  TASK_ACTIVE;\
}
#ifndef MAX
#define MAX(a, b)               ((a) > (b) ? (a) : (b))
#endif /* MAX */
/*---------------------------------------------------------------------------*/
PT_THREAD(glossy_thread(rtimer_t *rt)) 
{  
  static uint16_t bootstrap_cnt = 0;
  static uint16_t pkt_cnt = 0;
  static uint16_t miss_cnt = 0;
  static uint8_t  max_hop = 0;
  static uint8_t  sync_state = 0;
  static int32_t  avg_rssi = 0;
  static uint16_t rssi_cnt = 0;
                        
  /* note: all statements above PT_BEGIN() will be executed each time the 
   * protothread is scheduled */
  SVS_DISABLE;
  
  /* compose the packet */
  memset(glossy_payload, GLOSSY_PAYLOAD_LEN, GLOSSY_PAYLOAD_LEN); 
  
  PT_BEGIN(&glossy_pt);   /* declare variables before this statement! */
 
  /* main loop of this application task */
  while(1) {
    /* HOST NODE */
    if(node_id == HOST_ID) {      
      /* send a packet */
      glossy_start(node_id, (uint8_t*)&glossy_payload, GLOSSY_PAYLOAD_LEN,
                   GLOSSY_N_TX, GLOSSY_WITH_SYNC,
                   GLOSSY_WITH_RF_CAL);
      WAIT_UNTIL(rt->time + GLOSSY_T_SLOT);
      glossy_stop();
      if(glossy_get_rssi(0) != 0) {
        avg_rssi += glossy_get_rssi(0);
        rssi_cnt++;
      }
      DEBUG_PRINT_INFO("packet sent, rssi=%ddBm last_rssi=%ddBm avg=%ddBm", 
                       glossy_get_rssi(0), rf1a_get_last_packet_rssi(), 
                       (int16_t)(avg_rssi / MAX(1, rssi_cnt)));
            
      debug_print_poll();
      WAIT_UNTIL(rt->time - GLOSSY_T_SLOT + GLOSSY_PERIOD);
      
    /* SOURCE NODE */
    } else {
      if(!sync_state) {
        DEBUG_PRINT_INFO("BOOTSTRAP\r\n");
        bootstrap_cnt++;
        /* synchronize first! wait for a packet... */
        do {
          glossy_start(GLOSSY_UNKNOWN_INITIATOR, (uint8_t*)&glossy_payload, 
                       GLOSSY_UNKNOWN_PAYLOAD_LEN,
                       GLOSSY_N_TX, GLOSSY_WITH_SYNC,
                       GLOSSY_WITH_RF_CAL);
          WAIT_UNTIL(rt->time + GLOSSY_T_SLOT);
          glossy_stop();          
        } while(!glossy_is_t_ref_updated());        
        /* synchronized! */
        sync_state = 1;        
      } else { 
        /* already synchronized, receive a packet */
        glossy_start(GLOSSY_UNKNOWN_INITIATOR, (uint8_t*)&glossy_payload,
                     GLOSSY_UNKNOWN_PAYLOAD_LEN, 
                     GLOSSY_N_TX, GLOSSY_WITH_SYNC,
                     GLOSSY_WITH_RF_CAL);
        WAIT_UNTIL(rt->time + GLOSSY_T_SLOT + GLOSSY_T_GUARD);
        glossy_stop();
      
        /* at least one packet received? */
        if(glossy_get_n_rx()) {
          pkt_cnt++;
        } else {
          miss_cnt++;
        }
        /* has the reference time been updated? */
        if(glossy_is_t_ref_updated()) {
          /* sync received */
          if(glossy_get_snr()) {
            avg_rssi += glossy_get_snr();
            rssi_cnt++;
          }
        } else {
          /* sync missed */
          sync_state = 0;
          continue;
        }
        if(glossy_get_relay_cnt_first_rx() > max_hop) {
          max_hop = glossy_get_relay_cnt_first_rx();
        }
        /* print out some stats */
        DEBUG_PRINT_INFO("rcv=%u miss=%u boot=%u per=%u "
                         "m_hop=%u snr=%ddBm avg_snr=%ddBm",
                         pkt_cnt, 
                         miss_cnt, 
                         bootstrap_cnt,
                         glossy_get_per(),
                         max_hop,
                         glossy_get_snr(), 
                         (int16_t)(avg_rssi / MAX(1, rssi_cnt)));
      }    
      debug_print_poll();
      WAIT_UNTIL(glossy_get_t_ref() - GLOSSY_REF_OFS + GLOSSY_PERIOD - 
                 GLOSSY_T_GUARD);
    }
  }
  
  PT_END(&glossy_pt);
}
/*---------------------------------------------------------------------------*/
PROCESS(app_process, "Application Task");
AUTOSTART_PROCESSES(&app_process);
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(app_process, ev, data) 
{    
  PROCESS_BEGIN();  
  
  /* application specific initialization code */
  /* all other necessary initialization is done in contiki-cc430-main.c */
    
  if (HOST_ID == node_id) {
    // set the content of the payload
    uint8_t i;
    for (i = 0; i < GLOSSY_PAYLOAD_LEN; i++) {
      glossy_payload[i] = i;
    }
  }
  /* start the glossy thread in 1s */  
  rtimer_schedule(GLOSSY_RTIMER_ID, rtimer_now_hf() + RTIMER_SECOND_HF,
                  0, glossy_thread);    
  
  PROCESS_END();
  
  return 0;
}
/*---------------------------------------------------------------------------*/
