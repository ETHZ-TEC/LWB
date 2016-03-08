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
 * @brief Glossy Test Application
 * 
 * The host node sends a sync packet every 100ms.
 */


#include "contiki.h"
#include "platform.h"

static struct pt                glossy_pt; /* glossy protothread */
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
#define MAX(a, b)               ((a) > (b) ? (a) : (b))
/*---------------------------------------------------------------------------*/
PT_THREAD(glossy_thread(rtimer_t *rt)) 
{  
  static uint8_t  glossy_payload[GLOSSY_MAX_PACKET_LEN];
  static uint16_t bootstrap_cnt = 0;
  static int16_t  drift = 0;
  static uint16_t pkt_cnt = 0;
  static uint16_t miss_cnt = 0;
  static uint8_t  max_hop = 0;
  static uint8_t  sync_state = 0;
  static uint16_t t_guard = 0;
  static rtimer_clock_t t_ref = 0, 
                        t_ref_last = 0, 
                        t_start = 0;
  static int32_t  avg_rssi = 0;
  static uint16_t rssi_cnt = 0;
                        
  /* note: all statements above PT_BEGIN() will be executed each time the 
   * protothread is scheduled */
  SVS_DISABLE;
  
  PT_BEGIN(&glossy_pt);   /* declare variables before this statement! */
 
  /* main loop of this application task */
  while(1) {
    /* HOST NODE */
    if(node_id == HOST_ID) {  
      t_start = rt->time;
      /* compose the packet (just send the timestamp) */
      memcpy(glossy_payload, &t_start, sizeof(rtimer_clock_t));        
      /* send a packet */
      glossy_start(node_id, (uint8_t*)&glossy_payload, sizeof(rtimer_clock_t),
                   GLOSSY_N_TX, GLOSSY_WITH_SYNC,
                   GLOSSY_WITH_RF_CAL);
      WAIT_UNTIL(rt->time + GLOSSY_T_SLOT);
      glossy_stop();
      if(glossy_get_rssi() != 0) {
        avg_rssi += glossy_get_rssi();
        rssi_cnt++;
      }
      DEBUG_PRINT_INFO("packet sent, rssi=%ddBm last_pkt_rssi=%ddBm avg=%ddBm", 
                       glossy_get_rssi(), rf1a_get_last_packet_rssi(), 
                       (int16_t)(avg_rssi / MAX(1, rssi_cnt)));
        
    /* SOURCE NODE */
    } else {
      if(!sync_state) {
        DEBUG_PRINT_MSG_NOW("BOOTSTRAP\r\n");
        bootstrap_cnt++;
        drift   = 0;
        t_guard = GLOSSY_T_GUARD;
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
        t_start    = glossy_get_t_ref() - T_REF_OFS;
        
      } else { 
        /* already synchronized, receive a packet */
        glossy_start(GLOSSY_UNKNOWN_INITIATOR, (uint8_t*)&glossy_payload,
                     GLOSSY_UNKNOWN_PAYLOAD_LEN, 
                     GLOSSY_N_TX, GLOSSY_WITH_SYNC,
                     GLOSSY_WITH_RF_CAL);
        WAIT_UNTIL(rt->time + GLOSSY_T_SLOT + t_guard);
        uint16_t n_rx = glossy_stop();
        uint16_t snr = 0;
      
        /* at least one packet received? */
        if(n_rx) {
          pkt_cnt++;
        } else {
          miss_cnt++;
        }
        /* has the reference time been updated? */
        if(glossy_is_t_ref_updated()) {
          /* sync received */
          t_ref      = glossy_get_t_ref();   
          t_start    = t_ref - T_REF_OFS;
          sync_state = 1;         /* synchronized */
          t_guard    = GLOSSY_T_GUARD;
          snr        = glossy_get_snr();
          if(snr) {
            avg_rssi += snr;
            rssi_cnt++;
          }

#if (TIME_SCALE == 1) /* only calc drift when TIME_SCALE is not used */
          /* drift compensation (calculate drift in clock cycles per second) */
          int32_t d = (int32_t)((t_ref - t_ref_last) - ((int32_t)GLOSSY_PERIOD *
                      RTIMER_SECOND_HF)) / (int32_t)(GLOSSY_PERIOD);
          t_ref_last = t_ref;
          if((d < MAX_CLOCK_DEV) && (d > -MAX_CLOCK_DEV)) {
            drift = (int16_t)d;   /* update only if deviation is within specs */
          } else if(drift) {      
            /* deviation is too high (timer update overrun) */
            DEBUG_PRINT_ERROR("timing error");
          }
#endif
        } else {
          /* sync missed */
          if(sync_state == 2) {
            sync_state = 0; /* go back to bootstrap */
            continue;
          } else {
            drift = 0;
            sync_state = 2;               /* unsynced */   
            t_guard = GLOSSY_T_GUARD_2;   /* increase guard time */
            /* estimate t_ref */
            t_ref = t_ref + GLOSSY_PERIOD * (RTIMER_SECOND_HF) /
                            TIME_SCALE; 
            t_start = t_ref - T_REF_OFS;
          }
        }
        if(glossy_get_relay_cnt_first_rx() > max_hop) {
          max_hop = glossy_get_relay_cnt_first_rx();
        }
        /* print out some stats */
        DEBUG_PRINT_INFO("rcv=%u miss=%u boot=%u d=%d per=%u hop=%u "
                         "m_hop=%u snr=%ddBm avg_snr=%ddBm",
                         pkt_cnt, 
                         miss_cnt, 
                         bootstrap_cnt, 
                         drift, 
                         glossy_get_per(), 
                         glossy_get_relay_cnt_first_rx(), 
                         max_hop,
                         snr, 
                         (int16_t)(avg_rssi / MAX(1, rssi_cnt)));
      }
    }
    
    debug_print_poll();
    WAIT_UNTIL(t_start + GLOSSY_PERIOD * (RTIMER_SECOND_HF + drift) /
               TIME_SCALE - t_guard);
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
      
  /* start the glossy thread in 1s */  
  rtimer_schedule(GLOSSY_RTIMER_ID, rtimer_now_hf() + RTIMER_SECOND_HF,
                  0, glossy_thread);            
  
  PROCESS_END();
  
  return 0;
}
/*---------------------------------------------------------------------------*/
