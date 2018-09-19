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
  static uint8_t pkt_buffer[LWB_CONF_MAX_DATA_PKT_LEN];
  static uint8_t stream_state = 0;

  PROCESS_BEGIN();

  /* start the LWB thread */
  lwb_start(0, &app_process);

  /* main loop of this application task */
  while(1) {
    /* the app task should not do anything until it is explicitly granted 
     * permission (by receiving a poll event) by the LWB task */
    TASK_SUSPENDED;
    PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_POLL);
    TASK_ACTIVE;      /* application task runs now */

    if(HOST_ID == node_id) {
      /* we are the host: read the received data packets */
      uint16_t cnt = 0;
      uint16_t pkt_len;
      do {
        cnt++;
        pkt_len = lwb_rcv_pkt(pkt_buffer, 0, 0);
      } while(pkt_len);
      DEBUG_PRINT_INFO("rcvd packets: %u, FSR: %u, PER: %u", // CPU DC: %u, RF DC: %u",
                       cnt - 1,
                       glossy_get_fsr(),
                       glossy_get_per() /*,
                       DCSTAT_CPU_DC,
                       DCSTAT_RF_DC */);
      DEBUG_PRINT_INFO("Radio DC: %u.%02u", DCSTAT_RF_DC/100, DCSTAT_RF_DC%100);

    } else {
      /* we are a source node */
      if(stream_state != LWB_STREAM_STATE_ACTIVE) {
        stream_state = lwb_stream_get_state(1);
        if(stream_state == LWB_STREAM_STATE_INACTIVE) {
          /* request a stream */
          lwb_stream_req_t my_stream = { node_id, 0, 1, SOURCE_IPI };
          if(!lwb_request_stream(&my_stream, 0)) {
            DEBUG_PRINT_ERROR("stream request failed");
          }
        }
      } else {
        /* keep the output buffer filled, generate a dummy packets */
        memset(pkt_buffer, 0xaa, LWB_CONF_MAX_DATA_PKT_LEN - 3);
        uint16_t cnt = 0;
        while(lwb_send_pkt(0, 1, pkt_buffer,
                           LWB_CONF_MAX_DATA_PKT_LEN - 3)) cnt++;
        if(cnt) {
          DEBUG_PRINT_INFO("%u LWB packets created", cnt);
        }
      }
      DEBUG_PRINT_INFO("FSR: %u, PER: %u", // CPU DC: %u, RF DC: %u",
                       glossy_get_fsr(),
                       glossy_get_per() /*,
                       DCSTAT_CPU_DC,
                       DCSTAT_RF_DC*/);
      DEBUG_PRINT_INFO("Radio DC: %u.%02u", DCSTAT_RF_DC/100, DCSTAT_RF_DC%100);
    }
    /* since this process is only executed at the end of an LWB round, we 
     * can now configure the MCU for minimal power dissipation for the idle
     * period until the next round starts */
#if LWB_CONF_USE_LF_FOR_WAKEUP
    LWB_BEFORE_DEEPSLEEP();
#endif /* LWB_CONF_USE_LF_FOR_WAKEUP */
    
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
