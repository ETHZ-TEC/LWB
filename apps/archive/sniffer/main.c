/*
 * Copyright (c) 2018, Swiss Federal Institute of Technology (ETH Zurich).
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


#include "main.h"

/*---------------------------------------------------------------------------*/
config_t cfg = { 0 };
/*---------------------------------------------------------------------------*/
void
blink(uint16_t times)
{
  while(times) {
    LED_ON(LED_STATUS);
    __delay_cycles(MCLK_SPEED / 10);
    LED_OFF(LED_STATUS);
    __delay_cycles(MCLK_SPEED / 10);
    times--;
  }
}
/*---------------------------------------------------------------------------*/
PROCESS(app_post, "Sniffer");
AUTOSTART_PROCESSES(&app_post);
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(app_post, ev, data) 
{
  PROCESS_BEGIN();
  
  /* compile time checks */
  if(sizeof(dpp_message_t) != DPP_MSG_PKT_LEN) {
    DEBUG_PRINT_FATAL("invalid DPP message size");
  }
  DEBUG_PRINT_MSG_NOW("Process '%s' started", app_post.name);
  
  /* --- initialization --- */

  /* start the preprocess and LWB threads */
  elwb_start(0, &app_post);
  
  /* --- start of application main loop --- */
  while(1) {
    /* the app task should not do anything until it is explicitly granted 
     * permission (by receiving a poll event) by the LWB task */
    PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_POLL);
    
    const elwb_stats_t* stats = elwb_get_stats();
    if(stats->glossy_snr > 50) {
      blink(4);
    } else if(stats->glossy_snr > 40) {
      blink(3);
    } else if(stats->glossy_snr > 30) {
      blink(2);
    } else if(stats->glossy_snr > 20) {
      blink(1);
    }
    
    /* --- poll the debug task --- */
    debug_print_poll();
  } 
  /* --- end of application main loop --- */

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
