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
 *          Tonio Gsell
 */

/**
 * @brief eLWB Development Application (ETZ Test deployment)
 */

#include "main.h"

/*---------------------------------------------------------------------------*/
#ifdef APP_TASK_ACT_PIN
 #define APP_TASK_ACTIVE    PIN_SET(APP_TASK_ACT_PIN)
 #define APP_TASK_INACTIVE  PIN_CLR(APP_TASK_ACT_PIN)
#else 
 #define APP_TASK_ACTIVE
 #define APP_TASK_INACTIVE
#endif /* APP_TASK_ACT_PIN */
/*---------------------------------------------------------------------------*/
/* global variables */
static int64_t  captured = 0;     /* last captured timestamp */
static uint32_t t_last_health_pkt = 0;
static uint16_t max_stack_size = 0;
uint16_t seq_no_lwb  = 0;
uint16_t seq_no_bolt = 0;
config_t cfg;
/*---------------------------------------------------------------------------*/
void
capture_timestamp(void)
{ 
  /* simply store the timestamp, do calculations afterwards */
  rtimer_clock_t now = rtimer_now_lf();
  captured = now - ((uint16_t)(now & 0xffff) - BOLT_CONF_TIMEREQ_CCR);
}
/*---------------------------------------------------------------------------*/
PROCESS(app_proc_pre, "App Task Pre");
PROCESS_THREAD(app_proc_pre, ev, data) 
{  
  PROCESS_BEGIN();
  
  while(1) {  
    APP_TASK_INACTIVE;
    PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_POLL);
    APP_TASK_ACTIVE;
    
    AFTER_DEEPSLEEP();    /* restore all clocks */
          
    /* --- read messages from the BOLT queue and forward them to the LWB --- */
    uint8_t  bolt_buffer[BOLT_CONF_MAX_MSG_LEN];
    uint16_t msg_cnt = 0;
    while(BOLT_DATA_AVAILABLE &&
          (lwb_get_send_buffer_state() < LWB_CONF_OUT_BUFFER_SIZE)) {
      if(bolt_read(bolt_buffer)) {
        msg_cnt++;
        process_message((dpp_message_t*)bolt_buffer, 1);
      } /* else: invalid message received from BOLT */
    }
    if(msg_cnt) {
      DEBUG_PRINT_INFO("%u message(s) read from BOLT", msg_cnt);
    }
  }
  
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
PROCESS(app_proc_post, "App Task Post");
AUTOSTART_PROCESSES(&app_proc_post);
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(app_proc_post, ev, data) 
{
  PROCESS_BEGIN();

  /* --- initialization --- */

  /* init the ADC */
  adc_init();
  REFCTL0 &= ~REFON;             /* shut down REF module to save power */

  /* start the preprocess and LWB threads */
  lwb_start(&app_proc_pre, &app_proc_post);
  process_start(&app_proc_pre, NULL);
  
  /* --- load the configuration --- */
  if(!nvcfg_load((uint8_t*)&cfg)) {
    DEBUG_PRINT_MSG_NOW("WARNING: failed to load config");
  }
  /* update stats and save */
  cfg.rst_cnt++;
  nvcfg_save((uint8_t*)&cfg);
  
  /* enable the timestamp request interrupt */
  bolt_set_timereq_callback(capture_timestamp);

  /* generate a node info message (will be sent out as soon as there is
   * network connectivity) */
  send_node_info();

  /* --- start of application main loop --- */
  while(1) {
    /* the app task should not do anything until it is explicitly granted 
     * permission (by receiving a poll event) by the LWB task */
    APP_TASK_INACTIVE;
    PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_POLL);
    APP_TASK_ACTIVE;
        
    /* --- process messages received via network --- */
    dpp_message_t msg;
    uint16_t msg_cnt = 0;

    /* --- process all packets received from the network (forward to BOLT) --- */
    while(lwb_rcv_pkt((uint8_t*)&msg, 0, 0)) {
      process_message(&msg, 0);
      msg_cnt++;
    }
    if(msg_cnt) {
      DEBUG_PRINT_INFO("%d packet(s) received", msg_cnt);
    }
    
    /* --- send the timestamp if one has been requested --- */
    if(captured) {
      send_timestamp(captured);
    }
      
    /* --- generate a new health message if necessary --- */
    uint32_t time_now = lwb_get_time(0);
    if((time_now - t_last_health_pkt) >= health_msg_period) {
      send_node_health();
      t_last_health_pkt = time_now;
      DEBUG_PRINT_INFO("health message generated");
    }
    
    /* --- debugging --- */
    uint16_t stack_size = debug_print_get_max_stack_size();
    if(stack_size > max_stack_size) {
      max_stack_size = stack_size;
      DEBUG_PRINT_INFO("stack size: %uB, max %uB", (SRAM_START + SRAM_SIZE) -
                                            (uint16_t)&stack_size, stack_size);
    }
    
    /* --- poll the debug task --- */
    debug_print_poll();
    process_poll(&app_proc_post);
    APP_TASK_INACTIVE;
    PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_POLL);
    APP_TASK_ACTIVE;
    
    /* since this process is only executed at the end of an LWB round, we 
     * can now configure the MCU for minimal power dissipation for the idle
     * period until the next round starts */
    BEFORE_DEEPSLEEP();
  } /* --- end of application main loop --- */

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
