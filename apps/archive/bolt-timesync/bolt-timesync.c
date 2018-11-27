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
 * @brief BOLT timesync demo (high accuracy)
 * 
 * This demo application demonstrates how to use the time synchronization 
 * feature of BOLT on the Dual Processor Platform (DPP). The HF timer is used
 * to achieve higher accuracy.
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
#define IS_HOST                 (HOST_ID == NODE_ID)
/*---------------------------------------------------------------------------*/
PROCESS(app_process, "Application Task");
AUTOSTART_PROCESSES(&app_process);
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(app_process, ev, data) 
{ 
  PROCESS_BEGIN();

  /* start the LWB thread */
  lwb_start(0, &app_process);

  /* main loop of this application task */
  while(1) {
    /* the app task should not do anything until it is explicitly granted 
     * permission (by receiving a poll event) by the LWB task */
    PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_POLL);
    TASK_ACTIVE;      /* application task runs now */

    rtimer_clock_t bolt_timestamp = 0;
    if(bolt_handle_timereq(&bolt_timestamp)) {
  #if IS_HOST
      /* simply print out the timestamp */
      DEBUG_PRINT_MSG_NOW("timestamp: %llu", bolt_timestamp);
  #else /* !IS_HOST */
      /* get the time from the LWB */
      rtimer_clock_t local_rx_time = 0;
      uint32_t lwb_time_secs = lwb_get_time(&local_rx_time);
      /* convert the timestamp to global (LWB) time */
      int64_t lwb_time  = (uint64_t)lwb_time_secs * RTIMER_SECOND_HF;
      int64_t ofs        = lwb_time - local_rx_time;
      int32_t elapsed    = (int64_t)bolt_timestamp - local_rx_time;
      /* use the average drift */
      int16_t drift = lwb_get_stats()->drift;
      int32_t drift_comp = (elapsed / 16) * (int32_t)drift /
                           (int32_t)RTIMER_SECOND_HF;
      bolt_timestamp     = bolt_timestamp + ofs + drift_comp + TIMESYNC_OFS;
      DEBUG_PRINT_MSG_NOW("timestamp: %llu", bolt_timestamp);
      BOLT_WRITE((uint8_t*)&bolt_timestamp, 8);
  #endif /* IS_HOST */
    }
    
    TASK_SUSPENDED;
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
