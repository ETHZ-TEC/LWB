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
 * @note
 * Notes on Processes in Contiki: 
 * 
 * The only process control function that can be called from preemptive
 * mode (ISR) is process_poll().
 * A process can send a PROCESS_EVENT_MSG event asynchronously to some other
 * process, with a pointer to the message:
 *   process_post(&example_process, PROCESS_EVENT_CONTINUE, msg);
 * A msg can also be passed 
 * synchronously with:
 *   process_post_synch(&example_process, PROCESS_EVENT_MSG, msg);
 * 
 * A process can yield to other processes by calling PROCESS_YIELD(), in which
 * case it will wait for any event (equivalent to PROCESS_WAIT_EVENT()).
 * Alternatively, use PROCESS_WAIT_EVENT(), PROCESS_WAIT_EVENT_UNTIL() or 
 * PROCESS_WAIT_UNTIL() or PROCESS_PAUSE().
 * 
 * Remark: Protothreads are stackless threads optimized for event-driven OSs.
 * They are basically a function.
 * A process can react to events. 
 * 
 * @remarks 
 * - use "msp430-objdump -h <filename>" to print out all linker sections, their location and size; besides, gcc outputs the size of the RO (.text) and RAM (.bss + .data) sections
 * - make sure the unused RAM space for the stack is at least 100 B (check .bss section) -> 4 kB - .bss region - heap (should be 0) = max_stack_size
 * - the max. period may be much longer than 30s, test showed that even 30min are feasible (note though that the period is a uint8 variable with the last bit reserved)
 * - sprintf is very inefficient (~2000 cycles to copy a string) -> use memcpy instead
 * - structs always cause problems: misalignments due to "compiler optimizations", use uintx_t types instead of enum and union instead of conversion to a pointer to a struct
 */


#include "contiki.h"
#include "platform.h"

/*---------------------------------------------------------------------------*/
#define IS_HOST         (node_id == HOST_ID)
/*---------------------------------------------------------------------------*/
PROCESS(app_process, "Application Task");
AUTOSTART_PROCESSES(&app_process);
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(app_process, ev, data) 
{    
    static uint8_t stream_state = 0;
    
    PROCESS_BEGIN();
  
  /* all the necessary initialization is done in contiki-cc430-main.c */
  
  if(!IS_HOST) {
    adc_init();
  }
  
  /* start the LWB thread */
  lwb_start(0, &app_process);
  
  /* main loop of this application task */
  while(1) {
    /* the app task should not do anything until it is explicitly granted 
     * permission (by receiving a poll event) by the LWB task */
    PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_POLL);      
    /*DEBUG_PRINT_INFO("application task runs now...");*/
    
    if(IS_HOST) {
      /* we are the host */
      /* print out the received data */
      uint8_t pkt_buffer[LWB_CONF_MAX_DATA_PKT_LEN];
      uint16_t sender_id;
      uint8_t pkt_len = lwb_get_data(pkt_buffer, &sender_id, 0);
      if(pkt_len) {
        DEBUG_PRINT_INFO("data packet received from node %u: "
                         "%u°C, %umV", 
                         sender_id, pkt_buffer[0], 
                         (uint16_t)pkt_buffer[1] * 4 + 2000);
      } 
    } else {
      /* we are a source node */
      if(stream_state != LWB_STREAM_STATE_ACTIVE) {
        stream_state = lwb_stream_get_state(1);
        if(stream_state == LWB_STREAM_STATE_INACTIVE) {
          /* request a stream with ID 1 and IPI 5 */
          lwb_stream_req_t my_stream = { node_id, 0, 1, 20 };
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
        } else {
          DEBUG_PRINT_INFO("data packet passed to the LWB (%u, %u)", 
                           data[0], data[1]);
        }
      }        
    }
    /* output some info */
    /*DEBUG_PRINT_INFO("RSSI value: %u (last: %u)", rf1a_get_rssi(), 
                                                rf1a_get_last_packet_rssi());*/
    /*DELAY(100);*/
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
