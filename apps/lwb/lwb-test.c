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
 * compilation: .bss region goes into RAM (max. size 4 kB - (max_stack_size))
 */


#include "contiki.h"
#include "platform.h"
#include "log.h"


/*---------------------------------------------------------------------------*/
PROCESS(app_process, "Application Task");
AUTOSTART_PROCESSES(&app_process);
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(app_process, ev, data) 
{
  PROCESS_BEGIN();
  
  // ----- INIT -----
    
#if BOLT_CONF_ON
  //mm_init();              // message manager (only use it if not running on Flocklab)
#else
  //adc_init();
#endif // FLOCKLAB
  
  // ----- start the S-LWB thread -----
  lwb_start(&app_process);

  while (1) {
    /* the app task should not do anything until it is explicitly granted 
     * permission (by receiving a poll event) by the LWB task */
    PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_POLL);      
    
    DEBUG_PRINT_INFO("application task runs now...");
    __delay_cycles(MCLK_SPEED);
    //PROCESS_PAUSE();
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
ISR(UNMI, unmi_interrupt) 
{    
    //LED_ON(LED_0);      // error indication
    switch (SYSUNIV) {
        case SYSUNIV_NMIIFG:
            break;
        case SYSUNIV_OFIFG:
            LED_ON(LED_0);
            while(1);
            do
            {
                UCSCTL7 &= ~(XT1LFOFFG + DCOFFG + XT2OFFG);     // Clear XT1 & DCO fault flags
                SFRIFG1 &= ~OFIFG;                              // Clear OSC Fault flag
                __delay_cycles(406250);
            } while (SFRIFG1 & OFIFG);
            break;
            LED_OFF(LED_0);
        default:
            break;
    }
}
/*---------------------------------------------------------------------------*/
#ifdef PUSH_BUTTON
ISR(PORT1, port1_interrupt) 
{    
    static volatile uint8_t push_count = 0;
            
    if (PIN_IFG(PUSH_BUTTON)) {
        PIN_CLR_IFG(PUSH_BUTTON);
        
        if (push_count > 4) {
            WDTCTL &= ~WDTHOLD; /* trigger a watchdog password violation */
        } else if (push_count > 3) {
            DEBUG_PRINT_MSG_NOW("1..");
        } else if (push_count > 2) {
            DEBUG_PRINT_MSG_NOW("2..");
        } else if (push_count > 1) {
            DEBUG_PRINT_MSG_NOW("3..");
        } else if (push_count > 0) {
            DEBUG_PRINT_MSG_NOW("4..");
        } else {
            DEBUG_PRINT_MSG_NOW("Reset in 5..");
        }
        push_count++;
                
        char msg_buffer[BOLT_CONF_MAX_MSG_LEN];
        sprintf(msg_buffer, "hallo welt");
        BOLT_WRITE((uint8_t*)msg_buffer, strlen(msg_buffer));
        DEBUG_PRINT_MSG_NOW("message sent to BOLT");
    } 
}
#endif // PUSH_BUTTON
/*---------------------------------------------------------------------------*/
