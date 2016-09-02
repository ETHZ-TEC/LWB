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
 * @brief Low-Power Wireless Bus Test Application
 * 
 * A simple range test application. Each source node sends some status data
 * (RSSI, battery voltage, temperature, ...) to the host in each round.
 */


#include "main.h"

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
  PROCESS_BEGIN();

  /* init */
  if(HOST_ID == node_id) {
	  host_init();
  } else {
	  source_init();
  }

  /* start the LWB thread */
  lwb_start(0, &app_process);
  
  /* --- start of application main loop --- */
  while(1) {
    /* the app task should not do anything until it is explicitly granted 
     * permission (by receiving a poll event) by the LWB task */
    PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_POLL);
    TASK_ACTIVE;      /* application task runs now */
    
    if(HOST_ID == node_id) {
      host_run();
    } else {
      source_run();
    }

    /* since this process is only executed at the end of an LWB round, we 
     * can now configure the MCU for minimal power dissipation for the idle
     * period until the next round starts */
#if LWB_CONF_USE_LF_FOR_WAKEUP
    LWB_BEFORE_DEEPSLEEP();
#endif /* LWB_CONF_USE_LF_FOR_WAKEUP */
    
    TASK_SUSPENDED;
  } /* --- end of application main loop --- */

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
#if DEBUG_PORT2_INT
ISR(PORT2, port2_interrupt) 
{    
  PIN_XOR(LED_STATUS);  /* toggle LED */
  
  /* 
   * collect and print debugging info:
   * - stack address / size
   * - return address and status register before ISR (last 32 bits on stack)
   * - whether or not timers are still running and CCR interrupts enabled
   * - state of the global/static variables
   * - some registers, e.g. enabled peripherals
   */
  #define REGISTER_BYTES_ON_STACK       16       /* see lwb.dis file! */
  #define MAX_BSS_SIZE                  3072
    
  uint16_t stack_addr;
  uint16_t stack_size = SRAM_END - (uint16_t)&stack_addr + 1;
  uint8_t peripherals = ((UCA0CTL1 & UCSWRST) << 7) | 
                        ((UCB0CTL1 & UCSWRST) << 6) | 
                        (TA0CTL & (MC_3 | TAIE))    |
                        ((TA1CTL & MC_3) >> 2)      |
                        ((TA1CTL & TAIE) >> 1);

  /* look into the assembly code to find out how many registers have been 
   * put onto the stack since this function has been entered */
  uint16_t sr_addr  = (uint16_t)&stack_addr + REGISTER_BYTES_ON_STACK;
  
  /* status register bits:
   * 8 = arithmetic overflow
   * 7 = SCG1 (system clock generator 1 off)
   * 6 = SCG0
   * 5 = OSCOFF (oscillator off, turns off LFXT)
   * 4 = CPUOFF
   * 3 = GIE
   * 2 = N (result of last operation was negative)
   * 1 = Z (set if result of last operation was zero)
   * 0 = C (carry bit, set if result of last operation produced a carry) */
  
  /* print out the information */
  uart_enable(1);
  printf("stack: %u, ret: 0x%04x, sr: 0x%04x, peri: 0x%02x, lwb_ccr: 0x%x\r\n",
         stack_size, 
         *(volatile uint16_t*)sr_addr, 
         *(volatile uint16_t*)(sr_addr + 2), 
         peripherals,
         ((*(&TA0CCTL0 + LWB_CONF_RTIMER_ID) & CCIE) >> 3) |
           ((*(&TA1CCTL0 + LWB_CONF_LF_RTIMER_ID - RTIMER_LF_0) & CCIE) >> 4));
  
  /* print out the content of the bss section (global & static variables),
   * use objdump -t lwb.exe | grep "\.bss" to map addresses to variables! */
  uint16_t i;
  for(i = SRAM_START; i < MAX_BSS_SIZE + SRAM_START; i++) {
    if((i & 0x000f) == 0) {
      printf("\r\n0x%04x:", i);
    }
    printf(" %02x", *(uint8_t*)i);
  }

  PIN_CLR_IFG(BOLT_CONF_IND_PIN);
}
#endif /* DEBUG_PORT2_INT */
/*---------------------------------------------------------------------------*/
/* for debugging: define all unused ISRs */
ISR(SYSNMI, sysnmi_interrupt)
{
  PIN_SET(LED_0);
  switch (SYSSNIV) {
    case SYSSNIV_VMAIFG:
      while(1) { PIN_XOR(COM_MCU_INT1); __delay_cycles(MCLK_SPEED / 15); }
      break;
    default:
        break;
  }
  while(1) { PIN_XOR(COM_MCU_INT1); __delay_cycles(MCLK_SPEED / 10); }
}
ISR(AES, aes_interrupt)
{
  PIN_SET(LED_0); 
  while(1) { PIN_XOR(COM_MCU_INT1); __delay_cycles(MCLK_SPEED / 20); }    
}
ISR(RTC, rtc_interrupt)
{
  PIN_SET(LED_0); 
  while(1) { PIN_XOR(COM_MCU_INT1); __delay_cycles(MCLK_SPEED / 30); }   
}
ISR(PORT1, p1_interrupt)
{
  PIN_SET(LED_0); 
  while(1) { PIN_XOR(COM_MCU_INT1); __delay_cycles(MCLK_SPEED / 40); }   
}
ISR(ADC10, adc_interrupt)
{
  PIN_SET(LED_0); 
  while(1) { PIN_XOR(COM_MCU_INT1); __delay_cycles(MCLK_SPEED / 50); }    
}
ISR(USCI_B0, ucb0_interrupt)
{
  PIN_SET(LED_0); 
  while(1) { PIN_XOR(COM_MCU_INT1); __delay_cycles(MCLK_SPEED / 60); }    
}
ISR(WDT, wdt_interrupt)
{
  PIN_SET(LED_0); 
  while(1) { PIN_XOR(COM_MCU_INT1); __delay_cycles(MCLK_SPEED / 70); }    
}
ISR(COMP_B, comp_interrupt)
{
  PIN_SET(LED_0); 
  while(1) { PIN_XOR(COM_MCU_INT1); __delay_cycles(MCLK_SPEED / 80); }    
}
/*---------------------------------------------------------------------------*/
