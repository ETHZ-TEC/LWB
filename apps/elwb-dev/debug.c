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

/* --- DEBUGGING, trap program in an unexpected interrupt ---

  To trap in WDT ISR, set the following flags in config.h:

  #define WATCHDOG_CONF_RESET_ON_TA1IFG  0
  #define WATCHDOG_CONF_STOP_IN_LPM      0
  #define WATCHDOG_CONF_TIMER_MODE       1 

  To enable the trap within other interrupts, the GIE bit must be set upon
  entering the ISR:

  __eint(); __nop();


  To dump all the debug info, connect the serial cable (all 3 pins) and press
  any key.

*/


#include "main.h"

/*---------------------------------------------------------------------------*/
#define BLINK_LED(div)      while(1) { \
                              LED_TOGGLE(LED_STATUS); \
                              __delay_cycles(MCLK_SPEED / div); \
                            }
/*---------------------------------------------------------------------------*/
void
dump_debug_info(uint16_t stack_addr)
{
  /* collect and print debugging info:
   * - stack address / size
   * - return address and status register before ISR (last 4 bytes on stack)
   * - whether or not timers are still running and CCR interrupts enabled
   * - state of the global/static variables
   * - some registers, e.g. enabled peripherals
   */
  uint16_t xt2_off  = (UCSCTL6 & XT2OFF) > 0;
  uint16_t ta0_ctl  = TA0CTL;
  uint16_t ta1_ctl  = TA1CTL;
  uint8_t  uca0_ctl = UCA0CTL1;
  uint8_t  ucb0_ctl = UCB0CTL1;
  rtimer_clock_t now_lf = rtimer_now_lf();
  rtimer_clock_t now_hf = rtimer_now_hf();

  rtimer_clock_t next_exp_hf = 0;
  uint8_t hf_scheduled = rtimer_next_expiration(ELWB_CONF_RTIMER_ID,
                                                &next_exp_hf);
  rtimer_clock_t next_exp_lf = 0;
  uint8_t lf_scheduled = rtimer_next_expiration(ELWB_CONF_LF_RTIMER_ID,
                                                &next_exp_lf);

  uint8_t gpio_state = ((PIN_GET(COM_GPIO1) > 0) << 2) |
                       ((PIN_GET(COM_GPIO2) > 0) << 1) |
                       (PIN_GET(COM_GPIO3) > 0);
  
  /* re-enable the HFXT, required for UART (only change necessary settings!) */
  AFTER_DEEPSLEEP();
  
  /* wait until there is a falling edge on P1.5 (UART RXD) */
  P1DIR &= ~BIT5;
  P1OUT |= BIT5;
  P1REN |= BIT5;
  while (P1IN & BIT5);
  
  /* status register bits:
   * 8 = 0x100 = arithmetic overflow
   * 7 = 0x80 = SCG1 (system clock generator 1 off)
   * 6 = 0x40 = SCG0
   * 5 = 0x20 = OSCOFF (oscillator off, turns off LFXT)
   * 4 = 0x10 = CPUOFF
   * 3 = 0x08 = GIE
   * 2 = 0x04 = N (result of last operation was negative)
   * 1 = 0x02 = Z (set if result of last operation was zero)
   * 0 = 0x01 = C (carry bit, set if result of last op. produced a carry) */
  
  /* print out the information */
  uart_enable(1);
  printf("\r\n-------------------------------------------------------\r\n");
  printf("stack size:   %u\r\n"
         "return addr:  0x%04x\r\n"
         "status reg:   0x%04x\r\n"
         "UCA0CTL1:     0x%02x\r\n"
         "UCB0CTL1:     0x%02x\r\n"
         "XT2OFF:       %u\r\n"
         "TA0CTL:       0x%04x (HFXT)\r\n"
         "TA1CTL:       0x%04x (LFXT)\r\n"
         "rtimer LF:    %llu\r\n"
         "rtimer HF:    %llu\r\n"
         "LWB timer LF: %llu (%u)\r\n"
         "LWB timer HF: %llu (%u)\r\n"
         "Glossy state: %u\r\n"
         "GPIO state:   0x%02x\r\n",
              SRAM_END - stack_addr + 1, 
              *(volatile uint16_t*)(stack_addr + 2),
              *(volatile uint16_t*)(stack_addr),
              uca0_ctl,
              ucb0_ctl,
              xt2_off,
              ta0_ctl,
              ta1_ctl,
              now_lf,
              now_hf,
              next_exp_lf,
              lf_scheduled,
              next_exp_hf,
              hf_scheduled,
              glossy_is_active(),
              gpio_state);
  printf("\r\nSRAM dump:");
  /* print out the content of the bss section (global & static variables),
   * use objdump -t lwb.exe | grep "\.bss" to map addresses to variables! */
  uint16_t i;
  for(i = SRAM_START; i < SRAM_START + SRAM_SIZE; i++) {
    if((i & 0x000f) == 0) {
      printf("\r\n0x%04x:", i);
    }
    printf(" %02x", *(uint8_t*)i);
  }
  printf("\r\n-------------------------------------------------------\r\n");
}
/*---------------------------------------------------------------------------*/
ISR(PORT1, port1_interrupt)
{
  AFTER_DEEPSLEEP();
  DEBUG_PRINT_MSG_NOW("P1 ISR");
  P1IFG &= ~BIT5;
}
/*---------------------------------------------------------------------------*/
#if WATCHDOG_CONF_TIMER_MODE
ISR(WDT, wdt_interrupt)
{
  watchdog_stop();
  LED_ON(LED_ERROR);
  
  /* see lwb.dis file! (pushm #4 = 4x 16-bit register is pushed onto stack) */
  #define REGISTER_BYTES_ON_STACK       (8)
  /* look into the assembly code to find out how many registers have been
   * put onto the stack since this function has been entered */
  uint16_t stack_addr;
  dump_debug_info((uint16_t)&stack_addr + REGISTER_BYTES_ON_STACK + 2);
}
#endif /* WATCHDOG_CONF_TIMER_MODE */
/*---------------------------------------------------------------------------*/
