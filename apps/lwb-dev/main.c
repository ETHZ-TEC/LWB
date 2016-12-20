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
 * @brief Low-Power Wireless Bus Development Application (ETZ Test deployment)
 */

#include "main.h"

/*---------------------------------------------------------------------------*/
/* global variables */
uint16_t seq_no_lwb  = 0;
uint16_t seq_no_bolt = 0;
config_t cfg;
/*---------------------------------------------------------------------------*/
PROCESS(app_process, "Application Task");
AUTOSTART_PROCESSES(&app_process);
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(app_process, ev, data) 
{
  PROCESS_BEGIN();

  /* --- check parameters ---
   * (for debugging only, compiler will exclude this code from the assembly) */

  if(sizeof(comm_health_t) > MSG_PAYLOAD_LEN ||
     sizeof(comm_cmd_t) > MSG_PAYLOAD_LEN    ||
     sizeof(node_info_t) > MSG_PAYLOAD_LEN   ||
     sizeof(log_event_t) > MSG_PAYLOAD_LEN   ||
     sizeof(message_t) != MSG_PKT_LEN) {
    DEBUG_PRINT_MSG_NOW("ERROR: invalid struct message_t");
  }

  /* --- initialization --- */

#if DEBUG_INTERRUPT_ENABLE
  PIN_CFG_INT_INV(DEBUG_INTERRUPT_PIN);  /* enable ISR for debugging! */
#endif /* DEBUG_INTERRUPT_ENABLE */

  /* init the ADC */
  adc_init();
  REFCTL0 &= ~REFON;             /* shut down REF module to save power */

  /* start the LWB thread */
  lwb_start(0, &app_process);
  
  /* --- load the configuration --- */
  if(!nvcfg_load((uint8_t*)&cfg)) {
    DEBUG_PRINT_MSG_NOW("WARNING: failed to load config");
  }
  /* update stats and save */
  cfg.rst_cnt++;
  nvcfg_save((uint8_t*)&cfg);

  if(node_id == HOST_ID && !FLOCKLAB_SRC_NODE) {
    node_info_t tmp;
    get_node_info(&tmp);
    send_msg(DEVICE_ID_SINK, MSG_TYPE_NODE_INFO, (uint8_t*)&tmp, 0, 1);
  }

  /* --- start of application main loop --- */
  while(1) {
    /* the app task should not do anything until it is explicitly granted 
     * permission (by receiving a poll event) by the LWB task */
    PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_POLL);
#ifdef APP_TASK_ACT_PIN
    PIN_SET(APP_TASK_ACT_PIN);
#endif /* APP_TASK_ACT_PIN */
    
    if(HOST_ID == node_id && !FLOCKLAB_SRC_NODE) {
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

#ifdef APP_TASK_ACT_PIN
    PIN_CLR(APP_TASK_ACT_PIN);
#endif /* APP_TASK_ACT_PIN */
  } /* --- end of application main loop --- */

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
void
print_debug_info(uint16_t stack_addr)
{
  #define MAX_BSS_SIZE                  3504    /* heap */

  /* re-enable the HFXT, required for UART (only change necessary settings!) */
  uint16_t xt2_off = UCSCTL6 & XT2OFF;
  if(xt2_off) {
    SFRIE1  &= ~OFIE;
    ENABLE_XT2();
    WAIT_FOR_OSC();
    UCSCTL4  = SELA | SELS | SELM;
    UCSCTL7  = 0;
    WAIT_FOR_OSC();
    SFRIE1  |= OFIE;
    P1SEL    = (BIT2 | BIT3 | BIT4 | BIT5 | BIT6 | BIT7);
  }

  /* 
   * collect and print debugging info:
   * - stack address / size
   * - return address and status register before ISR (last 4 bytes on stack)
   * - whether or not timers are still running and CCR interrupts enabled
   * - state of the global/static variables
   * - some registers, e.g. enabled peripherals
   */
    
  uint16_t stack_size = SRAM_END - stack_addr + 1;
  uint8_t peripherals = ((UCA0CTL1 & UCSWRST) << 1) |
                        (UCB0CTL1 & UCSWRST);
  
  /* get more info about LWB timer */
  uint8_t lwb_timer = ((*(&TA0CCTL0 + LWB_CONF_RTIMER_ID) & CCIE) << 1) |
                   (*(&TA1CCTL0 + LWB_CONF_LF_RTIMER_ID - RTIMER_LF_0) & CCIE);
  rtimer_clock_t next_exp = 0;
  if(rtimer_next_expiration(LWB_CONF_RTIMER_ID, &next_exp)) {
    lwb_timer |= 0x02;  /* set bit */
  }
  rtimer_clock_t next_exp_lf = 0;
  if(rtimer_next_expiration(LWB_CONF_LF_RTIMER_ID, &next_exp_lf)) {
    lwb_timer |= 0x01;  /* set bit */
  }

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
  printf("\r\n-------------------------------------------------------\r\n"
         "debug info:\r\n\r\n");
  printf("stack size: %u, return addr: 0x%04x, status reg: 0x%04x\r\n"
         "usci: 0x%02x, xt2off: %u\r\n"
         "ta0ctl: 0x%x, ta1ctl: 0x%x\r\n"
         "rtimer now: %llu, %llu\r\n"
         "lwb timer: 0x%x, %llu, %llu\r\n"
         "heap:",
         stack_size, 
         *(volatile uint16_t*)(stack_addr + 2),
         *(volatile uint16_t*)(stack_addr),
         peripherals,
         xt2_off,
         TA0CTL,
         TA1CTL,
         rtimer_now_lf(), rtimer_now_hf(),
         lwb_timer, next_exp_lf, next_exp);
  
  /* print out the content of the bss section (global & static variables),
   * use objdump -t lwb.exe | grep "\.bss" to map addresses to variables! */
  uint16_t i;
  for(i = SRAM_START; i < MAX_BSS_SIZE + SRAM_START; i++) {
    if((i & 0x000f) == 0) {
      printf("\r\n0x%04x:", i);
    }
    printf(" %02x", *(uint8_t*)i);
  }
  printf("\r\n-------------------------------------------------------\r\n");
}
/*---------------------------------------------------------------------------*/
ISR(PORT2, port2_interrupt)
{
#if DEBUG_INTERRUPT_ENABLE
  LED_TOGGLE(LED_STATUS);

  /* see lwb.dis file! (pushm #6 = 6x 16-bit register is pushed onto stack) */
  #define REGISTER_BYTES_ON_STACK       (8)
  /* look into the assembly code to find out how many registers have been
   * put onto the stack since this function has been entered */
  uint16_t stack_addr;
  print_debug_info((uint16_t)&stack_addr + REGISTER_BYTES_ON_STACK + 2);

  PIN_CLR_IFG(DEBUG_INTERRUPT_PIN);
#else
  while(1);
#endif /* DEBUG_INTERRUPT_ENABLE */
}
/*---------------------------------------------------------------------------*/
/* --- for DEBUGGING: define all unused ISRs --- */

ISR(SYSNMI, sysnmi_interrupt)
{
#if DEBUG_ISR_TRAPS_ENABLE
  watchdog_stop();
  PIN_SET(LED_ERROR);
  switch (SYSSNIV) {
    case SYSSNIV_VMAIFG:    /* vacant memory access */
      while(1) { PIN_XOR(DEBUG_LED); __delay_cycles(MCLK_SPEED / 10); }
      break;
    case SYSSNIV_SVMLIFG:
      while(1) { PIN_XOR(DEBUG_LED); __delay_cycles(MCLK_SPEED / 11); }
      break;
    case SYSSNIV_SVMHIFG:
      while(1) { PIN_XOR(DEBUG_LED); __delay_cycles(MCLK_SPEED / 12); }
      break;
    case SYSSNIV_DLYLIFG:
      while(1) { PIN_XOR(DEBUG_LED); __delay_cycles(MCLK_SPEED / 13); }
      break;
    case SYSSNIV_DLYHIFG:
      while(1) { PIN_XOR(DEBUG_LED); __delay_cycles(MCLK_SPEED / 14); }
      break;
    case SYSSNIV_JMBOUTIFG:
      while(1) { PIN_XOR(DEBUG_LED); __delay_cycles(MCLK_SPEED / 15); }
      break;
    case SYSSNIV_JMBINIFG:
      while(1) { PIN_XOR(DEBUG_LED); __delay_cycles(MCLK_SPEED / 16); }
      break;
    case SYSSNIV_VLRLIFG:
      while(1) { PIN_XOR(DEBUG_LED); __delay_cycles(MCLK_SPEED / 17); }
      break;
    case SYSSNIV_VLRHIFG:
      while(1) { PIN_XOR(DEBUG_LED); __delay_cycles(MCLK_SPEED / 18); }
      break;
    default:
      while(1) { PIN_XOR(DEBUG_LED); __delay_cycles(MCLK_SPEED / 19); }
      break;
  }
#else  /* DEBUG_ISR_TRAPS_ENABLE */
  while(1);   /* wait for the watchdog to trigger a reset */
#endif /* DEBUG_ISR_TRAPS_ENABLE */
}
ISR(AES, aes_interrupt)
{
#if DEBUG_ISR_TRAPS_ENABLE
  watchdog_stop();
  PIN_SET(LED_ERROR);
  while(1) { PIN_XOR(DEBUG_LED); __delay_cycles(MCLK_SPEED / 20); }
#else  /* DEBUG_ISR_TRAPS_ENABLE */
  while(1);
#endif /* DEBUG_ISR_TRAPS_ENABLE */
}
ISR(RTC, rtc_interrupt)
{
#if DEBUG_ISR_TRAPS_ENABLE
  watchdog_stop();
  PIN_SET(LED_ERROR);
  while(1) { PIN_XOR(DEBUG_LED); __delay_cycles(MCLK_SPEED / 30); }
#else  /* DEBUG_ISR_TRAPS_ENABLE */
  while(1);
#endif /* DEBUG_ISR_TRAPS_ENABLE */
}
ISR(PORT1, p1_interrupt)
{
#if DEBUG_ISR_TRAPS_ENABLE
  watchdog_stop();
  PIN_SET(LED_ERROR);
  while(1) { PIN_XOR(DEBUG_LED); __delay_cycles(MCLK_SPEED / 40); }
#else  /* DEBUG_ISR_TRAPS_ENABLE */
  while(1);
#endif /* DEBUG_ISR_TRAPS_ENABLE */
}
ISR(ADC10, adc_interrupt)
{
#if DEBUG_ISR_TRAPS_ENABLE
  watchdog_stop();
  PIN_SET(LED_ERROR);
  while(1) { PIN_XOR(DEBUG_LED); __delay_cycles(MCLK_SPEED / 50); }
#else  /* DEBUG_ISR_TRAPS_ENABLE */
  while(1);
#endif /* DEBUG_ISR_TRAPS_ENABLE */
}
ISR(USCI_B0, ucb0_interrupt)
{
#if DEBUG_ISR_TRAPS_ENABLE
  watchdog_stop();
  PIN_SET(LED_ERROR);
  while(1) { PIN_XOR(DEBUG_LED); __delay_cycles(MCLK_SPEED / 60); }
#else  /* DEBUG_ISR_TRAPS_ENABLE */
  while(1);
#endif /* DEBUG_ISR_TRAPS_ENABLE */
}
ISR(WDT, wdt_interrupt)
{
#if DEBUG_ISR_TRAPS_ENABLE
  watchdog_stop();
  PIN_SET(LED_ERROR);
  while(1) { PIN_XOR(DEBUG_LED); __delay_cycles(MCLK_SPEED / 70); }
#else  /* DEBUG_ISR_TRAPS_ENABLE */
  while(1);
#endif /* DEBUG_ISR_TRAPS_ENABLE */
}
ISR(COMP_B, comp_interrupt)
{
#if DEBUG_ISR_TRAPS_ENABLE
  watchdog_stop();
  PIN_SET(LED_ERROR);
  while(1) { PIN_XOR(DEBUG_LED); __delay_cycles(MCLK_SPEED / 80); }
#else  /* DEBUG_ISR_TRAPS_ENABLE */
  while(1);
#endif /* DEBUG_ISR_TRAPS_ENABLE */
}
/*---------------------------------------------------------------------------*/
