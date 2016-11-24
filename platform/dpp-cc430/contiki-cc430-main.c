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
 */

#include "contiki.h"
#include "platform.h"

/*---------------------------------------------------------------------------*/
uint16_t TOS_NODE_ID = 0x1122;  /* do NOT change this default value! */
#ifndef NODE_ID
volatile uint16_t node_id;
#endif /* NODE_ID */
uint16_t rst_flag;    /* make it global to be accessible by the app task */
/*---------------------------------------------------------------------------*/
void
print_processes(struct process *const processes[])
{
  uart_enable(1);
  printf("Starting");
  while(*processes != NULL) {
    printf(" '%s'", (*processes)->name);
    processes++;
  }
  printf("\r\n");
}
/*---------------------------------------------------------------------------*/
/* prints some info about the system (e.g. MCU and reset source) */
void
print_device_info(void)
{
  const char* rst_source[14] = { "BOR", "nRST", "SWBOR", "SECV", "SVS", "SVM",
                                 "SWPOR", "WDT", "WDTPW", "KEYV", "PLLUL",
                                 "PERF", "PMMKEY", "Unknown" };
  uint8_t idx;
  /* 
   * note: this device does not offer an LPMx.5 mode, therefore there's no
   * corresponding reset source
   */
  rst_flag = SYSRSTIV; /* flag is automatically cleared by reading it */
  /* when the PMM causes a reset, a value is generated in the system reset
     interrupt vector generator register (SYSRSTIV), corresponding to the
     source of the reset */
  switch(rst_flag) {
    case SYSRSTIV_BOR:      idx = 0;  break;
    case SYSRSTIV_RSTNMI:   idx = 1;  break;
    case SYSRSTIV_DOBOR:    idx = 2;  break;
    case SYSRSTIV_SECYV:    idx = 3;  break;
    case SYSRSTIV_SVSL:
    case SYSRSTIV_SVSH:     idx = 4;  break;
    case SYSRSTIV_SVML_OVP:
    case SYSRSTIV_SVMH_OVP: idx = 5;  break;
    case SYSRSTIV_DOPOR:    idx = 6;  break;
    case SYSRSTIV_WDTTO:    idx = 7;  break;
    case SYSRSTIV_WDTKEY:   idx = 8;  break;
    case SYSRSTIV_KEYV:     idx = 9;  break; /* flash password violation */
    case SYSRSTIV_PLLUL:    idx = 10; break;
    case SYSRSTIV_PERF:     idx = 11; break;
    case SYSRSTIV_PMMKEY:   idx = 12; break;
    default:                idx = 13; break;
  }
  printf("\r\nReset Source: %s\r\nMCU: " MCU_DESC "\r\nFirmware: %u.%02u " \
         __DATE__ "\r\n", rst_source[idx], FW_VERSION >> 8, FW_VERSION & 0xff);

  /* note: KEYV indicates an incorrect FCTLx password was written to any flash
   * control register and generates a PUC when set. */
}
/*---------------------------------------------------------------------------*/
int
main(int argc, char **argv)
{
#if WATCHDOG_CONF_ON
  watchdog_init();
  watchdog_start();
#else
  watchdog_stop();
#endif /* WATCHDOG_CONF_ON */

  /* initialize hardware */

  /* set default configuration for all GPIOs */
  /* don't set P1.5/P1.6 (UART) and BOLT IND pins (P1.1 and P2.2) as output! */
  P1DIR = (BIT0 | BIT2 | BIT3 | BIT4 | BIT7);
  PORT_CLR_I(1);
  /* don't set the BOLT IND pin as outputs! */
  P2DIR = (BIT0 | BIT1 | BIT3 | BIT4 | BIT5 | BIT6 | BIT7);
  PORT_CLR_I(2);
  PORT_CFG_OUT_I(3);
  PORT_CLR_I(3);
  PORT_CFG_OUT_I(J);
  PORT_CLR_I(J);
  
  /* board-specific GPIO config */
  
  PIN_CFG_IN(BOLT_CONF_IND_PIN);
  PIN_CFG_IN(BOLT_CONF_IND_OUT_PIN);
  PIN_CFG_OUT(LED_STATUS);
  PIN_SET(LED_STATUS);
      
#ifdef MUX_SEL_PIN
  /* this board has a multiplexer (set it to UART) */
  PIN_CFG_OUT(MUX_SEL_PIN);
  PIN_SET(MUX_SEL_PIN);
#endif 

  /* pin mappings */
#ifdef RF_GDO0_PIN
  PIN_MAP_AS_OUTPUT(RF_GDO0_PIN, PM_RFGDO0);
#endif
#ifdef RF_GDO1_PIN
  PIN_MAP_AS_OUTPUT(RF_GDO1_PIN, PM_RFGDO1);
#endif
#ifdef RF_GDO2_PIN
  PIN_MAP_AS_OUTPUT(RF_GDO2_PIN, PM_RFGDO2);
#endif
#ifdef MCLK_PIN
  PIN_MAP_AS_OUTPUT(MCLK_PIN, PM_MCLK);
#endif
#ifdef SMCLK_PIN
  PIN_MAP_AS_OUTPUT(SMCLK_PIN, PM_SMCLK);
#endif
#ifdef ACLK_PIN
  PIN_MAP_AS_OUTPUT(ACLK_PIN, PM_ACLK);
#endif
  
  clock_init();
  rtimer_init();
  uart_init();
  uart_enable(1);
  uart_set_input_handler(serial_line_input_byte);
  print_device_info();
    
#if RF_CONF_ON
  /* init the radio module and set the parameters */
  rf1a_init();
#endif /* RF_CONF_ON */

  /* set the node ID */
#ifndef NODE_ID
  node_id = TOS_NODE_ID;
  printf(CONTIKI_VERSION_STRING " started. Node ID not set.\r\n");
#else /* NODE_ID */
  printf(CONTIKI_VERSION_STRING " started. Node ID is set to %u.\r\n",
         node_id);
#endif /* NODE_ID */

#if FRAM_CONF_ON
  if (!fram_init()) {
    DEBUG_PRINT_FATAL("ERROR: FRAM failure");
  }
#endif
#if BOLT_CONF_ON
  bolt_init(0);
#endif /* BOLT_CONF_ON */

#if SVS_CONF_ON
  SVS_ENABLE;
#else
  SVS_DISABLE;
#endif /* SVS_CONF_ON */

  process_init();
  process_start(&etimer_process, NULL);

  random_init(node_id * TA0R);
  serial_line_init();
  /* note: do not start the debug process here */

  energest_init();
  ENERGEST_ON(ENERGEST_TYPE_CPU);
  DCSTAT_CPU_ON;

#if NULLMAC_CONF_ON
  nullmac_init();
#endif /* NULLMAC_CONF_ON */
  
  /* start processes */
  print_processes(autostart_processes);
  autostart_start(autostart_processes);
  debug_print_init();
  /* note: start debug process as last due to process_poll() execution order */
  
  PIN_CLR(LED_STATUS);     /* init done */

  while(1) {
    int r;
    do {
#if WATCHDOG_CONF_ON
      watchdog_reset();
#endif /* WATCHDOG_CONF_ON */
      r = process_run();
    } while(r > 0);
    /* idle processing */
    /* disable interrupts */
    __dint();
    __nop();
    if(process_nevents() != 0 || UART_ACTIVE) {
      /* re-enable interrupts */
      __eint();
      __nop();
    } else {
      /* re-enable interrupts and go to sleep atomically */
      ENERGEST_OFF(ENERGEST_TYPE_CPU);
      DCSTAT_CPU_OFF;
#if WATCHDOG_CONF_ON && !WATCHDOG_RESET_ON_TA1IFG
      /* no need to stop the watchdog in the low-power mode if it is reset
       * within the timer update interrupt (which occurs every 2 seconds) */
      watchdog_stop();
#endif /* WATCHDOG_CONF_ON */
      /* LPM3 */
      __bis_status_register(GIE | SCG0 | SCG1 | CPUOFF);
      __no_operation();
#if WATCHDOG_CONF_ON && !WATCHDOG_RESET_ON_TA1IFG
      watchdog_start();
#endif /* WATCHDOG_CONF_ON */
      DCSTAT_CPU_ON;
      ENERGEST_ON(ENERGEST_TYPE_CPU);
    }
  }

  return 0;
}
/*---------------------------------------------------------------------------*/
