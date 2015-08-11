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
uint16_t TOS_NODE_ID = 0x1122;
volatile uint16_t node_id;
/*---------------------------------------------------------------------------*/
static void
print_processes(struct process *const processes[])
{
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
print_info(void)
{
  static uint32_t rst_flag;         /* flag is automatically cleared by reading
                                       it, therefore store its value */
  rst_flag = SYSRSTIV;

  printf("\r\n\r\nReset Source: ");
  /* when the PMM causes a reset, a value is generated in the system reset
     interrupt vector generator register (SYSRSTIV), corresponding to the
     source of the reset */
  if(SYSRSTIV_BOR == rst_flag) {         /* do not use switch .. case due to
                                            contiki! */
    printf("BOR");        /* brownout reset (BOR) */
  } else if(SYSRSTIV_RSTNMI == rst_flag) {
    printf("Reset Pin");
  } else if(SYSRSTIV_DOBOR == rst_flag) {
    printf("Software BOR");
  } else if(SYSRSTIV_LPM5WU == rst_flag) {
    printf("Wake-up from LPMx.5");
  } else if(SYSRSTIV_SECYV == rst_flag) {
    printf("Security violation");
  } else if(SYSRSTIV_SVSL == rst_flag || SYSRSTIV_SVSH == rst_flag) {
    printf("SVS");
  } else if(SYSRSTIV_SVML_OVP == rst_flag || SYSRSTIV_SVMH_OVP == rst_flag) {
    printf("SVM");
  } else if(SYSRSTIV_DOPOR == rst_flag) {
    printf("Software POR");
  } else if(SYSRSTIV_WDTTO == rst_flag) {
    printf("Watchdog timeout");
  } else if(SYSRSTIV_WDTKEY == rst_flag) {
    printf("Watchdog password violation");
  } else if(SYSRSTIV_KEYV == rst_flag) {
    printf("KEYV");
  } else if(SYSRSTIV_PLLUL == rst_flag) {
    printf("PLLUL");
  } else if(SYSRSTIV_PERF == rst_flag) {
    printf("Peripheral area fetch");
  } else if(SYSRSTIV_PMMKEY == rst_flag) {
    printf("PMMKEY");
  } else {
    printf("Unknown");
  }
  printf("\r\nMCU: " MCU_TYPE " (%d kB RAM, %d kB Flash, %u kB used)\r\n",
         SRAM_SIZE >> 10,
         FLASH_SIZE >> 10,
         flash_code_size() >> 10);
  printf("Compiler: " COMPILER_INFO "\r\nDate: " COMPILE_DATE "\r\n");
}
/*---------------------------------------------------------------------------*/
int
main(int argc, char **argv)
{
  watchdog_stop();
  watchdog_init();

  /* initialize hardware */

  /* default config for all pins: port function, output direction */
  PORT_UNSEL_I(1);
  PORT_CFG_OUT_I(1);
  PORT_CLR_I(1);
  PORT_CLR_IFG_I(1);
  PORT_UNSEL_I(2);
  PORT_CFG_OUT_I(2);
  PORT_CLR_I(2);
  PORT_CLR_IFG_I(2);
  PORT_UNSEL_I(3);
  PORT_CFG_OUT_I(3);
  PORT_CLR_I(3);
  PORT_CFG_OUT_I(J);
  PORT_CLR_I(J);
    
  /* board-specific optimal configuration of unused pins */
  PIN_SET_I(1, 1);    /* push-button, tied to 3V */
  PIN_SET_I(1, 6);    /* UART TX, set high if pin is in use */
  PIN_SET_I(1, 7);    /* SPI B0 STE (is tied to 3V) */
  /*PIN_SET_I(2, 0);*/    /* tied to 3V -> 0R resistor not mounted */
  /*PIN_SET_I(2, 1);*/    /* tied to 3V -> 0R resistor not mounted */

  LEDS_INIT;
  LEDS_ON;
  
#if defined(RF_GDO2_PIN) && defined(USE_LEDS)
  PIN_MAP_AS_OUTPUT(RF_GDO2_PIN, PM_RFGDO2);
#endif
#ifdef PUSH_BUTTON
  PIN_UNSEL(PUSH_BUTTON);
  PIN_CFG_INT(PUSH_BUTTON);
#endif

  clock_init();
  rtimer_init();
  uart_init();
  UART_ENABLE;
  uart_set_input_handler(serial_line_input_byte);
  print_info();

#ifdef WITH_RADIO
  rf1a_init();
  rf1a_set_tx_power(RF1A_CONF_TX_POWER);
  rf1a_set_channel(RF1A_CONF_TX_CHANNEL);
  rf1a_set_maximum_packet_length(RF1A_CONF_MAX_PKT_LEN);
#endif /* WITH_RADIO */

#if FRAM_CONF_ON
  fram_init();
#endif
  
#if BOLT_CONF_ON
  bolt_init();
#endif /* BOLT_CONF_ON */
  
  /* set the node ID */
#ifdef NODE_ID
  node_id = NODE_ID;
#else
  node_id = TOS_NODE_ID;
#endif /* NODE_ID */

  printf("\r\n" CONTIKI_VERSION_STRING " started. ");
  if(node_id > 0) {
    printf("Node ID is set to %u.\r\n", node_id);
  } else {
    printf("Node ID is not set.\r\n");
  }
  print_processes(autostart_processes);

  process_init();
  process_start(&etimer_process, NULL);

  random_init(node_id * TA0R);
  serial_line_init();
  /* note: do not start the debug process here */

  energest_init();
  ENERGEST_ON(ENERGEST_TYPE_CPU);

#ifdef WITH_NULLMAC
  nullmac_init();
#endif /* WITH_NULLMAC */

#if WATCHDOG_CONF_ON
  watchdog_start();
#endif /* WATCHDOG_CONF_ON */

  LEDS_OFF;     /* init done */
  
  /* start processes */
  debug_print_init();
  
  __eint();
  autostart_start(autostart_processes);
    

  while(1) {
    int r;
    do {
#if WATCHDOG_CONF_ON
      /* reset the watchdog */
      watchdog_periodic();
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
#if WATCHDOG_CONF_ON
      watchdog_stop();
#endif /* WATCHDOG_CONF_ON */
      /* LPM3 */
      __bis_status_register(GIE | SCG0 | SCG1 | CPUOFF);
      __no_operation();
#if WATCHDOG_CONF_ON
      watchdog_start();
#endif /* WATCHDOG_CONF_ON */
      ENERGEST_ON(ENERGEST_TYPE_CPU);
    }
  }

  return 0;
}
/*---------------------------------------------------------------------------*/
