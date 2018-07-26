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
uint16_t TOS_NODE_ID = NODE_ID;             /* required for tos-set-symbols! */
//#ifndef NODE_ID
volatile uint16_t node_id;
//#endif /* NODE_ID */
uint16_t rst_flag;        /* make it global to be accessible by the app task */
/*---------------------------------------------------------------------------*/
/* prints some info about the system (e.g. MCU and reset source) */
void
print_device_info(void)
{
  const char* rst_source[14] = { "BOR", "nRST", "SWBOR", "SECV", "SVS", "SVM",
                                 "SWPOR", "WDT", "WDTPW", "KEYV", "PLLUL",
                                 "PERF", "PMMKEY", "?" };
  uint8_t idx;
  /* when the PMM causes a reset, a value is generated in the system reset
     interrupt vector generator register (SYSRSTIV)
     reset sources 2 - 10 generate a BOR, 12 - 20 a POR and 22 - 32 a PUC */
  switch(rst_flag) {
    case SYSRSTIV_BOR:      idx = 0;  break; /* 2 brownout reset */
    case SYSRSTIV_RSTNMI:   idx = 1;  break; /* 4 reset pin */
    case SYSRSTIV_DOBOR:    idx = 2;  break; /* 6 software BOR */
    case SYSRSTIV_SECYV:    idx = 3;  break; /* 10 security violation */
    case SYSRSTIV_SVSL:                      /* 12 supply voltage supervisor */
    case SYSRSTIV_SVSH:     idx = 4;  break; /* 14 supply voltage supervisor */
    case SYSRSTIV_SVML_OVP:                  /* 16 */
    case SYSRSTIV_SVMH_OVP: idx = 5;  break; /* 18 */
    case SYSRSTIV_DOPOR:    idx = 6;  break; /* 20 (software reset) */
    case SYSRSTIV_WDTTO:    idx = 7;  break; /* 22 watchdog timeout */
    case SYSRSTIV_WDTKEY:   idx = 8;  break; /* 24 watchdog PW violation */
    case SYSRSTIV_KEYV:     idx = 9;  break; /* 26 flash password violation */
    case SYSRSTIV_PLLUL:    idx = 10; break; /* 28 */
    case SYSRSTIV_PERF:     idx = 11; break; /* 30 peripheral area fetch */
    case SYSRSTIV_PMMKEY:   idx = 12; break; /* 32 PMM PW violation */
    default:                idx = 13; break;
  }
  uint16_t major = FW_VERSION / 10000;
  printf("\r\nReset Source: %s\r\nMCU: " MCU_DESC "\r\nFW: %u.%02u " \
         __DATE__ "\r\n", rst_source[idx], major, FW_VERSION - (10000* major));

  /* note: KEYV indicates an incorrect FCTLx password was written to any flash
   * control register and generates a PUC when set. */
}
/*---------------------------------------------------------------------------*/
void
bsl_entry(void)
{
  PIN_CFG_OUT(LED_STATUS);
  PIN_SET(LED_STATUS);
  ((void (*)())0x1000)();
  __nop();
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
  
  rst_flag = SYSRSTIV;    /* read reset flag */
  if(rst_flag == SYSRSTIV_DOBOR) {
    bsl_entry();          /* enter bootstrap loader if software BOR detected */
  }

  /* initialize hardware */

  /* set default configuration for all GPIOs (output low) */
  PORT_CLR_I(1);
  P1DIR = 0xff & ~(BIT1 | BIT2 | BIT3 | BIT4 | BIT5 | BIT6);
  P1SEL = (BIT2 | BIT3 | BIT4 | BIT5 | BIT6);
  PORT_CLR_I(2);
  P2DIR = 0xff & ~(BIT1 | BIT2 | BIT5);
  PORT_CLR_I(3);
  PORT_CFG_OUT_I(3);
  PORT_CLR_I(J);
  PORT_CFG_OUT_I(J);
  
  /* enable status LED to indicate start of init routine */
  PIN_SET(LED_STATUS);

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
#if UART_CONF_RX_INTERRUPT
  uart_set_input_handler(serial_line_input_byte);
#endif /* UART_CONF_RX_INTERRUPT */

  print_device_info();

#if RF_CONF_ON
  /* init the radio module and set the parameters */
  rf1a_init();
#endif /* RF_CONF_ON */

  /* set the node ID */
  node_id = TOS_NODE_ID;
  printf(CONTIKI_VERSION_STRING " started. Node ID is set to %u.\r\n",
         node_id);

#if FRAM_CONF_ON
  if (!fram_init()) {
    DEBUG_PRINT_FATAL("ERROR: FRAM failure");
  }
#endif /* FRAM_CONF_ON */
  
#if BOLT_CONF_ON
  if (!bolt_init()) {
    DEBUG_PRINT_FATAL("ERROR: Bolt init failed");
  }
#endif /* BOLT_CONF_ON */

#if SVS_CONF_ON
  SVS_ENABLE;
#else
  SVS_DISABLE;
#endif /* SVS_CONF_ON */

  process_init();

  random_init(node_id * TA0R);
#if UART_CONF_RX_INTERRUPT
  serial_line_init();
#endif /* UART_CONF_RX_INTERRUPT */
  /* note: do not start the debug process here */

#if ENERGEST_CONF_ON
  energest_init();
  ENERGEST_ON(ENERGEST_TYPE_CPU);
  DCSTAT_CPU_ON;
#endif /* ENERGEST_CONF_ON */

#if NULLMAC_CONF_ON
  nullmac_init();
#endif /* NULLMAC_CONF_ON */
  
  /* start processes */
  autostart_start(autostart_processes);
  debug_print_init();
  /* note: start debug process as last due to process_poll() execution order */
  
  /* disable status LED to indicate successful termination of init routine */
  PIN_CLR(LED_STATUS);

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
#if WATCHDOG_CONF_ON && WATCHDOG_CONF_STOP_IN_LPM
      /* no need to stop the watchdog in the low-power mode if it is reset
       * within the timer update interrupt (which occurs every 2 seconds) */
      watchdog_stop();
#endif /* WATCHDOG_CONF_ON */
      /* LPM3 */
      __bis_status_register(GIE | SCG0 | SCG1 | CPUOFF);
      __no_operation();
#if WATCHDOG_CONF_ON && WATCHDOG_CONF_STOP_IN_LPM
      watchdog_start();
#endif /* WATCHDOG_CONF_ON */
      DCSTAT_CPU_ON;
      ENERGEST_ON(ENERGEST_TYPE_CPU);
    }
  }

  return 0;
}
/*---------------------------------------------------------------------------*/
