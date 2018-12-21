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
 *          Federico Ferrari
 */

#include "contiki.h"
#include "platform.h"

/*---------------------------------------------------------------------------*/
void
clock_init(void)
{
  /* set clock sources: */
  /* FLLREFCLK <- XT1 (low-frequency crystal, 32,768 Hz) */
  /* SMCLK     <- XT2 (high-frequency crystal, 26 MHz / 8 = 3.25 MHz) */
  /* ACLK      <- XT1 (low-frequency crystal, 32768 Hz) */
  /* MCLK      <- XT2 (high-frequency crystal, 26 MHz / 2 = 13 MHz) */

  /* set the supply voltage to the 3rd level (there are 4 levels) */
  SetVCore(PMMCOREV_2);

  /* disable oscillator fault interrupt */
  SFRIE1 &= ~OFIE;

  /* enable XT2 and XT1 (ensures that they stay always on) */
  ENABLE_XT2();
#if CLOCK_CONF_XT1_ON
  ENABLE_XT1();
  /* set pins 5.0 and 5.1 to analog (XT1 operation) */
  P5SEL |= BIT0 + BIT1;
  /* set internal load capacitor for XT1
   * note: the total capacitance seen by the crystal is (XCAP + 2pF) / 2 
   * where XCAP can be 2 (XCAP_0), 6, 9 or 12pF (XCAP_3)
   * default value is 3 */
  UCSCTL6 &= ~XCAP_3;
  UCSCTL6 |= CLOCK_CONF_XT1_CAP;
#endif /* CLOCK_CONF_XT1_ON */
  
  /* initially, use the internal REFO and DCODIV clock sources */
  UCSCTL3 = SELREF__REFOCLK | FLLREFDIV_0;
  UCSCTL4 = SELA__REFOCLK | SELS__DCOCLKDIV | SELM__DCOCLKDIV;
  /* wait until XT1, XT2, and DCO stabilize */
  WAIT_FOR_OSC();
  
#if CLOCK_CONF_XT1_ON
  /* XT1 is now stable: reduce its drive strength to save power
   * for eff. load cap. btw. 6 and 9pF, use DRIVE_1 (see datasheet p.49) */
  UCSCTL6 &= ~XT1DRIVE_3;
  UCSCTL6 |= XT1DRIVE_1;
#endif /* CLOCK_CONF_XT1_ON */

  /* disable the FLL control loop */
  DISABLE_FLL();

#if CLOCK_CONF_FLL_ON
 #if CLOCK_CONF_XT1_ON
  /* use XT1 as FLL reference, divider 1 */
  UCSCTL3 = SELREF__XT1CLK | FLLREFDIV_0;
 #endif /* CLOCK_CONF_XT1_ON */
  /* set the lowest possible DCOx, MODx */
  UCSCTL0 = 0;
  /* select the suitable DCO frequency range */
  UCSCTL1 = DCORSEL_6;
  /* set the FLL loop divider to 2 and
   * the multiplier N such that (N + 1) * f_FLLREF = f_DCO --> N = 396 */
  UCSCTL2 = FLLD_0 + 396;   /* 396 * 32768 = ~13MHz */
  /* enable the FLL control loop */
  ENABLE_FLL();
  /* wait until the DCO stabilizes */
  /* (up to 1 x 32 x 32 x 13 MHz / 32,768 Hz = 406,250 DCO cycles) */
  __delay_cycles(406250);
#endif /* CLOCK_CONF_FLL_ON */

  /* finally, use the desired clock sources and speeds */
  UCSCTL5 = DIVA | DIVS | DIVM;
  UCSCTL4 = SELA | SELS | SELM;
  /* note: will automatically use DCOCLKDIV for SMCLK and MCLK if XT2CLK is 
   * not available */
  
  /* oscillator fault flag may be set after switching the clock source */
  WAIT_FOR_OSC();
  
  /* enable oscillator fault interrupt (NMI) as well as vacant memory access
   * interrupt (VMAIE) and flash controller access violation int. (ACCVIE) */
  SFRIE1 |= OFIE | VMAIE | ACCVIE;
}
/*---------------------------------------------------------------------------*/
ISR(UNMI, unmi_interrupt)       /* user non-maskable interrupts */
{    
  DEBUG_ISR_ENTRY;
  DCSTAT_CPU_ON;
  ENERGEST_ON(ENERGEST_TYPE_CPU);

  PIN_SET(LED_ERROR);           /* use PIN_SET instead of LED_ON */
  switch (SYSUNIV) {
    case SYSUNIV_NMIIFG:        /* non-maskable interrupt */
      while(1);
      break;
    case SYSUNIV_OFIFG:         /* oscillator fault */
      WAIT_FOR_OSC();           /* try to clear the fault flag */
      break;
    case SYSUNIV_ACCVIFG:       /* Flash Memory Access Violation */
      /* will be accidentially triggered after flash programming when using
       * a voltage between 2.4 and 2.6V (errata SYS12) */
      while(1);
      break;
    case SYSUNIV_SYSBERRIV:
      while(1);
      break;
    default:
      while(1);
      break;
  }
  PIN_CLR(LED_ERROR);
  
  ENERGEST_OFF(ENERGEST_TYPE_CPU);
  DCSTAT_CPU_OFF;
  DEBUG_ISR_EXIT;
}
/*---------------------------------------------------------------------------*/
