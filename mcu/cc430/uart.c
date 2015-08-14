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

#include "contiki.h"
#include "platform.h"

/*---------------------------------------------------------------------------*/
static int (*uart0_input_handler)(unsigned char c);
/* static volatile uint8_t transmitting; */
static uint32_t prescaler = 0;
static uint32_t mod = 0;
/*---------------------------------------------------------------------------*/
/* pin definitions */
#ifndef UART_RXD
#define UART_RXD        PORT1, PIN5     /* input (receive line) */
#endif /* UART_RXD */
#ifndef UART_TXD
#define UART_TXD        PORT1, PIN6     /* output (transmit line) */
#endif /* UART_TXD */
#define UART_ACTIVE     (UCA0STAT & UCBUSY)
#define UART_ENABLE     (UCA0CTL1 &= ~UCSWRST)
#define UART_DISABLE    (UCA0CTL1 |= UCSWRST)
/*---------------------------------------------------------------------------*/
/* this function must be defined (referenced from std lib printf.c) */
int
putchar(int c)
{
  /* print out a single byte over UART */
#if WATCHDOG_CONF_ON
  watchdog_periodic();
#endif /* WATCHDOG_CONF_ON */
  /* Loop until the transmission buffer is available. */
  while((UCA0STAT & UCBUSY));

  /* Transmit the data. */
  UCA0TXBUF = c;
  return c;
}
/*---------------------------------------------------------------------------*/
void
uart_set_input_handler(int (*input)(unsigned char c))
{
  uart0_input_handler = input;
}
/*---------------------------------------------------------------------------*/
void
uart_init(void)
{
  /*PIN_MAP_AS_INPUT(UART_A0_RX, PM_UCA0RXD);*/   /* map UART RX input to port
                                                   1.5 (same as SPI A0 SOMI) */
  /*PIN_MAP_AS_OUTPUT(UART_A0_TX, PM_UCA0TXD);*/ /* map UART TX output to port
                                                   1.6 (same as SPI A0 SIMO) */
  PIN_CFG_OUT(UART_TXD);
  PIN_CFG_IN(UART_RXD);
  PIN_SEL(UART_TXD);
  PIN_SEL(UART_RXD);

  UCA0CTL1 |= UCSWRST;                     /* Hold peripheral in reset state */
  UCA0CTL1 &= ~UCSSEL_3;
  UCA0CTL1 |= UCSSEL__SMCLK;                   /* clock source select: SMCLK */
  /* LSB first is 0, one stop bit is 0, no parity is 0 > just clear the bits */
  UCA0CTL0 &= ~(UCMSB + UCSPB + UCPEN + UCSYNC + UC7BIT + UCMODE_3);     

  uint32_t f = XT2CLK_SPEED / SMCLK_SPEED;
  uint32_t ratio = (SMCLK_SPEED * 100) / UART_CONF_BAUDRATE;
  prescaler = ratio / 100;
  /* note: factor of 8 results from 26 MHz / SMCLK_SPEED */
  mod = ((ratio - prescaler * 100) * f + 50) / 100;  

  UCA0BRW = (uint16_t)prescaler;    /* write the 16-bit clock prescaler */
  UCA0MCTL = (uint8_t)mod << 1;     /* modulation control register */

  /* select UART mode */
  UCA0CTL0 |= UCMODE_0;     /* default mode */

  UCA0IFG &= ~(UCRXIFG + UCTXIFG);        /* clear pending interrupts */
  /* initialize USCI state machine **before** enabling interrupts */
  /*UCA0CTL1 &= ~UCSWRST;*/
  /* note: DO NOT enable interrupts, they can mess up the timing due to higher
     priority */
  /* UCA0IE |= UCRXIE; */
}
/*---------------------------------------------------------------------------*/
void
uart_reinit(void)
{
  if (USCI_A0_IN_SPI_MODE)      /* only reconfigure if module is in SPI mode */
  {
    while(UART_ACTIVE);      /* wait until all transmissions have terminated */
    UCA0CTL1 |= UCSWRST;
    UCA0CTL0 &= ~(UCMSB + UCSPB + UCPEN + UCSYNC + UC7BIT + UCMODE_3);
    UCA0BRW   = (uint16_t)prescaler;
    UCA0CTL0 |= UCMODE_0;
    /*UCA0CTL1 &= ~UCSWRST;*/
  }
}
/*---------------------------------------------------------------------------*/
void
uart_enable(uint8_t enable)
{
  if(enable) {
    UART_BEFORE_ENABLE; 
    UART_ENABLE;
  } else {
    while(UART_ACTIVE);
    UART_DISABLE;
    UART_AFTER_DISABLE;
  }    
}
/*---------------------------------------------------------------------------*/
ISR(USCI_A0, uart0_rx_interrupt) 
{
  ENERGEST_ON(ENERGEST_TYPE_CPU);

  uint8_t c;

  if(UCA0IV == 2) {
    if(UCA0STAT & UCRXERR) {
      c = UCA0RXBUF;   /* Clear error flags by forcing a dummy read. */
    } else {
      c = UCA0RXBUF;
      if(uart0_input_handler != NULL) {
        if(uart0_input_handler(c)) {
          LPM4_EXIT;
        }
      }
    }
  }

  ENERGEST_OFF(ENERGEST_TYPE_CPU);
}
/*---------------------------------------------------------------------------*/
