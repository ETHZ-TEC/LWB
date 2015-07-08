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

#include "platform.h"

/*---------------------------------------------------------------------------*/
uint32_t spi_sck_div_a0 = 0;
/*---------------------------------------------------------------------------*/
void
spi_a0_init(uint32_t bit_clk_speed)
{
  PIN_SET_AS_MODULE_FUNC(SPI_A0_SOMI);
  PIN_SET_AS_MODULE_FUNC(SPI_A0_SIMO);
  PIN_SET_AS_MODULE_FUNC(SPI_A0_CLK);
  PIN_SET_AS_INPUT(SPI_A0_SOMI);
  PIN_SET_AS_OUTPUT(SPI_A0_SIMO);
  PIN_SET_AS_OUTPUT(SPI_A0_CLK);
  /*P1DS = 0; port 1 drive strength */

  while(SPI_A0_ACTIVE);            /* busy wait */

  spi_sck_div_a0 = SMCLK_SPEED / bit_clk_speed;

  /* see comments in code above for explanation */
  UCA0CTL1 |= UCSWRST;  /* note: UCA0CTL1 for byte, UCA0CTL for word access */
  UCA0CTL0 &= ~(UCCKPH + UCCKPL + UC7BIT + UCMSB + UCMST + UCMODE_3 + UCSYNC);
  UCA0CTL1 &= ~(UCSSEL_3);
  UCA0CTL1 |= UCSSEL__SMCLK;
  UCA0BRW = spi_sck_div_a0;
  UCA0CTL0 |=
    (UCMSB + UCMST + UCSYNC + (SPI_A0_CPOL ? UCCKPL : 0) +
     (SPI_A0_CPHA ? 0 : UCCKPH));
  UCA0IE &= ~(UCRXIFG + UCTXIFG);
}
/*---------------------------------------------------------------------------*/
void
spi_b0_init(uint32_t bit_clk_speed)
{
  /* GPIO configuration */
  PIN_SET_AS_MODULE_FUNC(SPI_B0_SOMI);
  PIN_SET_AS_MODULE_FUNC(SPI_B0_SIMO);
  PIN_SET_AS_MODULE_FUNC(SPI_B0_CLK);
  PIN_SET_AS_INPUT(SPI_B0_SOMI);
  /* PIN_RESISTOR_EN(SPI_B0_SOMI); */
  PIN_SET_AS_OUTPUT(SPI_B0_SIMO);
  PIN_SET_AS_OUTPUT(SPI_B0_CLK);

  while(SPI_B0_ACTIVE);            /* busy wait */

  uint32_t sck_div = SMCLK_SPEED / bit_clk_speed;

  UCB0CTL1 |= UCSWRST;              /* reset USCI (disable the module) */
  /* reset OFS_UCBxCTL0 bits */
  UCB0CTL0 &= ~(UCCKPH + UCCKPL + UC7BIT + UCMSB + UCMST + UCMODE_3 + UCSYNC);
  /* reset OFS_UCBxCTL1 bits */
  UCB0CTL1 &= ~(UCSSEL_3);
  /* select clock source */
  UCB0CTL1 |= UCSSEL__SMCLK;        /* use SMCLK */
  /* bit rate control register (clock prescaler) */
  UCB0BRW = sck_div;
  /* UCMST = Master mode, UCSYNC = Synchronous mode, UCMSB = MSB first,
     UCMODE_0 = 0 = 3-pin SPI */
  UCB0CTL0 |=
    (UCMSB + UCMST + UCSYNC + (SPI_B0_CPOL ? UCCKPL : 0) +
     (SPI_B0_CPHA ? 0 : UCCKPH));
  /* clear pending interrupts */
  UCB0IE &= ~(UCRXIE + UCTXIE);
  /* UCB0CTL1 &= ~UCSWRST;         // re-enable the module */
}
/*---------------------------------------------------------------------------*/
void
spi_a0_reinit(void)
{
  if(!USCI_A0_IN_SPI_MODE) {
    while(SPI_A0_ACTIVE);
    UCA0CTL1 |= UCSWRST;
    UCA0CTL0 &= ~(UCCKPH + UCCKPL + UC7BIT + UCMODE_3);
    UCA0BRW = spi_sck_div_a0;
    UCA0CTL0 |=
        (UCMSB + UCMST + UCSYNC + (SPI_A0_CPOL ? UCCKPL : 0) +
        (SPI_A0_CPHA ? 0 : UCCKPH));
  }
}
/*---------------------------------------------------------------------------*/
