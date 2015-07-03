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
#define DMA_TRIGGERSRC_A0RX 0x10
#define DMA_TRIGGERSRC_A0TX 0x11
#define DMA_TRIGGERSRC_B0RX 0x12
#define DMA_TRIGGERSRC_B0TX 0x13
/*---------------------------------------------------------------------------*/
static dma_callback_t dma_tc_callback = 0;
uint8_t dummy_byte = 0x00;
/*---------------------------------------------------------------------------*/
void
dma_init_spi(uint16_t spi_addr,
             uint16_t rx_buffer_addr,
             uint16_t tx_buffer_addr,
             dma_callback_t callback_func)
{
  /* use channel 0 for RX (reception) and channel 1 for TX (transmission) */

  /* channel 0 (RX) */

  /* single transfer, byte-to-byte, rising edge triggers */
  DMA0CTL = DMADT_0 + DMASRCBYTE + DMADSTBYTE;
  /* set transfer size to a default value */
  DMA0SZ  = 0;
  /* set DMA control 0 register */
  DMACTL0 &= 0xFF00;                    /* reset trigger select */
  if (spi_addr == USCI_A0) {
    DMACTL0 |= DMA_TRIGGERSRC_A0RX;
  } else {
    DMACTL0 |= DMA_TRIGGERSRC_B0RX;
  }  
  /* set source address */
  if (spi_addr == USCI_A0) {
    DMA0SA = UCA0RXBUF; //SPI_A0_RXBUF;      
  } else {
    DMA0SA = UCB0RXBUF; //SPI_B0_RXBUF;
  }
  /* reset bits before setting them */
  DMA0CTL &= ~DMASRCINCR_3;
  DMA0CTL |= DMASRCINCR_0;   /* do not increment source address */
  /* set destination address */
  DMA0DA = (uint16_t)rx_buffer_addr;
  /* reset bits before setting them */
  /* REGVAL16(DMA_CH0CTL) &= ~DMADSTINCR_3; */
  DMA0CTL |= DMADSTINCR_3;   /* increment destination address */

  /* channel 1 (TX) */

  /* single transfer, byte-to-byte, rising edge triggers */
  DMA1CTL = DMADT_0 + DMASRCBYTE + DMADSTBYTE;
  DMA1SZ  = 0;
  /* set DMA control 0 register */
  DMACTL0 &= 0x00FF;     /* reset trigger select */
  if (spi_addr == USCI_A0) {
    DMACTL0 |= (DMA_TRIGGERSRC_A0TX << 8);
  } else {
    DMACTL0 |= (DMA_TRIGGERSRC_B0TX << 8);
  }
  /* set source address */
  DMA1SA = (uint16_t)(tx_buffer_addr + 1);

  DMA1CTL |= DMASRCINCR_3;  /* increment address */
  /* set destination address */
  if (spi_addr == USCI_A0) {
    DMA1DA = UCA0TXBUF;
  } else {
    DMA1DA = UCB0TXBUF;
  }
  /* reset bits before setting them */
  DMA1CTL &= ~DMADSTINCR_3;
  DMA1CTL |= DMADSTINCR_0;  /* do not increment address */

  /* enable DMA interrupts */
  DMA0CTL &= ~DMAIFG;       /* clear the interrupt flag */
  DMA1CTL &= ~DMAIFG;

  dma_tc_callback = callback_func;
}
/*---------------------------------------------------------------------------*/
void
dma_init_timer(uint16_t src_addr, uint16_t dest_addr)
{
  /* use channel 2 to take a snapshot of the sw extension of the timer TA1 */

  /* disable DMA and reset bits */
  DMA2CTL &=
    ~(DMASRCBYTE + DMADSTBYTE + DMALEVEL + DMASRCINCR_3 + DMADSTINCR_3 +
      DMAEN +
      DMAIE + DMAIFG);
  DMA2CTL = DMADT_1;                    /* block transfer, word-to-word, rising
                                           edge triggers */
  DMA2SZ = 4;                           /* set transfer size */
  DMACTL1 &= ~0x00FF;                   /* reset trigger select */
  DMACTL1 |= 0x0003;                    /* set trigger in DMA control 1
                                           register */
  DMA2SA = src_addr;                    /* set source address */
  DMA2DA = dest_addr;                   /* set destination address */
  DMA2CTL |= (DMASRCINCR_3 + DMADSTINCR_3);     /* increment source and
                                                   destination address */
  DMA2CTL |= DMAEN;                     /* enable DMA */
}
/*---------------------------------------------------------------------------*/
ISR(DMA, dma_interrupt)
{
  ENERGEST_ON(ENERGEST_TYPE_CPU);

  if(DMA0CTL & DMAIFG) {
    DMA0CTL &= ~DMAIFG;
  } else if(DMA1CTL & DMAIFG) {
    DMA1CTL &= ~DMAIFG;
  }

  DEBUG_PRINT_VERBOSE("DMA transfer complete");
  if(dma_tc_callback) {
    dma_tc_callback();
  }

  ENERGEST_OFF(ENERGEST_TYPE_CPU);
}
/*---------------------------------------------------------------------------*/
