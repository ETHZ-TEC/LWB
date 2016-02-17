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

/* TODO:
 * - test this DMA driver
 * - verify that spi_write_byte() won't block! waiting for the TXE flag while SPI 
 *   is disabled might not be the best idea
 */

#include "contiki.h"
#include "platform.h"

/*---------------------------------------------------------------------------*/
#define DMA_DISABLE_RX                  (DMA0CTL &= ~(DMAEN))
#define DMA_ENABLE_RX                   (DMA0CTL |= DMAEN)
#define DMA_DISABLE_TX                  (DMA1CTL &= ~(DMAEN))
#define DMA_ENABLE_TX                   (DMA1CTL |= DMAEN)
#define DMA_ENABLE_RX_INTERRUPT         (DMA0CTL |= DMAIE)
#define DMA_ENABLE_TX_INTERRUPT         (DMA1CTL |= DMAIE)
#define DMA_DISABLE_RX_INTERRUPT        (DMA0CTL &= ~DMAIE)
#define DMA_DISABLE_TX_INTERRUPT        (DMA1CTL &= ~DMAIE)
#define DMA_RX_IFG                      (DMA0CTL & DMAIFG)
#define DMA_TX_IFG                      (DMA1CTL & DMAIFG)
#define DMA_ENABLE_TX_SRCADDRINC        (DMA1CTL |= DMASRCINCR_3)
#define DMA_DISABLE_TX_SRCADDRINC       (DMA1CTL = (DMA1CTL & ~(DMASRCINCR_3)))
#define DMA_SET_RX_TRANSFERSIZE(size)   (DMA0SZ = (uint16_t)(size))
#define DMA_SET_TX_TRANSFERSIZE(size)   (DMA1SZ = (uint16_t)(size))
#define DMA_SET_RX_BUFADDR(addr)        (DMA0DA = addr) /* dest. for CH0 */
#define DMA_SET_TX_BUFADDR(addr)        (DMA1SA = addr) /* src for CH1 */
/*---------------------------------------------------------------------------*/
#define DMA_TRIGGERSRC_TA0CCR0  0x01
#define DMA_TRIGGERSRC_TA0CCR2  0x02
#define DMA_TRIGGERSRC_TA1CCR0  0x03
#define DMA_TRIGGERSRC_TA1CCR2  0x04
#define DMA_TRIGGERSRC_A0RX     0x10 
#define DMA_TRIGGERSRC_A0TX     0x11
#define DMA_TRIGGERSRC_B0RX     0x12
#define DMA_TRIGGERSRC_B0TX     0x13
/*---------------------------------------------------------------------------*/
static dma_callback_t dma_tc_callback = 0;
static uint16_t dma_spi_addr = 0;
static uint8_t dma_dummy_byte = 0x00;
/*---------------------------------------------------------------------------*/
void
dma_config_spi(uint16_t spi_addr,
               dma_callback_t callback_func)
{
  /* use channel 0 for RX (reception) and channel 1 for TX (transmission) */

  /* single transfer, byte-to-byte, rising edge triggers, 
   * do not increment source address, increment destination address */
  DMA0CTL = DMADT_0 + DMASRCBYTE + DMADSTBYTE + DMASRCINCR_0 + DMADSTINCR_3;
  DMA0SZ  = 0;
  /* single transfer, byte-to-byte, rising edge triggers,
   * increment src address, do not increment dst address */
  DMA1CTL = DMADT_0 + DMASRCBYTE + DMADSTBYTE + DMASRCINCR_3 + DMADSTINCR_0;
  DMA1SZ  = 0;
  /* set source address */
  if (spi_addr == USCI_A0) {
    DMA0SA = UCA0RXBUF_;        /* address of the RX buffer */ 
    DMA1DA = UCA0TXBUF_;        /* address of the TX buffer */ 
    DMACTL0 = (DMA_TRIGGERSRC_A0TX << 8) | DMA_TRIGGERSRC_A0RX;
  } else {
    DMA0SA = UCB0RXBUF_;
    DMA1DA = UCB0TXBUF_;
    DMACTL0 = (DMA_TRIGGERSRC_B0TX << 8) | DMA_TRIGGERSRC_B0RX;
  }
  /* set destination address */
  /*DMA0DA = (uint16_t)rx_buffer_addr;*/
  /*DMA1SA = (uint16_t)(tx_buffer_addr + 1);*/
  /*DMA0CTL &= ~DMAIFG;        clear the interrupt flag */
  /*DMA1CTL &= ~DMAIFG; */

  dma_tc_callback = callback_func;
  dma_spi_addr = spi_addr;
}
/*---------------------------------------------------------------------------*/
void
dma_config_timer(dma_triggersrc_t trigger_src, 
                 uint16_t src_addr, 
                 uint16_t dest_addr, 
                 uint8_t num_bytes)
{
  /* use DMA CH2 to e.g. take a snapshot of the sw extension of the timer */

  /* disable DMA and reset bits */
  DMA2CTL &=
    ~(DMASRCBYTE + DMADSTBYTE + DMALEVEL + DMASRCINCR_3 + DMADSTINCR_3 +
      DMAEN +
      DMAIE + DMAIFG);
  DMA2CTL = DMADT_1;                    /* block transfer, word-to-word, rising
                                           edge triggers */
  DMA2SZ = num_bytes >> 1;              /* set transfer size */
  DMACTL1 &= ~0x00ff;                   /* reset trigger select */
  DMACTL1 |= trigger_src;               /* set trigger in DMA control 1
                                           register */
  DMA2SA = src_addr;                    /* set source address */
  DMA2DA = dest_addr;                   /* set destination address */
  DMA2CTL |= (DMASRCINCR_3 + DMADSTINCR_3);     /* increment source and
                                                   destination address */
  DMA2CTL |= DMAEN;                     /* enable DMA */
}
/*---------------------------------------------------------------------------*/
void 
dma_enable_timer(uint8_t enable)
{
  if(enable) {
    DMA2CTL &= ~DMAIFG;
    DMA2CTL |= DMAEN;
  } else {
    DMA2CTL &= ~DMAEN;
  }
}
/*---------------------------------------------------------------------------*/
uint8_t
dma_start(uint16_t rx_buf_addr, uint16_t tx_buf_addr, uint16_t num_bytes)
{
  if(!dma_spi_addr) {
    DEBUG_PRINT_ERROR("DMA not configured");
    return 0;
  }
  if(rx_buf_addr) {
    /* the received data is of interest */
    DMA_SET_RX_TRANSFERSIZE(num_bytes);
    DMA_SET_TX_TRANSFERSIZE(num_bytes - 1); 
    DMA_SET_RX_BUFADDR((uint16_t)rx_buf_addr);   /* destination buffer */        
    DMA_DISABLE_TX_INTERRUPT; 
    DMA_ENABLE_RX_INTERRUPT;
    DMA_ENABLE_RX;
    DMA_ENABLE_TX;
    if(tx_buf_addr) {
      /* the transmitted data matters */  
      DMA_SET_TX_BUFADDR(tx_buf_addr); 
      DMA_ENABLE_TX_SRCADDRINC;
      /* write the frist byte to trigger the DMA (TXE) */
      spi_write_byte(dma_spi_addr, *((uint8_t*)tx_buf_addr)); 
    } else {
      /* transmit dummy data (all zero's) */  
      DMA_SET_TX_BUFADDR((uint16_t)&dma_dummy_byte); 
      DMA_DISABLE_TX_SRCADDRINC;
      /* write the frist byte to trigger the DMA (TXE) */
      spi_write_byte(dma_spi_addr, dma_dummy_byte); 
    }  
  } else {
    if(!tx_buf_addr) {
      DEBUG_PRINT_ERROR("DMA: invalid rx/tx buffer address");
      return 0;         /* error */
    }
    /* only the transmitted data matters */
    DMA_SET_TX_BUFADDR(tx_buf_addr + 1);   /* source buffer */
    DMA_SET_TX_TRANSFERSIZE(num_bytes - 1);
    DMA_ENABLE_TX_SRCADDRINC;
    DMA_DISABLE_RX_INTERRUPT;
    DMA_ENABLE_TX_INTERRUPT;
    DMA_DISABLE_RX;
    DMA_ENABLE_TX;      /* TX only */
    /* write the frist byte to trigger the DMA (TXE) */
    spi_write_byte(dma_spi_addr, *(uint8_t*)tx_buf_addr);
  }
  return 1;
}
/*---------------------------------------------------------------------------*/
void
dma_set_dummy_byte_value(uint8_t c)
{
  dma_dummy_byte = c;   
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
  
  /* disable the DMA requests */
  DMA_DISABLE_TX;
  DMA_DISABLE_RX;
  
  DEBUG_PRINT_VERBOSE("DMA transfer complete");
  if(dma_tc_callback) {
    dma_tc_callback();
  }

  ENERGEST_OFF(ENERGEST_TYPE_CPU);
}
/*---------------------------------------------------------------------------*/
