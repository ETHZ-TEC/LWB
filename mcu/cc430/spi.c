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

/**
 * @brief SPI driver implementation
 * @note 
 * SPI_0 = USCI A0
 * SPI_1 = USCI_B0
 * 
 * Configures the USCI peripheral module in SPI mode as follows:
 * Master mode, MSB first, 8-bit, 3 lines.
 * Specify the SPI phase and polarity with the SPI_CPOL/SPI_CPHA defines.
 */

/*---------------------------------------------------------------------------*/
/* pin definitions (not configurable, thus without _CONF */
#define SPI_B0_SOMI             PORT1, PIN2
#define SPI_B0_SIMO             PORT1, PIN3
#define SPI_B0_CLK              PORT1, PIN4
#define SPI_A0_SOMI             PORT1, PIN5
#define SPI_A0_SIMO             PORT1, PIN6
#define SPI_A0_CLK              PORT1, PIN7
/*---------------------------------------------------------------------------*/
/* checks if a transmission is ongoing on SPI A0 */
#define SPI_A0_ACTIVE           (UCA0STAT & UCBUSY)
/* checks if a transmission is ongoing on SPI B0 */
#define SPI_B0_ACTIVE           (UCB0STAT & UCBUSY)
/* checks the RX buffer overrun error flag (set when the RX buffer is
 * overwritten without reading the previous value */
#define SPI_A0_RX_OVERRUN       (UCA0STAT & UCOE)
/* checks the RX buffer overrun error flag (set when the RX buffer is
 * overwritten without reading the previous value) */
#define SPI_B0_RX_OVERRUN       (UCB0STAT & UCOE)
/*---------------------------------------------------------------------------*/
typedef enum {
  SPI_REG_RXBUF = 0,
  SPI_REG_TXBUF,
  SPI_REG_IFG,
  SPI_REG_STAT,
  NUM_SPI_REG
} spi_reg_t;
/* for easier access to the registers */
volatile uint8_t* const spi_reg[NUM_OF_SPI_MODULES][NUM_SPI_REG] =
{
  { (volatile uint8_t*)&UCA0RXBUF, &UCA0TXBUF, &UCA0IFG, &UCA0STAT },/*SPI_0 */
  { (volatile uint8_t*)&UCB0RXBUF, &UCB0TXBUF, &UCB0IFG, &UCB0STAT } /*SPI_1 */
};
/*---------------------------------------------------------------------------*/
uint32_t clk_div_a0 = 0, 
         clk_div_b0 = 0;
/*---------------------------------------------------------------------------*/
uint8_t
spi_init(spi_module_t spi, uint32_t bclk_speed)
{
  if(SPI_0 == spi) {
    /* configure the GPIO pins */
    PIN_SEL(SPI_A0_SOMI);
    PIN_SEL(SPI_A0_SIMO);
    PIN_SEL(SPI_A0_CLK);
    //PIN_CFG_IN(SPI_A0_SOMI);
    /* PIN_RESISTOR_EN(SPI_A0_SOMI); */
    //PIN_CFG_OUT(SPI_A0_SIMO);
    //PIN_CFG_OUT(SPI_A0_CLK);

    while(SPI_A0_ACTIVE); /* busy wait, just in case the SPI is still active */

    clk_div_a0 = SMCLK_SPEED / bclk_speed;

    /* note: use UCA0CTL1 for byte and UCA0CTL for word access */
    UCA0CTL1 |= UCSWRST;  
    UCA0CTL0 &= ~(UCCKPH + UCCKPL + UC7BIT + UCMSB + UCMST + UCMODE_3 +UCSYNC);
    UCA0CTL1 &= ~(UCSSEL_3);
    UCA0CTL1 |= UCSSEL__SMCLK;        /* use SMCLK as clock source */
    UCA0BRW = clk_div_a0;
    UCA0CTL0 |= (UCMSB + UCMST + UCSYNC + (SPI_CPOL ? UCCKPL : 0) +
                (SPI_CPHA ? 0 : UCCKPH));
    UCA0IFG &= ~(UCRXIFG + UCTXIFG);       /* clear interrupt flags */
    
    return 1;
    
  } else if(SPI_1 == spi) {
    /* GPIO configuration */
    PIN_SEL(SPI_B0_SOMI);
    PIN_SEL(SPI_B0_SIMO);
    PIN_SEL(SPI_B0_CLK);
    //PIN_CFG_IN(SPI_B0_SOMI);
    /* PIN_RESISTOR_EN(SPI_B0_SOMI); */
    //PIN_CFG_OUT(SPI_B0_SIMO);
    //PIN_CFG_OUT(SPI_B0_CLK);

    while(SPI_B0_ACTIVE); /* busy wait, just in case the SPI is still active */

    clk_div_b0 = SMCLK_SPEED / bclk_speed;

    UCB0CTL1 |= UCSWRST;              /* reset USCI (disable the module) */
    UCB0CTL0 &= ~(UCCKPH + UCCKPL + UC7BIT + UCMSB + UCMST + UCMODE_3 +UCSYNC);
    UCB0CTL1 &= ~(UCSSEL_3);
    UCB0CTL1 |= UCSSEL__SMCLK;        /* use SMCLK as clock source */
    /* bit rate control register (clock prescaler) */
    UCB0BRW = clk_div_b0;
    /* UCMST = Master mode, UCSYNC = Synchronous mode, UCMSB = MSB first,
        UCMODE_0 = 0 = 3-pin SPI */
    UCB0CTL0 |= (UCMSB + UCMST + UCSYNC + (SPI_CPOL ? UCCKPL : 0) +
                (SPI_CPHA ? 0 : UCCKPH));
    UCB0IFG &= ~(UCRXIFG + UCTXIFG);        /* clear interrupt flags */
    /* UCB0CTL1 &= ~UCSWRST; -> do not re-enable the module */
    
    return 1;      
  }
  return 0;
}
/*---------------------------------------------------------------------------*/
void
spi_reinit(spi_module_t spi)
{
  if(SPI_0 == spi) {
    if(!USCI_A0_IN_SPI_MODE) {
      while(SPI_A0_ACTIVE);
      UCA0CTL1 |= UCSWRST;
      UCA0CTL0 &= ~(UCCKPH + UCCKPL + UC7BIT + UCMODE_3);
      UCA0BRW = clk_div_a0;
      UCA0CTL0 |= (UCMSB + UCMST + UCSYNC + (SPI_CPOL ? UCCKPL : 0) +
                  (SPI_CPHA ? 0 : UCCKPH));
    }
  } else {
    if(!USCI_B0_IN_SPI_MODE) {
      while(SPI_B0_ACTIVE);
      UCB0CTL1 |= UCSWRST;
      UCB0CTL0 &= ~(UCCKPH + UCCKPL + UC7BIT + UCMODE_3);
      UCB0BRW = clk_div_b0;
      UCB0CTL0 |= (UCMSB + UCMST + UCSYNC + (SPI_CPOL ? UCCKPL : 0) +
                  (SPI_CPHA ? 0 : UCCKPH));
    }
  }
}
/*---------------------------------------------------------------------------*/
void
spi_enable(spi_module_t spi, uint8_t enable)
{
  if(enable) {
    /* let the application do whatever it needs to do before enabling SPI */
    SPI_BEFORE_ENABLE(spi);
    if(SPI_0 == spi) {
      UCA0CTL1 &= ~UCSWRST;
      /* wait for the clock signal to reach its 'inactive' level */
      while((0 == SPI_CPOL) ? PIN_GET(SPI_A0_CLK) : (!PIN_GET(SPI_A0_CLK)));
    } else {
      UCB0CTL1 &= ~UCSWRST;
      while((0 == SPI_CPOL) ? PIN_GET(SPI_B0_CLK) : (!PIN_GET(SPI_B0_CLK)));
    } 
  } else {
    if(SPI_0 == spi) {
      while(SPI_A0_ACTIVE);       /* wait until the data transfer is done */
      UCA0CTL1 |= UCSWRST;
    } else {
      while(SPI_B0_ACTIVE);       /* wait until the data transfer is done */
      UCB0CTL1 |= UCSWRST;      
    }
    /* let the application do whatever it needs to do after disabling SPI */
    SPI_AFTER_DISABLE(spi); 
  }
}
/*---------------------------------------------------------------------------*/
void
spi_read(spi_module_t spi, uint8_t* out_buffer, uint16_t num_bytes)
{ 
  if(num_bytes < 2) {
    *out_buffer = spi_read_byte(spi, 1);
    return;
  }
#if SPI_CONF_FAST_READ
  /* transmit one byte ahead */
  *spi_reg[spi][SPI_REG_TXBUF] = 0x00;
  num_bytes--;
#endif
  do {
    /* dummy write to generate the clock */
    /* wait until the TX buffer is empty */
    while(!(*spi_reg[spi][SPI_REG_IFG] & UCTXIFG));
    *spi_reg[spi][SPI_REG_TXBUF] = 0x00;
    /* wait for the data */
    while(!(*spi_reg[spi][SPI_REG_IFG] & UCRXIFG));
    *out_buffer = *spi_reg[spi][SPI_REG_RXBUF];
    out_buffer++;
    num_bytes--;
  } while(num_bytes);
#if SPI_CONF_FAST_READ
  while(!(*spi_reg[spi][SPI_REG_IFG] & UCRXIFG));
  *out_buffer = *spi_reg[spi][SPI_REG_RXBUF];
#endif  
}
/*---------------------------------------------------------------------------*/
void
spi_write(spi_module_t spi, const uint8_t* data, uint16_t num_bytes)
{
  do {
    /* wait until the TX buffer is empty */
    while(!(*spi_reg[spi][SPI_REG_IFG] & UCTXIFG));
    *spi_reg[spi][SPI_REG_TXBUF] = *data;    
    data++;
    num_bytes--;
  } while(num_bytes);
  /* clear the RX buffer */
  (void)*spi_reg[spi][SPI_REG_RXBUF];
}
/*---------------------------------------------------------------------------*/
inline uint8_t
spi_read_byte(spi_module_t spi, uint8_t wait_for_rxne)
{
  if(SPI_0 == spi) { 
    /* wait until there is data available */
    if(wait_for_rxne) {
      while(!(UCA0IFG & UCRXIFG));
    }
    return UCA0RXBUF;
  } else if(SPI_1 == spi) { 
    /* wait until there is data available */
    if(wait_for_rxne) {
      while(!(UCB0IFG & UCRXIFG));
    }
    return UCB0RXBUF;
  }
  return 0;
}
/*---------------------------------------------------------------------------*/
inline void
spi_write_byte(spi_module_t spi, uint8_t b)
{
  if(SPI_0 == spi) { 
    /* wait until the TX buffer is empty */
    while(!(UCA0IFG & UCTXIFG));
    UCA0TXBUF = b;
  } else if(SPI_1 == spi) { 
    /* wait until the TX buffer is empty */
    while(!(UCB0IFG & UCTXIFG));
    UCB0TXBUF = b;
  }
}
/*---------------------------------------------------------------------------*/
inline void
spi_wait(spi_module_t spi)
{    
  if(SPI_0 == spi) { 
    while(SPI_A0_ACTIVE);
  } else if(SPI_1 == spi) {
    while(SPI_B0_ACTIVE);
  }
}
/*---------------------------------------------------------------------------*/
