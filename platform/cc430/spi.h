
/**
 * @brief provides functionality to configure the USCI A0 and B0 modules in SPI mode
 *
 * @file
 * @author rdaforno
 */
  
#ifndef __SPI_H__
#define __SPI_H__


/**
 * @brief disables the specified SPI module
 */
#define SPI_DISABLE(spi)            ( REGVAL8(spi + SPI_CTL1_OFS) |= UCSWRST )
#define SPI_A0_DISABLE              ( UCA0CTL1 |= UCSWRST )
#define SPI_B0_DISABLE              ( UCB0CTL1 |= UCSWRST )

/**
 * @brief enables the specified SPI module
 */
#define SPI_ENABLE(spi)             ( REGVAL8(spi + SPI_CTL1_OFS) &= ~UCSWRST )

/**
 * @brief busy wait until the TX buffer of the specified SPI module is empty
 */
#define SPI_WAIT_TXE(spi)           { while (!(REGVAL8(spi + SPI_IFG_OFS) & UCTXIFG)); }

/**
 * @brief busy wait until data is available, i.e. the RX buffer of the specified SPI module is not empty
 */
#define SPI_WAIT_RXNE(spi)          { while (!(REGVAL8(spi + SPI_IFG_OFS) & UCRXIFG)); }

/**
 * @brief busy wait until all transmissions on the specified SPI are finished
 */
#define SPI_WAIT_BUSY(spi)          { while (REGVAL8(spi + SPI_STAT_OFS) & UCBUSY); }

/**
 * @brief checks if a transmission is ongoing on the specified SPI, same as SPI_A0_ACTIVE or SPI_B0_ACTIVE
 */
#define SPI_IS_BUSY(spi)            ( REGVAL8(spi + SPI_STAT_OFS) & UCBUSY )

/**
 * @brief checks if a transmission is ongoing on SPI A0, same as SPI_IS_BUSY(SPI_A0_BASE)
 */
#define SPI_A0_ACTIVE               ( UCA0STAT & UCBUSY )

/**
 * @brief checks if a transmission is ongoing on SPI B0, same as SPI_IS_BUSY(SPI_B0_BASE)
 */
#define SPI_B0_ACTIVE               ( UCB0STAT & UCBUSY )
 
/**
 * @brief checks the RX buffer overrun error flag (set when the RX buffer is overwritten without reading the previous value)
 */
#define SPI_A0_RX_OVERRUN           ( UCA0STAT & UCOE )
 
/**
 * @brief checks the RX buffer overrun error flag (set when the RX buffer is overwritten without reading the previous value)
 */
#define SPI_B0_RX_OVERRUN           ( UCB0STAT & UCOE )

/**
 * @brief transmit the single byte b over the SPI, i.e. wait until the TX buffer is empty and then write the byte into the buffer
 */
#define SPI_TRANSMIT_BYTE(spi, b)   { SPI_WAIT_TXE(spi); REGVAL8(spi + SPI_TXBUF_OFS) = (b); }

/**
 * @brief write a byte into the TX buffer of the specified SPI
 */
#define SPI_WRITE_BYTE(spi, b)      ( REGVAL8(spi + SPI_TXBUF_OFS) = (b) ) 

/**
 * @brief receive a single byte from the SPI, i.e. wait until data is available and then copy the byte from the RX buffer into b
 */
#define SPI_RECEIVE_BYTE(spi, b)    { SPI_WAIT_RXNE(spi); b = REGVAL8(spi + SPI_RXBUF_OFS); }

/**
 * @brief copy the byte from the RX buffer into b
 */
#define SPI_READ_BYTE(spi, b)       ( b = REGVAL8(spi + SPI_RXBUF_OFS) )

/**
 * @brief clears the RX buffer by reading it
 */
#define SPI_CLEAR_RXBUF(spi)        ( REGVAL8(spi + SPI_RXBUF_OFS) )

#define SPI_CLEARIFG_RX(spi)        ( REGVAL16(spi + UCB0IFG) &= ~UCRXIE )
#define SPI_CLEARIFG_TX(spi)        ( REGVAL16(spi + UCB0IFG) &= ~UCTXIE )

/**
 * @brief reconfigures the USCI module in SPI mode and selects the output of the MUX
 */
#ifdef MUX_SEL
    #define SPI_SELECT(spi)         { if (spi == SPI_A0_BASE && !USCI_A0_SPI_MODE) spi_a0_reinit(); }
#else
    #define SPI_SELECT(spi)         {   }
#endif


void spi_b0_init(uint32_t bit_clk_speed);
void spi_a0_init(uint32_t bit_clk_speed);
void spi_a0_reinit(void);

//extern spi_mode spi_b0_mode;


#endif /* __SPI_H__ */
