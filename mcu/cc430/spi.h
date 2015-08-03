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

/**
 * @addtogroup  Platform
 * @{
 *
 * @defgroup    spi SPI
 * @{
 *
 * @brief provides functionality to configure the USCI A0 and B0 modules in SPI
 * mode (as master)
 *
 * @file
 * @author rdaforno
 */

#ifndef __SPI_H__
#define __SPI_H__


#ifndef SPI_CONF_FAST_READ
#define SPI_CONF_FAST_READ      0
#endif /* SPI_CONF_FAST_READ */

/* hardware addresses */
#define SPI_A0_BASE             USCI_A0     /* 0x05c0 */
#define SPI_B0_BASE             USCI_B0     /* 0x05e0 */
#define SPI_CTL0_OFS            0x0001      /* ctrl reg 0 offset */
#define SPI_CTL1_OFS            0x0000      /* ctrl reg 1 offset */
#define SPI_BR_OFS              0x0006      /* baud-rate offset */
#define SPI_TXBUF_OFS           0x000E      /* TX buffer offset */
#define SPI_IE_OFS              0x001C      /* interrupt enable reg offset */
#define SPI_IFG_OFS             0x001D      /* interrupt flag reg offset */
#define SPI_STAT_OFS            0x000A      /* status reg offset */
#define SPI_RXBUF_OFS           0x000C

/* pin definitions (not configurable, thus without _CONF */
#define SPI_B0_SOMI             PORT1, PIN2
#define SPI_B0_SIMO             PORT1, PIN3
#define SPI_B0_CLK              PORT1, PIN4
#define SPI_B0_STE              PORT1, PIN7 /* enable / select pin */
#define SPI_A0_SOMI             PORT1, PIN5
#define SPI_A0_SIMO             PORT1, PIN6
#define SPI_A0_CLK              PORT1, PIN7

/*
 * Note on the SPI phase and polarity:
 * For CPOL = 0 and ...
 * ... CPHA = 0, data is captured on the clock's rising edge and changed
 *     (latched) on the falling edge.
 * ... CPHA = 1, data is captured on the clock's falling edge and changed
 *     (latched) on the rising edge.
 * For CPOL = 1 and ...
 * ... CPHA = 0, data is captured on the clock's falling edge and changed
 *     (latched) on the rising edge.
 * ... CPHA = 1, data is captured on the clock's rising edge and changed
 *     (latched) on the falling edge.
 */
#ifndef SPI_A0_CPOL
#define SPI_A0_CPOL             0
#endif
#ifndef SPI_A0_CPHA
#define SPI_A0_CPHA             0
#endif
#ifndef SPI_B0_CPOL
#define SPI_B0_CPOL             0   /* clock polarity (0 = inactive low) */
#endif
#ifndef SPI_B0_CPHA
#define SPI_B0_CPHA             0   /* clock phase */
#endif

/**
 * @brief define this macro in a platform specific file to specify the
 * instructions that need to be executed before enabling SPI
 */
#ifndef SPI_BEFORE_ENABLE
#define SPI_BEFORE_ENABLE(spi)
#endif /* SPI_BEFORE_ENABLE */

/**
 * @brief define this macro in a platform specific file to specify the
 * instructions that need to be executed after disabling SPI
 */
#ifndef SPI_AFTER_DISABLE
#define SPI_AFTER_DISABLE
#endif /* SPI_AFTER_DISABLE */

/**
 * @brief disables the specified SPI module
 */
#define SPI_DISABLE(spi)        { /*SPI_WAIT_BUSY(spi);*/ \
                                  REGVAL8(spi + SPI_CTL1_OFS) |= UCSWRST; \
                                  SPI_AFTER_DISABLE; }

/**
 * @brief enables the specified SPI module
 */
#define SPI_ENABLE(spi)         { SPI_BEFORE_ENABLE(spi); \
                                  REGVAL8(spi + SPI_CTL1_OFS) &= ~UCSWRST; }

/**
 * @brief busy wait until the TX buffer of the specified SPI module is empty
 */
#define SPI_WAIT_TXE(spi)       { while(!(REGVAL8(spi + SPI_IFG_OFS) & \
                                          UCTXIFG)); }

/**
 * @brief busy wait until data is available, i.e. the RX buffer of the
 * specified SPI module is not empty
 */
#define SPI_WAIT_RXNE(spi)      { while(!(REGVAL8(spi + SPI_IFG_OFS) & \
                                          UCRXIFG)); }

/**
 * @brief busy wait until all transmissions on the specified SPI are finished
 */
#define SPI_WAIT_BUSY(spi)      { while(REGVAL8(spi + SPI_STAT_OFS) & \
                                        UCBUSY); }

/**
 * @brief checks if a transmission is ongoing on the specified SPI, same as
 * SPI_A0_ACTIVE or SPI_B0_ACTIVE
 */
#define SPI_IS_BUSY(spi)        (REGVAL8(spi + SPI_STAT_OFS) & UCBUSY)

/**
 * @brief checks if a transmission is ongoing on SPI A0, same as
 * SPI_IS_BUSY()
 */
#define SPI_A0_ACTIVE           (UCA0STAT & UCBUSY)

/**
 * @brief checks if a transmission is ongoing on SPI B0, same as
 * SPI_IS_BUSY()
 */
#define SPI_B0_ACTIVE           (UCB0STAT & UCBUSY)

/**
 * @brief checks the RX buffer overrun error flag (set when the RX buffer is
 * overwritten without reading the previous value)
 */
#define SPI_A0_RX_OVERRUN       (UCA0STAT & UCOE)

/**
 * @brief checks the RX buffer overrun error flag (set when the RX buffer is
 * overwritten without reading the previous value)
 */
#define SPI_B0_RX_OVERRUN       (UCB0STAT & UCOE)

/**
 * @brief transmit the single byte b over the SPI, i.e. wait until the TX
 * buffer is empty and then write the byte into the buffer
 */
#define SPI_TRANSMIT_BYTE(spi, b)  { SPI_WAIT_TXE(spi); REGVAL8( \
                                        spi + SPI_TXBUF_OFS) = (b); }

/**
 * @brief write a byte into the TX buffer of the specified SPI
 */
#define SPI_WRITE_BYTE(spi, b)  (REGVAL8(spi + SPI_TXBUF_OFS) = (b))

/**
 * @brief receive a single byte from the SPI, i.e. wait until data is available
 *and then copy the byte from the RX buffer into b
 */
#define SPI_RECEIVE_BYTE(spi, b) { SPI_WAIT_RXNE(spi); \
                                   (b) = REGVAL8(spi + SPI_RXBUF_OFS); }

/**
 * @brief copy the byte from the RX buffer into b
 */
#define SPI_READ_BYTE(spi, b)   ((b) = REGVAL8(spi + SPI_RXBUF_OFS))

/**
 * @brief clears the RX buffer by reading it
 */
#define SPI_CLR_RXBUF(spi)      (REGVAL8(spi + SPI_RXBUF_OFS))

#define SPI_CLRIFG_RX(spi)      (REGVAL16(spi + UCB0IFG) &= ~UCRXIE)
#define SPI_CLRIFG_TX(spi)      (REGVAL16(spi + UCB0IFG) &= ~UCTXIE)

/**
 * @brief initialize the SPI A0 module
 * @param[in]  bit_clk_speed the desired serial clock frequency in Hz
 * @note this function does not enable the SPI module
 * @remark     clock phase and polarity can be set with the definitions
 * SPI_A0_CPOL and SPI_A0_CPHA in hal.h
 *
 * Configure USCI A0 peripheral in SPI mode as follows: Master, MSB first,
 * 8-bit, 3lines.
 */
void spi_a0_init(uint32_t bit_clk_speed);

/**
 * @brief initialize the SPI B0 module
 * @param[in]  bit_clk_speed the desired serial clock frequency in Hz
 * @note this function does not enable the SPI module
 * @remark     clock phase and polarity can be set with the definitions
 * SPI_B0_CPOL and SPI_B0_CPHA in hal.h
 *
 * Configure USCI B0 peripheral in SPI mode as follows: Master, MSB first,
 * 8-bit, 3 lines.
 */
void spi_b0_init(uint32_t bit_clk_speed);

/**
 * @brief re-initialize the SPI A0 module (when it was configured in UART mode
 * before)
 * @note this function does not enable the SPI module
 *
 * Re-configure USCI A0 peripheral in SPI mode. Use this function to switch
 * from UART to SPI mode.
 */
void spi_a0_reinit(void);

#endif /* __SPI_H__ */

/**
 * @}
 * @}
 */
