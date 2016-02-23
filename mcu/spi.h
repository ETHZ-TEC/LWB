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
 */

#ifndef __SPI_H__
#define __SPI_H__

#ifndef SPI_CONF_NUM_MODULES
#error "SPI_CONF_NUM_MODULES not defined!"
#endif /* SPI_CONF_NUM_MODULES */

#ifndef SPI_CONF_FAST_READ
#define SPI_CONF_FAST_READ      0
#endif /* SPI_CONF_FAST_READ */


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
 * 
 * by default, clock polarity and phase is the same for all SPI modules
 */
#ifndef SPI_CPOL
#define SPI_CPOL        0   /* clock polarity (0 = inactive low) */
#endif
#ifndef SPI_CPHA
#define SPI_CPHA        0   /* clock phase */
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
#define SPI_AFTER_DISABLE(spi)
#endif /* SPI_AFTER_DISABLE */


typedef enum {
  SPI_0 = 0,
  SPI_1,
#if SPI_CONF_NUM_MODULES > 2
  SPI_2,
#endif 
#if SPI_CONF_NUM_MODULES > 3
  SPI_3,
#endif 
#if SPI_CONF_NUM_MODULES > 4
  SPI_4,
#endif 
#if SPI_CONF_NUM_MODULES > 5
  SPI_5,
#endif 
#if SPI_CONF_NUM_MODULES > 6
  SPI_6,
#endif 
#if SPI_CONF_NUM_MODULES > 7
  SPI_7,
#endif 
  NUM_OF_SPI_MODULES
} spi_module_t; 


/**
 * @brief initialize a USCI module in SPI mode
 * @param[in] bclk_speed the desired serial/bit clock frequency in Hz
 * @param[in] spi the USCI module to use, must be of type spi_module_t
 * @return 1 if successful, 0 otherwise
 * @note this function does not enable the SPI module
 * @remark clock phase and polarity can be set with the definitions
 * SPI_CPOL and SPI_CPHA
 */
uint8_t spi_init(spi_module_t spi, uint32_t bclk_speed);

/**
 * @brief re-initialize the SPI A0 module (when it was configured in UART mode
 * before)
 * @note this function does not enable the SPI module
 *
 * Re-configure USCI A0 peripheral in SPI mode. Use this function to switch
 * from UART to SPI mode.
 */
void spi_reinit(spi_module_t spi);

/**
 * @brief enable or disable an SPI (USCI) module
 * @param spi the USCI module, must be of type spi_module_t
 * @param enable 0 = disable, 1 = enable
 */
void spi_enable(spi_module_t spi, uint8_t enable);

/**
 * @brief receive several bytes from the SPI
 * @param spi the USCI module, must be of type spi_module_t
 * @param out_buffer the output buffer, must be at least num_bytes long
 * @param num_bytes the number of bytes to read
 * @note this function will generate the SPI clock for num_bytes bytes
 * this is a blocking call (polling based)
 */
void spi_read(spi_module_t spi, uint8_t* out_buffer, uint16_t num_bytes);

/**
 * @brief write several bytes to the SPI
 * @param spi the USCI module, must be of type spi_module_t
 * @param data the input data, must be num_bytes long
 * @param num_bytes the number of bytes to read
 * @note this is a blocking call (polling based)
 */
void spi_write(spi_module_t spi, const uint8_t* data, uint16_t num_bytes);

/**
 * @brief receive a single byte from the SPI, i.e. wait until data is available
 * (if wait_for_rxne is set to 1) and then read the RX buffer
 * @param spi the USCI module, must be of type spi_module_t
 * @param wait_for_rxne 1 = wait until data is available, 0 = read right away 
 * (e.g. to make sure the RX buffer is cleared)
 * @return the read byte
 */
inline uint8_t spi_read_byte(spi_module_t spi, uint8_t wait_for_rxne);


/**
 * @brief write a single byte to the SPI, i.e. wait until the TXE interrupt 
 * flag is set and then copy b into the TX buffer
 */
inline void spi_write_byte(spi_module_t spi, uint8_t b);

/**
 * @brief wait until all ongoing data transfers on the SPI bus have terminated
 * @param spi the USCI module, must be of type spi_module_t
 */
inline void spi_wait(spi_module_t spi);


#endif /* __SPI_H__ */

/**
 * @}
 * @}
 */
