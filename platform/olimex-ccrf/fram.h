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
 * @defgroup    fram External FRAM
 * @{
 *
 * @file
 * @author
 *              Reto Da Forno
 *
 * This lib provides the interface towards external serial memory.
 * 
 * @note This library is intended to be used with the 2 Mbit FRAM chip from
 * Cypress (tested with the FM25V20)
 * All operations can either be synchronous (blocking) or asynchronous
 * (non-blocking, DMA-driven).
 * @remark The write latency is approximately 16 + num_bytes * 2.5us for an SPI
 * clock speed of 3.25 MHz. The read latency is approx. 6us shorter.
 */

#ifndef __FRAM_H__
#define __FRAM_H__

#if FRAM_CONF_ON

/* base address of the SPI module used to communicate with the external
 * memory (FRAM) */
#ifndef FRAM_CONF_SPI            
#define FRAM_CONF_SPI           USCI_B0     /* SPI_B0_BASE */
#endif /* FRAM_CONF_SPI */

#ifndef FRAM_CTRL_PIN
#define FRAM_CTRL_PIN           PORT1, PIN7 /* control line for the external
                                             FRAM (SPI chip select / enable
                                             line) */
#endif

#ifndef FRAM_CONF_SCLK_SPEED                /* serial clock speed */
#define FRAM_CONF_SCLK_SPEED    (SMCLK_SPEED)
#endif /* FRAM_CONF_SCLK_SPEED */

#ifndef FRAM_CONF_USE_DMA
#define FRAM_CONF_USE_DMA       0
#endif /* FRAM_CONF_USE_DMA */

#define FRAM_ALLOC_ERROR        0xffffffff  /* this address indicates a memory
                                               allocation error */

/**
 * @brief checks whether the control pin is high
 */
#define FRAM_TRANSFER_IN_PROGRESS   (!PIN_GET_INPUT_BIT(FRAM_CTRL))
/**
 * @brief wait until the data transfer is complete and the control pin is high
 */
#define FRAM_WAIT_COMPLETE          while(!PIN_GET_INPUT_BIT(FRAM_CTRL))



/**
 * @brief initializes the external memory
 * @note      Due to an added safety delay, it takes more than 1ms for this
 *function to complete.
 * @return    zero if an error occurred, one otherwise
 * @remark    This function checks the external memory by reading and verifying
 * the device ID (all device IDs start with the manufacturer ID). Therefore, a
 * non-zero value will only be returned for FRAM chips from Cypress.
 *
 * This function ensures that the external FRAM chip (connected to the MCU over
 * the SPI bus) is available and operates properly.
 */
uint8_t fram_init(void);

/**
 * @brief returns the device ID of the external memory chip
 * @param[out] out_buffer pointer to the output buffer (allocated memory block
 * must be at least 27 bytes long)
 * @param[in]  formatted if unequal zero, the output will be a string (ID coded
 * in hex format), otherwise the output is just an array of 9 bytes
 * @return     the address of the output buffer
 *
 * This function returns the device ID of the external FRAM chip, which is
 * connected to the MCU over the SPI bus.
 */
const char *fram_get_id(char *const out_buffer, uint8_t formatted);

/**
 * @brief puts the external memory into sleep mode
 * @remark This function does nothing in case the external memory already is in
 * LPM.
 * @return zero if an error occurred, one otherwise
 *
 * This function puts the external FRAM chip into a low-power mode.
 */
uint8_t fram_sleep(void);

/**
 * @brief wake the external memory up
 * @remark This function does nothing in case the external memory already is in
 * active mode.
 * @return zero if an error occurred, one otherwise
 *
 * This function puts the external FRAM chip back into active mode.
 */
uint8_t fram_wakeup(void);

/**
 * @brief read data from the external memory
 * @param[in] start_addr start address (first byte to be read)
 * @param[in] num_bytes the number of bytes to be read
 * @param[out] out_data pointer to the output buffer, must be sufficiently
 * large to hold all read bytes
 * @return    zero if an error occurred, one otherwise
 *
 * This function sequentially reads a specified amount of bytes from the
 * external FRAM, starting at the given address.
 */
uint8_t fram_read(uint32_t start_address, uint16_t num_bytes,
                  uint8_t *out_data);

/**
 * @brief write data to the external memory
 * @param[in] start_addr start address (first byte to be written to)
 * @param[in] num_bytes the number of bytes to be written
 * @param[in] data pointer to the input data, must contain [num_bytes] bytes
 * @return    zero if an error occurred, one otherwise
 *
 * This function sequentially writes a specified amount of bytes to the
 * external FRAM, starting at the given address.
 */
uint8_t fram_write(uint32_t start_address,
                   uint16_t num_bytes,
                   const uint8_t *data);

/**
 * @brief erase parts of the external memory
 * @param[in] start_addr start address (first byte to be overwritten)
 * @param[in] num_bytes size of the memory area to be filled with the constant
 * value
 * @param[in] fill_value the value to be used
 * @return    zero if an error occurred, one otherwise
 *
 * This function overwrites the specified area of the external FRAM with the
 * given (constant) value.
 */
uint8_t fram_fill(uint32_t start_address,
                  uint16_t num_bytes,
                  const uint8_t fill_value);

/* memory manager is only available if DMA is not being used for the data
   transfer into the FRAM */
#ifndef FRAM_USE_DMA
/**
 * @brief provides a simple allocation scheme for the external memory
 * @param[in] size the desired size of the memory block to be allocated
 * @return    FRAM_ALLOC_ERROR if the memory is full and the start address of
 * the allocated memory block otherwise
 * @note      Once allocated memory cannot be freed!
 *
 * This function returns the address of the first free memory block of the
 * specified size on the external FRAM.
 */
uint32_t fram_alloc(uint16_t size);
#endif /* FRAM_USE_DMA */

#endif /* FRAM_CONF_ON */

#endif /* __FRAM_H__ */

/**
 * @}
 * @}
 */
