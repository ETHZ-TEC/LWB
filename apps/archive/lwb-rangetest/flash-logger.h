/*
 * Copyright (c) 2016, Swiss Federal Institute of Technology (ETH Zurich).
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
 * @defgroup    flash Flash memory logger
 * @{
 *
 * @file
 *
 * @brief provides utility functions to keep a small amount of data
 * (e.g. program state or statistics) in the non-volatile flash memory
 */

#ifndef __FLASH_LOGGER_H__
#define __FLASH_LOGGER_H__

/* default: use the 2nd info memory segment of the flash */
#define FLASH_LOGGER_CONF_START_ADDR    (INFO_START + INFO_SEG_SIZE)
#define FLASH_LOGGER_CONF_SEG_SIZE      INFO_SEG_SIZE


/* may NOT be bigger than INFO_SEG_SIZE! ideally, size of this struct is an
 * integer multiple of INFO_SEG_SIZE */
typedef struct {
  uint16_t      crc;
  uint8_t       reset_cnt;
  uint8_t       reserved[5];       /* usued */
} flash_logger_block_t;


/**
 * @brief load the most recent data block from the predefined flash memory
 * location
 * @param out_data the buffer to hold the read data
 * @return 1 if successful, 0 otherwise
 */
uint8_t flash_logger_load(flash_logger_block_t* out_data);

/**
 * @brief store a block of data in the predefined flash memory segment
 * @param data data to save
 */
void flash_logger_save(flash_logger_block_t* data);


#endif /* __FLASH_LOGGER_H__ */

/**
 * @}
 * @}
 */
