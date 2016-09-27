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
 * @brief non-volatile configuration storage (in flash memory)
 */

#ifndef __NVCFG_H__
#define __NVCFG_H__

/* default: use the 2nd info memory segment of the flash
 * note: if this is changed to the main flash memory, then nvcfg_save must
 * be adjusted as well */
#define NVCFG_CONF_START_ADDR   (INFO_START + INFO_SEG_SIZE)
#define NVCFG_CONF_SEG_SIZE     INFO_SEG_SIZE

/* must be an even number and must not be bigger than NVCFG_CONF_SEG_SIZE!
 * ideally, INFO_SEG_SIZE is an integer multiple of BLOCK_SIZE */
#ifndef NVCFG_CONF_BLOCK_SIZE
#define NVCFG_CONF_BLOCK_SIZE   8         /* length without CRC */
#endif /* NVCFG_CONF_BLOCK_SIZE */

/* sanity check */
#if NVCFG_CONF_BLOCK_SIZE > NVCFG_CONF_SEG_SIZE || NVCFG_CONF_BLOCK_SIZE & 0x1
#error "invalid NVCFG_CONF_BLOCK_SIZE"
#endif /* NVCFG_CONF_BLOCK_SIZE check */

/**
 * @brief load the most recent data block from the predefined flash memory
 * location
 * @param out_data the buffer to hold the loaded data, must be at least
 * NVCFG_CONF_BLOCK_SIZE bytes long
 * @return 1 if successful (i.e. CRC ok), 0 otherwise
 * @note the crc checksum will not be copied into out_data!
 */
uint8_t nvcfg_load(uint8_t* out_data);

/**
 * @brief store a block of data in the predefined flash memory segment
 * @param data data to save, must be exactly NVCFG_CONF_BLOCK_SIZE bytes long
 * @return 1 if successful, 0 otherwise
 */
uint8_t nvcfg_save(uint8_t* data);


#endif /* __NVCFG_H__ */

/**
 * @}
 * @}
 */
