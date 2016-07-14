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
 * @defgroup    flash Flash Memory
 * @{
 *
 * @file
 *
 * @brief provides utility functions for the flash memory
 */

#ifndef __FLASH_H__
#define __FLASH_H__

/* memory definitions */
#define FLASH_START         0x8000
#define FLASH_END           0xffff      /* last byte of the flash memory */
#define FLASH_SIZE          0x8000      /* 32 kB */    
#define FLASH_SEG_SIZE      0x0200      /* 512 B */
#define FLASH_INVALID       0x0000      /* invalid address (indicate errors) */
#define CODE_START          FLASH_START
#define CODE_END            (FLASH_END - INT_VECT_TABLE_SIZE)
#define CODE_SIZE           0x7f80      /* 32640 B */
#define INT_TABLE_START     0xff80      /* interrupt vector table */
#define INT_TABLE_SIZE      0x0080      /* 128 B */
#define INFO_START          0x1800
#define INFO_END            0x19ff
#define INFO_SEG_SIZE       0x0080      /* 128 B */
#define INFO_SIZE           0x0200      /* 512 B */
#define BSL_START           0x1000
#define BSL_END             0x17ff
#define BSL_SIZE            0x0800      /* 8 * 256 = 2 kB */


/**
 * @brief counts the unused bytes in flash memory to get an estimate on the
 * code size
 * @return the code size (.text section) in the flash memory
 */
uint16_t flash_code_size(void);

/**
 * @brief erase a flash segment (main memory)
 * @param addr address of the flash segment to erase
 */
void flash_erase_segment(uint8_t* addr);


/**
 * @brief erase a flash segment of the information memory
 * @param addr address of the flash segment to erase
 */
void flash_erase_info_segment(uint8_t* addr);

/**
 * @brief erase the whole flash memory bank (mass erase, main memory only,
 * info memory is not touched)
 */
void flash_erase_bank(void);

/**
 * @brief check whether the flash erase operation was successful
 * @param addr start address in the flash memory (pass 0 to check the whole 
 * flash memory bank)
 * @param num_bytes number of bytes to check (may be 0)
 * @return 1 if successful, 0 otherwise
 */
uint8_t flash_erase_check(uint8_t* addr, uint16_t num_bytes);

/**
 * @brief write data words into the flash memory
 * @param data pointer to the data
 * @param flash_addr start address of the destination in flash memory
 * @param num_bytes number of bytes to program
 */
void flash_write(uint8_t* data, uint8_t* flash_addr, uint16_t num_bytes);


#endif /* __FLASH_H__ */

/**
 * @}
 * @}
 */