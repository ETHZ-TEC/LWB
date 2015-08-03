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
 * @defgroup    flash Flash Memory
 * @{
 *
 * @file
 * @author
 *              Reto Da Forno
 *
 * @brief provides utility functions for the flash memory
 */

#ifndef __FLASH_H__
#define __FLASH_H__

/* memory definitions */
#define FLASH_START         0x8000
#define FLASH_END           0xff7f      /* last byte of the flash memory (note:
                                           the last 128 bytes are used for the
                                           interrupt vectors!) */
#define FLASH_LAST_SEG      0xfe00      /* last erasable segment (do not touch
                                           the very last segment!) */
#define FLASH_SIZE          0x7f80      /* 32640 B */
#define INFO_START          0x1800
#define INFO_END            0x19ff
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

#endif /* __FLASH_H__ */

/**
 * @}
 * @}
 */