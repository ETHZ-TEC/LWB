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
 * @addtogroup  Dev
 * @{
 *
 * @defgroup    xmem External memory
 * @{
 *
 * @file
 * 
 * @brief external memory interface definition
 */


#ifndef __XMEM_H__
#define __XMEM_H__

#define XMEM_ALLOC_ERROR        0xffffffff  /* this address indicates a memory
                                               allocation error */
                                               
/**
 * @brief prototype for the device init function, must be implemented in a
 *  file in the platform directory
 */
inline uint8_t xmem_init(void);

/**
 * @brief prototype for the read function, must be implemented in a file
 * in the platform directory
 */
inline uint8_t xmem_read(uint32_t start_address, uint16_t num_bytes,
                         uint8_t *out_data);

/**
 * @brief prototype for the write function, must be implemented in a file
 * in the platform directory
 */
inline uint8_t xmem_write(uint32_t start_address, uint16_t num_bytes,
                          const uint8_t *data);

/**
 * @brief prototype for the erase function, must be implemented in a file
 * in the platform directory
 */
inline uint8_t xmem_erase(uint32_t start_address, uint16_t num_bytes);

/**
 * @brief prototype for the memory allocation routine, must be implemented 
 * in a file in the platform directory
 */
inline uint32_t xmem_alloc(uint32_t size);


inline uint8_t xmem_sleep(void);

inline uint8_t xmem_wakeup(void);



#endif /* __XMEM_H__ */

/**
 * @}
 * @}
 */