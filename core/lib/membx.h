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
 * @brief Memory block management functions for external FRAM
 *
 * @file
 *         Memory block allocation routines.
 */

#ifndef __MEMBX_H__
#define __MEMBX_H__

#include <string.h>

#define MEMBX_INVALID_ADDR      0xffffffff

/**
 * @brief declare a memory block (in external memory)
 * @param elem_size size of one data element
 * @param num number of data elements in this memory block
 * @note 9 + (num_units + 7) / 8 bytes are required to store the meta data
 */
#define MEMBX(name, elem_size, num) \
  static char name##_memb_count[(num + 7) >> 3]; \
  static struct membx name = { elem_size, 0, num, 0, name##_memb_count, 0 }

/**
 * @brief structure for a memory block
 */
struct membx {
  unsigned char size;   /* size of one data unit */
  unsigned short last;  /* last allocated data unit */
  unsigned short num;   /* number of data units in this memory block */
  unsigned short n_alloc;   /* number of allocated data units */
  char *count;    /* meta data: stores whether a block is used (allocated) */
  uint32_t mem;         /* pointer to the beginning of the data block (do not
                           dereference this address!) */
};

/**
 * @brief initialize a memory block that was declared with MEMBX()
 * @param m A memory block previously declared with MEMBX().
 * @param start_addr the address of the first element in this memory block
 */
void membx_init(struct membx *m, uint32_t start_addr);

/**
 * @brief allocate a memory block from a block of memory declared with MEMBX()
 * @param m A memory block previously declared with MEMBX().
 * @return (virtual) start address of the allocated memory
 */
uint32_t membx_alloc(struct membx *m);

/**
 * @brief deallocate a memory block from a memory block previously declared
 * with MEMBX()
 * @param m m A memory block previously declared with MEMBX().
 * @param addr address of the memory block to be deallocated
 */
void membx_free(struct membx *m, uint32_t addr);

/**
 * @brief get the address of the first non-empty block from a memory block
 * previously declared with MEMBX()
 * @param m memory block previously declared with MEMBX().
 * @param start_idx search for a non-empty block will start at this index
 * @return the start address of the block if successful, 
 * MEMBX_INVALID_ADDR otherwise
 */
uint32_t membx_get_next(struct membx *m, uint16_t start_idx);


#endif /* __MEMBX_H__ */
