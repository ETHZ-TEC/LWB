/*
 * Copyright (c) 2004, Swedish Institute of Computer Science.
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
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * This file is part of the Contiki operating system.
 *
 * Author: Adam Dunkels <adam@sics.se>
 *
 * --------------------------------------------------------------------------
 *
 * File modified by rdaforno, 2014
 * Changes:
 * - comments added, CONCAT() macros removed
 * - memory usage for meta data (block used/unused) reduced by a factor of 8
 * - data unit size (structure size) reduced to 8-bit, i.e. max. size is 255 bytes
 * - variable added to remember the most recently allocated slot within the memory block (speeds-up the search for an empty block)
 * - support for external memory added
 * - return type for memb_free changed to void
 */

/**
 * \addtogroup mem
 * @{
 */

 
#define XMEM_INVALID_ADDR    0xffffffff
 

/**
 * \defgroup memb Memory block management functions
 *
 * The memory block allocation routines provide a simple yet powerful
 * set of functions for managing a set of memory blocks of fixed
 * size. A set of memory blocks is statically declared with the
 * MEMB() macro. Memory blocks are allocated from the declared
 * memory by the memb_alloc() function, and are deallocated with the
 * memb_free() function.
 *
 * @{
 */


/**
 * \file
 *         Memory block allocation routines.
 * \author
 *         Adam Dunkels <adam@sics.se>
 *
 */

#ifndef __MEMB_H__
#define __MEMB_H__

#include "sys/cc.h"

/**
 * Declare a memory block (in RAM).
 *
 * This macro is used to statically declare a block of memory that can
 * be used by the block allocation functions. The macro statically
 * declares a C array with a size that matches the specified number of
 * blocks and their individual sizes.
 *
 * Example:
 \code
MEMB(connections, struct connection, 16);
 \endcode
 *
 * \param name The name of the memory block (later used with
 * memb_init(), memb_alloc() and memb_free()).
 *
 * \param structure The name of the struct that the memory block holds
 *
 * \param num The total number of memory chunks in the block.
 *
 */
#define MEMB(name, structure, num) \
        static char name##_memb_count[(num + 7) >> 3]; \
        static structure name##_memb_mem[num]; \
        static struct memb name = { sizeof(structure), 0, num, name##_memb_count, (void *)name##_memb_mem }

        
/**
 * Declare a memory block (in external memory).
 *
 * @note 9 + (num_units + 7) / 8 bytes are required to store the meta data
 * @author rdaforno
 */
#define MEMBX(name, elem_size, num) \
        static char name##_memb_count[(num + 7) >> 3]; \
        static struct membx name = { elem_size, 0, num, 0, name##_memb_count, 0 }

                
        
        
// structure for a memory block
struct memb {
  unsigned char size;   // size of one data unit
  unsigned short last;  // last allocated data unit
  unsigned short num;   // number of data units in this memory block
  char *count;          // meta data: stores whether a block is used (allocated)
  void *mem;            // pointer to the beginning of the data block
};
       
// structure for a memory block
struct membx {
  unsigned char size;   // size of one data unit
  unsigned short last;  // last allocated data unit
  unsigned short num;   // number of data units in this memory block
  unsigned short n_alloc;   // number of allocated data units
  char *count;          // meta data: stores whether a block is used (allocated)
  uint32_t mem;         // pointer to the beginning of the data block (do not dereference this address!)
};

/**
 * Initialize a memory block that was declared with MEMB().
 *
 * \param m A memory block previously declared with MEMB().
 */
void memb_init(struct memb *m);
void membx_init(struct membx *m);

/**
 * Allocate a memory block from a block of memory declared with MEMB().
 *
 * \param m A memory block previously declared with MEMB().
 */
void *memb_alloc(struct memb *m);
uint32_t membx_alloc(struct membx *m);

/**
 * Deallocate a memory block from a memory block previously declared
 * with MEMB().
 *
 * \param m m A memory block previously declared with MEMB().
 *
 * \param ptr A pointer to the memory block that is to be deallocated.
 *
 * \return The new reference count for the memory block (should be 0
 * if successfully deallocated) or -1 if the pointer "ptr" did not
 * point to a legal memory block.
 */
void memb_free(struct memb *m, void *ptr);
void membx_free(struct membx *m, uint32_t ptr);

/**
 * Returns the address of the first non-empty block from a memory block previously declared
 * with MEMBX().
 *
 * \param m memory block previously declared with MEMBX().
 * \param start_idx search for a non-empty block will start at this index
 *
 * \return address of the found block, XMEM_INVALID_ADDR otherwise
 */
uint32_t membx_get_next(struct membx *m, uint16_t start_idx);

int memb_inmemb(struct memb *m, void *ptr);


/** @} */
/** @} */

#endif /* __MEMB_H__ */
