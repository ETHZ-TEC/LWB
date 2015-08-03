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

#ifndef __FIFO_H__
#define __FIFO_H__

#include <string.h>

/**
 * @file
 *
 * First-in, first-out queue based on a linear data array. All elements
 * have the same maximum size. 
 * This lib provides address management only, no actual memory allocation.
 * Therefore, it is suitable for any type of memory (RAM, Flash or 
 * external memory).
 */

#define FIFO_ERROR      0xffffffff

/**
 * @brief declare a FIFO
 * @param start start address of the data array
 * @param elem_size size of one data element
 * @param num number of data elements in this memory block
 * @note It is the users responsibility to allocate a memory block of the
 * size 'elem_size * num' starting at the address 'start_addr'. The FIFO
 * itself takes 12 bytes.
 */
#define FIFO(name, elem_size, num) \
  static struct fifo name = { 0, elem_size, num - 1, 0, 0 }// better to save the last index than the number of elements
  
struct fifo {
  uint32_t start;     /* start address of the array */
  uint16_t size;      /* size of one data unit */
  uint16_t last;      /* number of data units in this memory block */
  uint16_t read;      /* the read pointer */
  uint16_t write;     /* the write pointer */
};

#define FIFO_RESET(f)                   ( (f)->read = (f)->write = 0 )
#define FIFO_EMPTY(f)                   ( (f)->read == (f)->write )
#define FIFO_FULL(f)                    ( ((f)->write < (f)->read) ? ( ((f)->write + 1) == (f)->read ) : ((f)->read == 0 && (f)->write == (f)->last) )
#define FIFO_READ_ADDR(f)               ( (f)->start + ((uint32_t)(f)->read * (uint32_t)(f)->size) )
#define FIFO_WRITE_ADDR(f)              ( (f)->start + ((uint32_t)(f)->write * (uint32_t)(f)->size) )
#define FIFO_INCR_READ(f)               ( (f)->read = (((f)->read == (f)->last) ? 0 : ((f)->read + 1) ) )     /* increment the read index */
#define FIFO_INCR_WRITE(f)              ( (f)->write = (((f)->write == (f)->last) ? 0 : ((f)->write + 1) ) )  /* increment the write index */

static inline void
fifo_init(struct fifo * const f, uint32_t start_addr)
{
    f->start = start_addr;
}

/**
 * @brief delivers the address of the next element to read and shifts the 
 * read pointer forward by 1 element
 * @return the address of the next element to read or FIFO_ERROR if
 * the queue is empty
 */
static inline uint32_t
fifo_get(struct fifo * const f) 
{
  if(FIFO_EMPTY(f)) { return FIFO_ERROR; }
  uint32_t next_read = FIFO_READ_ADDR(f);
  FIFO_INCR_READ(f);
  return next_read;
}

/**
 * @brief shifts the write pointer to the next position
 * @return the address of the next free element or FIFO_ERROR if the
 * queue is full
 */
static inline uint32_t
fifo_put(struct fifo * const f) 
{
  if(FIFO_FULL(f)) { return FIFO_ERROR; }
  uint32_t next_write = FIFO_WRITE_ADDR(f);
  FIFO_INCR_WRITE(f);
  return next_write;
}


#endif /* __FIFO_H__ */
