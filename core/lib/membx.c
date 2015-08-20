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

#include "membx.h"

/*---------------------------------------------------------------------------*/
void
membx_init(struct membx *m, uint32_t start_addr)
{
  memset(m->count, 0, (m->num + 7) >> 3);
  m->mem = start_addr;
}
/*---------------------------------------------------------------------------*/
uint32_t
membx_alloc(struct membx *m)
{
  unsigned short i, limit;
  uint8_t bit;

  i = m->last;
  limit = m->num;
find_free:
  /* loop through all data units in this memory block */
  for(; i < limit; i++) {       
    bit = (1 << (i & 0x07));
    if(0 == (m->count[i >> 3] & bit)) {      /* bit set? */
      m->count[i >> 3] |= bit;    /* set bit */
      m->last = i;
      m->n_alloc++;
      return m->mem + ((uint32_t)i * (uint32_t)m->size);    /* return the
                                                               address */
    }
  }
  if(m->last != 0 && i == m->num) {
    i = 0;
    limit = m->last;
    goto find_free;
  }
  /* No free block was found, so we return MEMBX_INVALID_ADDR to indicate
     failure to
     allocate block. */
  m->last = 0;
  return MEMBX_INVALID_ADDR;
}
/*---------------------------------------------------------------------------*/
void
membx_free(struct membx *m, uint32_t addr)
{
  unsigned short i = (addr - m->mem) / m->size;
  if((i < m->num) && (m->count[i >> 3] & (1 << (i & 0x07)))) {
    m->count[i >> 3] &= ~(1 << (i & 0x07));    /* clear bit */
    m->n_alloc--;
  }
}
/*---------------------------------------------------------------------------*/
uint32_t
membx_get_next(struct membx *m, uint16_t start_idx)
{
  uint16_t i;
  uint8_t bit;
  if(start_idx >= m->num) { start_idx = 0; }
  i = start_idx;
  /* loop through all data units in this memory block */
  do {                         
    bit = (1 << (i & 0x07));
    if(m->count[i >> 3] & bit) {      /* bit set? */
      /* return the address for the first non-empty block */
      return m->mem + ((uint32_t)i * (uint32_t)m->size);
    }
    i++;
    if(i == m->num) { i = 0; }
  } while(i != start_idx);
  return MEMBX_INVALID_ADDR;    /* no used block found */
}
/*---------------------------------------------------------------------------*/

