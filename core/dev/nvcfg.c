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

/*
 * non-volatile configuration storage (in flash memory)
 *
 * How it works:
 * - memory segment is all 0xffff at the beginning
 * - on every startup, we search for an existing data block (start at pos. 0,
 *   last occurrence of != 0xffff is the block we're looking for)
 * - if it exists (that is, if the beginning of the first block is != 0xffff)
 *   then read it and check the CRC; if CRC wrong, reject it
 * - write the new data block to the first unused position; if the whole 
 *   segment has been used, then erase it and start again at pos. 0
 */

#include "contiki.h"

/*---------------------------------------------------------------------------*/
uint8_t
nvcfg_load(uint8_t* out_data)
{ 
  uint16_t addr = NVCFG_CONF_START_ADDR;
  uint16_t end  = NVCFG_CONF_START_ADDR + NVCFG_CONF_SEG_SIZE;
  
  if(*(uint16_t*)addr == 0xffff) {
    return 0;   /* no data available */
  }
  addr += NVCFG_CONF_BLOCK_SIZE + 2;
  
  while(addr < end) {
    if(*(uint16_t*)addr == 0xffff) {
      break;
    }
    addr += (NVCFG_CONF_BLOCK_SIZE + 2);
  }
  addr -= (NVCFG_CONF_BLOCK_SIZE + 2);
  
  /* check the CRC (first 2 bytes of the block are the CRC!) */
  if(crc16((uint8_t*)addr + 2, NVCFG_CONF_BLOCK_SIZE, 0) ==
     *(uint16_t*)addr) {
    memcpy(out_data, (uint8_t*)addr + 2, NVCFG_CONF_BLOCK_SIZE);
    return 1;
  }
  return 0;
}
/*---------------------------------------------------------------------------*/
uint8_t
nvcfg_save(uint8_t* data)
{
  uint16_t addr = NVCFG_CONF_START_ADDR;     /* seg. 2 */
  uint16_t end  = NVCFG_CONF_START_ADDR + NVCFG_CONF_SEG_SIZE;
  
  /* find the first empty block */
  while(addr < end) {
    if(*(uint16_t*)addr == 0xffff && *(uint16_t*)(addr + 2) == 0xffff) {
      break;
    }
    addr += NVCFG_CONF_BLOCK_SIZE + 2;
  }
  if(addr >= end) {        /* no empty slot found? */
    /* erase the segment */
    flash_erase_info_segment((uint8_t*)NVCFG_CONF_START_ADDR);
    if(!flash_erase_check((uint8_t*)NVCFG_CONF_START_ADDR,
                          NVCFG_CONF_SEG_SIZE)) {
      //DEBUG_PRINT_MSG_NOW("ERROR: failed to erase flash segment!");
      return 0;
    }
    __delay_cycles(MCLK_SPEED / 2);   /* wait 0.5s */
    addr = NVCFG_CONF_START_ADDR;
  }
  /* update the CRC */
  uint16_t buffer[NVCFG_CONF_BLOCK_SIZE / 2 + 1];
  memcpy(&buffer[1], data, NVCFG_CONF_BLOCK_SIZE);
  buffer[0] = crc16((uint8_t*)data, NVCFG_CONF_BLOCK_SIZE, 0);
  /* save the data */
  flash_write((uint8_t*)buffer, (uint8_t*)addr, NVCFG_CONF_BLOCK_SIZE + 2);

  return 1;
}
/*---------------------------------------------------------------------------*/
