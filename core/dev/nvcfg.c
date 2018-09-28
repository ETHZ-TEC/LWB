/*
 * Copyright (c) 2018, Swiss Federal Institute of Technology (ETH Zurich).
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
nvcfg_load(void* out_data)
{
  uint16_t addr = NVCFG_CONF_START_ADDR;

  /* find the first empty block */
  while((addr < (NVCFG_CONF_START_ADDR + NVCFG_CONF_SEG_SIZE)) &&
        REGVAL16(addr) != 0xffff) {
    addr += NVCFG_BLOCK_SIZE_WITH_CRC;
  }
  if(NVCFG_CONF_START_ADDR == addr) {   /* 1st block is empty? */
    return 0; /* no data to load */
  }
  addr -= NVCFG_BLOCK_SIZE_WITH_CRC;  /* go back to previous block */

  /* check the CRC (first 2 bytes of the block are the CRC!) */
  uint16_t crc_calc = crc16((uint8_t*)addr + 2, NVCFG_CONF_BLOCK_SIZE, 0);
  if(crc_calc == 0xffff) {
    crc_calc--;             /* CRC mustn't be 0xffff */
  }
  if(crc_calc == *(uint16_t*)addr) {
    memcpy(out_data, (uint8_t*)addr + 2, NVCFG_CONF_BLOCK_SIZE);
    return 1;
  }
  return 0;   /* invalid CRC */
}
/*---------------------------------------------------------------------------*/
uint8_t
nvcfg_save(void* data)
{
  uint16_t addr = NVCFG_CONF_START_ADDR;
  uint16_t tries = NVCFG_CONF_ERASE_RETRY;

  /* find the first empty block */
  while(addr < (NVCFG_CONF_START_ADDR + NVCFG_CONF_SEG_SIZE) &&
        REGVAL16(addr) != 0xffff) {
    addr += NVCFG_BLOCK_SIZE_WITH_CRC;
  }
  /* no empty slot found? */
  if((addr + NVCFG_BLOCK_SIZE_WITH_CRC) >
     (NVCFG_CONF_START_ADDR + NVCFG_CONF_SEG_SIZE)) {
    /* erase the segment */
    do {
      flash_erase_info_segment((uint8_t*)NVCFG_CONF_START_ADDR);
      tries--;
    } while(!flash_erase_check((uint8_t*)NVCFG_CONF_START_ADDR,
                               NVCFG_CONF_SEG_SIZE) && tries);
    /* basic wear leveling protection: wait 100ms, just in case nvcfg_save is
     * accidentially called in a loop */
    __delay_cycles(MCLK_SPEED / 10);
    if(tries == 0) {
      return 0; /* failed */
    }
    addr = NVCFG_CONF_START_ADDR;
  }
  /* copy the data and calculate the CRC */
  uint16_t buffer[NVCFG_BLOCK_SIZE_WITH_CRC / 2];      /* block size is even */
  memcpy(&buffer[1], data, NVCFG_CONF_BLOCK_SIZE);
  uint16_t crc_calc = crc16((uint8_t*)data, NVCFG_CONF_BLOCK_SIZE, 0);
  if(crc_calc == 0xffff) {
    crc_calc--;             /* CRC mustn't be 0xffff! */
  }
  buffer[0] = crc_calc;
  /* save the data */
  tries = NVCFG_CONF_ERASE_RETRY;
  do {
    flash_write((uint8_t*)buffer, (uint8_t*)addr, NVCFG_BLOCK_SIZE_WITH_CRC);
    tries--;
  } while(!flash_verify((uint8_t*)buffer, (uint8_t*)addr,
                        NVCFG_BLOCK_SIZE_WITH_CRC) && tries);
  return (tries > 0);
}
/*---------------------------------------------------------------------------*/
