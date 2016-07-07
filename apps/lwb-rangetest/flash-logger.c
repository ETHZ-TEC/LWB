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
#include "flash-logger.h"

/*---------------------------------------------------------------------------*/
uint8_t
flash_logger_load(flash_logger_block_t* out_data) 
{ 
  uint16_t addr = FLASH_LOGGER_CONF_START_ADDR;
  uint16_t end  = FLASH_LOGGER_CONF_START_ADDR + FLASH_LOGGER_CONF_SEG_SIZE;
  
  if(*(uint16_t*)addr == 0xffff) {
    return 0;   /* no data available */
  }
  addr += sizeof(flash_logger_block_t);
  
  while(addr < end) {
    if(*(uint16_t*)addr == 0xffff) {
      break;
    }
    addr += sizeof(flash_logger_block_t);
  }
  addr -= sizeof(flash_logger_block_t);
  
  /* check the CRC (first 2 bytes of the block are the CRC!) */
  if(crc16((uint8_t*)addr + 2, sizeof(flash_logger_block_t) - 2, 0) ==
     *(uint16_t*)addr) {
    memcpy(out_data, (uint8_t*)addr, sizeof(flash_logger_block_t));
    return 1;
  }
  return 0;
}
/*---------------------------------------------------------------------------*/
void
flash_logger_save(flash_logger_block_t* data) 
{
  uint16_t addr = FLASH_LOGGER_CONF_START_ADDR;     /* seg. 2 */
  uint16_t end  = FLASH_LOGGER_CONF_START_ADDR + FLASH_LOGGER_CONF_SEG_SIZE;
  
  /* find the first empty block */
  while(addr < end) {
    if(*(uint16_t*)addr == 0xffff && *(uint16_t*)(addr + 2) == 0xffff) {
      break;
    }
    addr += sizeof(flash_logger_block_t);
  }
  if(addr >= end) {        /* no empty slot found? */
    /* erase the segment */
    flash_erase_info_segment((uint8_t*)FLASH_LOGGER_CONF_START_ADDR);
    if(!flash_erase_check((uint8_t*)FLASH_LOGGER_CONF_START_ADDR,
                          FLASH_LOGGER_CONF_SEG_SIZE)) {
      DEBUG_PRINT_MSG_NOW("ERROR: failed to erase flash segment!");
    }
    __delay_cycles(MCLK_SPEED / 2); /* wait 0.5s */
    addr = FLASH_LOGGER_CONF_START_ADDR;
  }
  /* update the the CRC */
  data->crc = crc16((uint8_t*)data + 2, 
                    sizeof(flash_logger_block_t) - 2, 0);
  /* save the data */
  flash_write((uint8_t*)data, (uint8_t*)addr, sizeof(flash_logger_block_t));
}
/*---------------------------------------------------------------------------*/