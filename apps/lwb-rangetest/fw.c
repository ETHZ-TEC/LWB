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
#include "platform.h"
#include "fw.h"

#if FW_CONF_ON

#ifndef XMEM_SIZE
#warning "XMEM_SIZE not defined! set to 0"
#define XMEM_SIZE       0
#endif /* XMEM_SIZE */

static fw_info_t fw = { 0 };
static uint8_t fw_block_info[FW_BLOCK_INFO_SIZE] = { 0 };
/*---------------------------------------------------------------------------*/
uint8_t
fw_init(void)
{
  /* quick error check */
  if((FW_BACKUP_ADDR_XMEM + FW_MAX_SIZE) > XMEM_SIZE) {
    return 1;   /* does not fit into memory!! */
  }
  /* verify integrity of the FW info block */
  if(xmem_read(FW_ADDR_XMEM, sizeof(fw_info_t), (uint8_t*)&fw)) {
    if(fw.crc != crc16((uint8_t*)&fw, sizeof(fw_info_t) - 2, 0)) {
      /* clear the data */
      memset(&fw, 0, sizeof(fw_info_t));
      fw.crc = crc16((uint8_t*)&fw, sizeof(fw_info_t) - 2, 0);
      xmem_write(FW_ADDR_XMEM, sizeof(fw_info_t), (uint8_t*)&fw);
      return 2; /* ERROR: integrity test failed, info struct reset */
    }
    /* FW OK! */
    /* load the block info data */
    xmem_read(FW_ADDR_XMEM + sizeof(fw_info_t), FW_BLOCK_INFO_SIZE,
              fw_block_info);
    return 0;
  }
  return 3;
}
/*---------------------------------------------------------------------------*/
uint8_t
fw_reset(const fw_info_t* new_fw_info)
{
  uint16_t crc = crc16((uint8_t*)new_fw_info, sizeof(fw_info_t) - 2, 0);
  if(FW_BLOCK_SIZE != new_fw_info->block_size || crc != new_fw_info->crc ||
     new_fw_info->len > FW_MAX_SIZE) {
    return 1;   /* invalid parameters */
  }  
  memcpy(&fw, new_fw_info, sizeof(fw_info_t));
  /* update status and CRC */
  fw.status = FW_STATUS_RECEIVING;
  fw.crc    = crc16((uint8_t*)&fw, sizeof(fw_info_t) - 2, 0);
  if(!xmem_write(FW_ADDR_XMEM, sizeof(fw_info_t), (uint8_t*)&fw)) {
    return 2;
  }
  /* erase the FW block info */
  memset(fw_block_info, 0, FW_BLOCK_INFO_SIZE);
  xmem_erase(FW_ADDR_XMEM + sizeof(fw_info_t), FW_BLOCK_INFO_SIZE);
  
  return 0;
}
/*---------------------------------------------------------------------------*/
void
fw_store_data(const data_t* data)
{
  /* validate address */
  if((data->pktnr * FW_BLOCK_SIZE) >= fw.len) {
    DEBUG_PRINT_ERROR("invalid FW data received!");
    return;
  }
  /* only store it if it is not yet there */
  uint8_t mask = (1 << (data->pktnr & 0x07));
  if((fw_block_info[data->pktnr / 8] & mask) == 0) {
    xmem_write(FW_DATA_START + data->pktnr * FW_BLOCK_SIZE, FW_BLOCK_SIZE,
               data->payload);
    /* mark this block as received */
    fw_block_info[data->pktnr / 8] |= mask;
    xmem_write(FW_ADDR_XMEM + sizeof(fw_info_t), 1,  
               &fw_block_info[data->pktnr / 8]);
  }
}
/*---------------------------------------------------------------------------*/
uint8_t
fw_validate(void)
{
  /* verify the received FW data */
  /* first, check the info struct */
  if(crc16((uint8_t*)&fw, sizeof(fw_info_t) - 2, 0) != fw.crc) {
    return 1;
  }
  uint16_t n_blocks = 0, i = 0;
  while(i < FW_BLOCK_INFO_SIZE) {
    if(fw_block_info[i] == 0xff) {
      n_blocks += 8;
      i++;
    } else {
      /* these are the last blocks */
      for(i = 0; i < 8; i++) {
        if(fw_block_info[i] & (1 << i)) {
          n_blocks++;
        }
      }
      break;  /* stop here */
    }                    
  }
  if(((fw.len + (fw.block_size - 1)) / fw.block_size) != 
     n_blocks) {
    return 2;
  }
  /* received all blocks! -> now check CRC */
  uint16_t crc = 0, len = 0;
  uint8_t buffer[128];
  while(len < fw.len) {
    uint16_t block_size = 128;
    if(!xmem_read(FW_DATA_START + len, block_size, buffer)) {
      return 3;
    }
    if((fw.len - len) < 128) {
      block_size = fw.len - len;
    }
    crc = crc16(buffer, block_size, crc);  
    len += 128;
  }
  if(crc != fw.data_crc) {
      return 4;
  } 
  return 0;     /* success! */
}
/*---------------------------------------------------------------------------*/
uint16_t
fw_get_missing_block_ids(uint16_t* out_data, uint8_t max_num_ids)
{
  uint16_t n_blocks = ((fw.len + (fw.block_size - 1)) / fw.block_size);
  /* loop through the block info data */
  uint16_t i, cnt = 0;  
  for(i = 0; i < n_blocks; i++) {
    /* is the bit for block i cleared? */
    if((fw_block_info[i / 8] & (1 << (i & 7))) == 0) {
      /* store this ID in the output buffer */
      *out_data++ = i;
      cnt++;
      max_num_ids--;
      if(!max_num_ids) {
        break;    /* no more space in the buffer for additional IDs */
      }
    }
  }
  return cnt;
}
/*---------------------------------------------------------------------------*/
uint8_t
fw_backup(void)
{
  /* start at address 0, copy the whole flash content into the external 
   * memory */
  if(!xmem_write(FW_BACKUP_ADDR_XMEM, FW_MAX_SIZE, (uint8_t*)FLASH_START)) {
    return 1;
  }  
  /* verify the content */
  uint32_t xmem_addr = FW_BACKUP_ADDR_XMEM;
  uint16_t flash_addr = CODE_START,
           read_bytes = 0;
  uint8_t buffer[256];  
  while(read_bytes < FW_MAX_SIZE) { 
    /* load a block of data into the RAM */
    if(!xmem_read(xmem_addr, 256, buffer)) {
      return 2;
    }
    uint16_t i;
    for(i = 0; i < 256; i++) {
      if(buffer[i] != *((uint8_t*)flash_addr + i)) {
        return 3;
      }
    }
    flash_addr += 256;
    xmem_addr += 256;
    read_bytes += 256;
  }  
  /* update the state */
  fw.status = FW_STATUS_VALIDATED;
  fw.crc    = crc16((uint8_t*)&fw, sizeof(fw_info_t) - 2, 0);
  xmem_write(FW_ADDR_XMEM, sizeof(fw_info_t), (uint8_t*)&fw);
  
  return 0;
}
/*---------------------------------------------------------------------------*/

#endif /* FW_CONF_ON */