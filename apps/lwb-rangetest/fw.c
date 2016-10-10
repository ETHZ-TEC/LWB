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
 * TODO
 * - instead of copying the update routine into SRAM, one could use a
 *   dedicated flash memory segment for the update routine
 * - to make it more robust, one could add a bootloader which checks the
 *   integritiy of the memory before the program execution starts
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
    DEBUG_PRINT_MSG_NOW("FW found (vs: %u, len: %u, status: %u)", 
                        fw.version, fw.len, fw.status);
    return 0;
  }
  return 3;
}
/*---------------------------------------------------------------------------*/
uint8_t
fw_reset(const fw_info_t* new_fw_info)
{
  uint16_t crc = crc16((uint8_t*)new_fw_info, sizeof(fw_info_t) - 2, 0);
  if(FW_MIN_BLOCK_SIZE > new_fw_info->block_size || crc != new_fw_info->crc ||
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
  uint32_t ofs = (data->pktnr * fw.block_size);
  if(ofs >= fw.len) {
    DEBUG_PRINT_ERROR("invalid FW data received!");
    return;
  }
  /* only store it if it is not yet there */
  uint8_t mask = (1 << (data->pktnr & 0x07));
  uint16_t idx = data->pktnr / 8;
  if((fw_block_info[idx] & mask) == 0) {
    xmem_write(FW_DATA_START + ofs, fw.block_size, data->payload);
    /* mark this block as received */
    fw_block_info[idx] |= mask;
    xmem_write(FW_ADDR_XMEM + sizeof(fw_info_t) + idx, 1,
               &fw_block_info[idx]);
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
      uint8_t j = 1;
      while(fw_block_info[i] & j) {
        n_blocks++;
        j = j << 1;
      }
      break;  /* stop here */
    }
  }
  if(((fw.len + (fw.block_size - 1)) / fw.block_size) != n_blocks) {
    return 2;
  }
  /* received all blocks! -> now check CRC */
  uint16_t crc = 0, ofs = 0;
  uint8_t buffer[128];
  uint16_t block_size;
  while(ofs < fw.len) {
    block_size = 128;
    if((fw.len - ofs) < 128) {
      block_size = fw.len - ofs;        /* remaining bytes */
    }
    if(!xmem_read(FW_DATA_START + ofs, block_size, buffer)) {
      return 3;
    }
    crc = crc16(buffer, block_size, crc);  
    ofs += block_size;
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
void
fw_update_routine(uint16_t fw_size)
{
  /* the following code must execute from SRAM in order to work!
   * any called subroutines in this function must also be in SRAM or inlined
   * here
   * the heap may not be accessed at this point and interrupts are disabled!
   * note that this function is tailored to a specific architecture / MCU! */
  
  /* Note: if this routine resides in a memory segment outside of the region
   * that needs to be programmed, then it is not necessary to move this 
   * routine into SRAM (use segment erase instead of mass erase */
  
  /* Note: Another way to execute from RAM on a MSP430 is to use the following
   * pragma, which will tell the compiler/linker to put the function into the
   * .data section:
   * __attribute__((section(".data"), noinline))
   * void sram_function(void) { ... }  */
  
#define FW_UPDATE_SIMULATE      0

  PIN_SET(LED_0);       /* to indicate the start */
  
  /* configure peripherals and GPIOs */
  UCA0CTL1 |= UCSWRST;
  UCA0BRW   = 1;
  /* config SPI in 3-wire master mode */
  UCA0CTL0  = (UCMSB + UCMST + UCSYNC + UCCKPH);
  PIN_CLR(MUX_SEL_PIN);
  UCA0CTL1 &= ~UCSWRST;
  
#if !FW_UPDATE_SIMULATE && FW_UPDATE_MASS_ERASE
  /* mass erase */
  FCTL3 = FWPW;                        /* unlock */
  while(FCTL3 & 1);                    /* wait for BUSY flag */
  FCTL1 = FWPW + MERAS;                /* set mass erase bit */
  *((uint16_t*)FLASH_START) = 0;       /* dummy write */
  while(FCTL3 & 1);                    /* wait for BUSY flag */
  FCTL1 = FWPW;                        /* clear mass erase bit */
  FCTL3 = FWPW + LOCK;                 /* lock the module */
#endif /* FW_UPDATE_SIMULATE */
  
  /* program data */
  uint32_t xmem_addr = FW_DATA_START;
  uint16_t flash_addr = FLASH_START;
  uint16_t remaining_bytes = fw_size - INT_TABLE_SIZE;
  uint8_t  segment[FLASH_SEG_SIZE];
  while(remaining_bytes) {
    uint16_t len = FLASH_SEG_SIZE;
    if(remaining_bytes > FLASH_SEG_SIZE) {
      remaining_bytes -= FLASH_SEG_SIZE;
    } else {
      len = remaining_bytes;
      remaining_bytes = 0;
    }
    
#if FW_UPDATE_SIMULATE
    /* reconfigure */
    while(UCA0STAT & UCBUSY);
    UCA0CTL1 |= UCSWRST;
    UCA0BRW   = 1;
    /* config SPI in 3-wire master mode */
    UCA0CTL0  = (UCMSB + UCMST + UCSYNC + UCCKPH);
    PIN_CLR(MUX_SEL_PIN);
    UCA0CTL1 &= ~UCSWRST;
    __delay_cycles(MCLK_SPEED / 1000);
#else
  #if !FW_UPDATE_MASS_ERASE
    /* erase this segment */
    FCTL3 = FWPW;
    FCTL1 = FWPW + ERASE;
    *(uint8_t*)flash_addr = 0;    /* dummy write */
    while(FCTL3 & 1);             /* wait until done */
    FCTL1 = FWPW;
    FCTL3 = FWPW + LOCK;
  #endif /* FW_UPDATE_MASS_ERASE */
#endif /* FW_UPDATE_SIMULATE */
  
    /* read from external memory */
    PIN_CLR(FRAM_CONF_CTRL_PIN);     
    __delay_cycles(MCLK_SPEED / 2000);
    /* send opcode and address */
    while(!(UCA0IFG & UCTXIFG));
    UCA0TXBUF = 0x03;   /* read command */
    while(!(UCA0IFG & UCTXIFG));
    UCA0TXBUF = (xmem_addr >> 16) & 0xff;
    while(!(UCA0IFG & UCTXIFG));
    UCA0TXBUF = (xmem_addr >> 8) & 0xff;
    while(!(UCA0IFG & UCTXIFG));
    UCA0TXBUF = xmem_addr & 0xff;
    while(UCA0STAT & UCBUSY);
    (void)UCA0RXBUF;      /* clear RX buffer (0x05cc) */
    /* receive data */
    uint16_t n = 0;
    while(n < len) {
      while(!(UCA0IFG & UCTXIFG));
      UCA0TXBUF = 0x00;   /* dummy write */
      while(!(UCA0IFG & UCRXIFG));
      segment[n++] = UCA0RXBUF;
    }
    PIN_SET(FRAM_CONF_CTRL_PIN);
    
#if !FW_UPDATE_SIMULATE
    /* write to flash memory */
    FCTL3 = FWPW + LOCKA;
    FCTL1 = FWPW + WRT;
    n = 0;
    while(n < len) {
      while(!(FCTL3 & 0x08));
      *(uint8_t*)(flash_addr + n) = segment[n];
      n++;
    }
    while(!(FCTL3 & 0x08));
    FCTL1 = FWPW;
    FCTL3 = FWPW + LOCK + LOCKA;
    
#else    
    char buffer[128];
    while(UCA0STAT & UCBUSY);
    UCA0CTL1 |= UCSWRST;
    UCA0BRW   = (uint16_t)((SMCLK_SPEED * 100) / UART_CONF_BAUDRATE / 100);
    UCA0CTL0  = 0;    
    UCA0CTL1 &= ~UCSWRST;
    PIN_SET(MUX_SEL_PIN);
    snprintf(buffer, 128, "writing %u bytes at 0x%04x...\r\n", len, flash_addr);
    printf(buffer);
#endif /* FW_UPDATE_SIMULATE */

    xmem_addr += len;
    if(remaining_bytes == 0 && flash_addr != INT_TABLE_START) {
      flash_addr = INT_TABLE_START;
      remaining_bytes = INT_TABLE_SIZE;
    } else {  
      flash_addr += len;
    }
  }
  
  /* force a reboot */
  WDTCTL = 0;
}
/*---------------------------------------------------------------------------*/
void
fw_update(void)
{
  if(fw.status != FW_STATUS_VALIDATED) {
    return;     /* abort */
  }
  /* do as many error checks a possible at this point! */

  /* update the state */
  fw.status = FW_STATUS_UPDATING;
  fw.crc    = crc16((uint8_t*)&fw, sizeof(fw_info_t) - 2, 0);
  xmem_write(FW_ADDR_XMEM, sizeof(fw_info_t), (uint8_t*)&fw);
    
  /* disable all interrupts for the following procedure */
  __dint(); __nop();
  
  /* copy the update routine into SRAM (this will corrupt the heap and 
   * make further program execution impossible without a reset!) */
  uint16_t fw_len = fw.len;
  
  /* get this info from the .dis file or just set it sufficiently high,
   * at most 0x0400 (1kB) */
#define FW_UPD_FUNC_SIZE  0x0200        
  
  void (*upd_now)(uint16_t) = (void*)((uint8_t*)SRAM_START);
  /* copy function into the SRAM (overwrite the heap) */
  memcpy((uint8_t*)SRAM_START, (uint8_t*)fw_update_routine, FW_UPD_FUNC_SIZE);
  /* jump to the function */
  upd_now(fw_len);
}
/*---------------------------------------------------------------------------*/

#endif /* FW_CONF_ON */