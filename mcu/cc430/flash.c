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

#include "platform.h"

#define FLASH_CTRL_BASE_REG     0x0140
/*---------------------------------------------------------------------------*/
uint16_t
flash_code_size(void)
{
  /* 16-bit word alignment */
  uint16_t *curr_addr = (uint16_t *)(FLASH_END - 1);   
  /* assumption: each unused word in the flash memory has the value 0xffff;
     flash programming starts at the smallest address */
  while(((uint16_t)curr_addr > FLASH_START) && (*curr_addr == 0xffff)) {
    curr_addr--;
  }
  return (uint16_t)curr_addr - FLASH_START + 2;
}
/*---------------------------------------------------------------------------*/
void
flash_erase_segment(uint8_t* addr)
{
  /* verify the address */
  if(addr < (uint8_t*)FLASH_START || addr > (uint8_t*)FLASH_END) {
    return;
  }
  /* unlock and set the erase bit */
  FCTL3 = FWPW;
  FCTL1 = FWPW + ERASE;
  *addr = 0;                    /* dummy write */
  while(FCTL3 & 1);             /* wait until done */
  FCTL1 = FWPW;
  FCTL3 = FWPW + LOCK;
}
/*---------------------------------------------------------------------------*/
void
flash_erase_info_segment(uint8_t* addr)
{
  /* verify the address */
  if(addr < (uint8_t*)INFO_START || addr > (uint8_t*)INFO_END) {
    return;
  }
  /* unlock and set the erase bit */
  FCTL3 = FWPW + LOCKA;         /* set LOCKA to change lock state of infomem */
  FCTL1 = FWPW + ERASE;
  *addr = 0;                    /* dummy write */
  while(FCTL3 & 1);             /* wait until done */
  FCTL1 = FWPW;
  FCTL3 = FWPW + LOCK + LOCKA;
}
/*---------------------------------------------------------------------------*/
void 
flash_erase_bank(void)  /* there's only one flash bank */
{
  FCTL3 = FWPW;                        /* unlock */
  while(FCTL3 & 1);                    /* wait for BUSY flag */
  FCTL1 = FWPW + MERAS;                /* set mass erase bit */
  *((uint16_t*)FLASH_START) = 0;       /* dummy write */
  while(FCTL3 & 1);                    /* wait for BUSY flag */
  FCTL1 = FWPW;                        /* clear mass erase bit */
  FCTL3 = FWPW + LOCK;                 /* lock the module */
}
/*---------------------------------------------------------------------------*/
uint8_t
flash_erase_check(uint8_t* start_addr, uint16_t num_bytes)
{
  if(start_addr == 0) {
    start_addr = (uint8_t*)FLASH_START;
    num_bytes  = FLASH_SIZE;
  } else if(num_bytes == 0) {
    num_bytes = FLASH_SEG_SIZE;
  }
  while(num_bytes) {
    if(*start_addr != 0xff) {
      return 0;
    }
    start_addr++;
    num_bytes--;
  }
  return 1;
}
/*---------------------------------------------------------------------------*/
void
flash_write(uint8_t* data, uint8_t* flash_addr, uint16_t num_bytes)
{
  FCTL3 = FWPW + LOCKA;
  FCTL1 = FWPW + WRT;
  while(num_bytes) {
    while(!(FCTL3 & 0x08));  /* bit 3 set? -> ready for next write operation */
    *flash_addr++ = *data++;
    num_bytes--;
  }
  while(!(FCTL3 & 0x08));
  FCTL1 = FWPW;
  FCTL3 = FWPW + LOCK + LOCKA;
}
/*---------------------------------------------------------------------------*/
void 
flash_erase_bank_from_sram(void)  /* there's only one flash bank */
{
  /* disable all interrupts for the following procedure */
  uint16_t interrupt_enabled = __get_interrupt_state() & GIE;
  __dint(); __nop();
    
  /* can only erase the whole bank if no code is being executed from the 
   * flash memory, hence the current code must be copied into the SRAM */
  uint16_t var = 0;
  void (*jump_to_sram)(void) = (void*)(&var - 256);
  memcpy(&var, flash_erase_bank, 256);
  jump_to_sram();
  
  if(interrupt_enabled) {
    __eint(); __nop();
  }
}
/*---------------------------------------------------------------------------*/