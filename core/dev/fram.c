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

#include "contiki.h"

#if FRAM_CONF_ON

/*---------------------------------------------------------------------------*/
/* operation codes */
#define FRAM_OPCODE_WREN    0x06            /* disable write protection */
#define FRAM_OPCODE_WRDI    0x04            /* enable write protection */
#define FRAM_OPCODE_RDSR    0x05            /* read status register */
#define FRAM_OPCODE_WRSR    0x01            /* write status register */
#define FRAM_OPCODE_READ    0x03            /* read from FRAM */
#define FRAM_OPCODE_WRITE   0x02            /* write to FRAM */
#define FRAM_OPCODE_SLEEP   0xb9            /* enter sleep mode */
#define FRAM_OPCODE_RDID    0x9f            /* read ID */
/*---------------------------------------------------------------------------*/
static volatile uint8_t fram_sleeps = 0;
static uint16_t fram_num_alloc_blocks = 0;
static uint32_t fram_curr_offset = FRAM_CONF_ALLOC_START;
static volatile uint8_t fram_fill_value = 0;
static uint8_t fram_initialized = 0;
/*---------------------------------------------------------------------------*/
/**
 * @brief release the external memory chip, i.e. set the control/select pin
 *(FRAM_CONF_CTRL_PIN) high and disable the SPI
 */
void 
fram_release(void)
{
  if(!PIN_GET(FRAM_CONF_CTRL_PIN)) {
    spi_enable(FRAM_CONF_SPI, 0);     /* disable */
    spi_read_byte(FRAM_CONF_SPI, 0); 
    PIN_SET(FRAM_CONF_CTRL_PIN); 
  }
}
/*---------------------------------------------------------------------------*/
/**
 * @brief access the FRAM by pulling the control pin low
 * @return 1 if successful, 0 otherwise
 * @note  This macro returns zero in case the control pin is already low (not
 *in idle state).
 */
uint8_t
fram_acquire(void) 
{
  if(PIN_GET(FRAM_CONF_CTRL_PIN)) {
    spi_enable(FRAM_CONF_SPI, 1);
    PIN_CLR(FRAM_CONF_CTRL_PIN);     /* pull select line low */
    if(fram_sleeps) {     /* in LPM? -> wait ~0.5ms */
      __delay_cycles(MCLK_SPEED / 2000);
      fram_sleeps = 0;
    } 
    return 1;
  }
  return 0;
}
/*---------------------------------------------------------------------------*/
uint8_t
fram_get_id(uint8_t* out_buffer)
{
  if(!fram_acquire()) {
    return 0;
  }
  /* send opcode */
  spi_write_byte(FRAM_CONF_SPI, FRAM_OPCODE_RDID);
  spi_read_byte(FRAM_CONF_SPI, 1);
  /* receive the ID */
  spi_read(FRAM_CONF_SPI, out_buffer, 9);
  fram_release();
  return 1;
}
/*---------------------------------------------------------------------------*/
uint8_t
fram_init(void)
{
  uint8_t dev_id[10];
  uint8_t c = 0;

  if(!fram_initialized) {    
    PIN_SET(FRAM_CONF_CTRL_PIN);
    PIN_CFG_OUT(FRAM_CONF_CTRL_PIN);
    if(!spi_init(FRAM_CONF_SPI, FRAM_CONF_SCLK_SPEED)) {
      DEBUG_PRINT_MSG_NOW("ERROR: spi init failed");
      return 0;
    }
    fram_num_alloc_blocks = 0;
    fram_curr_offset = FRAM_CONF_ALLOC_START;

    /* make sure that at least 1ms has passed since power-up before using 
     * the FRAM */
    __delay_cycles(MCLK_SPEED / 1000);      

    /* assume the FRAM is in sleep mode after power-up of the MCU */
    fram_sleeps = 1;
    /* read and verify the device ID (this implicitly checks if the external
       FRAM is available and working) */
    if(!fram_get_id(dev_id)) {
        DEBUG_PRINT_MSG_NOW("ERROR: fram_get_id failed");
        return 0;
    }
    while(c < 6) {
      if(0x7f != dev_id[c]) {
        DEBUG_PRINT_MSG_NOW("ERROR: FRAM init failed (disconnected?)");
        return 0;
      }
      c++;
    }
    DEBUG_PRINT_MSG_NOW("FRAM initialized");
    fram_initialized = 1;
  }
  return 1;
}
/*---------------------------------------------------------------------------*/
uint8_t
fram_sleep(void)
{
  if(fram_sleeps) {
    return 2;
  }
  if(!fram_acquire()) {
    return 0;
  }
  spi_write_byte(FRAM_CONF_SPI, FRAM_OPCODE_SLEEP);      /* send opcode */
  fram_release();
  fram_sleeps = 1;
  return 1;
}
/*---------------------------------------------------------------------------*/
uint8_t
fram_wakeup(void)
{
  if(!fram_sleeps) {
    return 1;
  }
  if(!fram_acquire()) {
    return 0;
  }
  fram_release();
  fram_sleeps = 0;
  return 1;
}
/*---------------------------------------------------------------------------*/
uint8_t
fram_read(uint32_t start_addr, uint16_t num_bytes, uint8_t *out_data)
{
  /* validate the start address */
  if((FRAM_CONF_SIZE + FRAM_CONF_START) <= start_addr) {
    return 0;
  }
  if(!fram_acquire()) {
    return 0;
  }
  /* send opcode */
  spi_write_byte(FRAM_CONF_SPI, FRAM_OPCODE_READ);
  /* send the 18-bit address (as 3 bytes, the first 6 bits are not used) */
  spi_write_byte(FRAM_CONF_SPI, (start_addr >> 16) & 0xff);
  spi_write_byte(FRAM_CONF_SPI, (start_addr >> 8) & 0xff);
  spi_write_byte(FRAM_CONF_SPI, start_addr & 0xff);
  spi_wait(FRAM_CONF_SPI);
  spi_read_byte(FRAM_CONF_SPI, 0);      /* clear RX buffer */
  /* receive data */
#if FRAM_CONF_USE_DMA
  /* set up a DMA transfer */
  dma_config_spi(FRAM_CONF_SPI, fram_release);
  dma_start(out_data, 0, num_bytes);
#else /* FRAM_CONF_USE_DMA */
  spi_read(FRAM_CONF_SPI, out_data, num_bytes);
  fram_release();
#endif /* FRAM_CONF_USE_DMA */
  return 1;
}
/*---------------------------------------------------------------------------*/
uint8_t
fram_write(uint32_t start_address, uint16_t num_bytes, const uint8_t *data)
{
  /* validate the start address */
  if((FRAM_CONF_SIZE + FRAM_CONF_START) <= start_address) {
    return 0;
  }
  if(!fram_acquire()) {
    return 0;
  }
  /* disable the write protection feature */
  spi_write_byte(FRAM_CONF_SPI, FRAM_OPCODE_WREN);
  fram_release();  
  if(!fram_acquire()) {
    return 0;
  }
  /* send opcode */
  spi_write_byte(FRAM_CONF_SPI, FRAM_OPCODE_WRITE);
  /* send the 18-bit address (as 3 bytes, the first 6 bits are not used) */
  spi_write_byte(FRAM_CONF_SPI, (start_address >> 16) & 0xff);
  spi_write_byte(FRAM_CONF_SPI, (start_address >> 8) & 0xff);
  spi_write_byte(FRAM_CONF_SPI, start_address & 0xff);
#if FRAM_CONF_USE_DMA
  dma_config_spi(FRAM_CONF_SPI, fram_release);
  dma_start(0, data, num_bytes);
#else  
  spi_write(FRAM_CONF_SPI, data, num_bytes);
  fram_release();
  /* Note: the write protection feature is now enabled again! */
#endif /* FRAM_CONF_USE_DMA */
  return 1;
}
/*---------------------------------------------------------------------------*/
uint8_t
fram_fill(uint32_t start_address, uint16_t num_bytes, const uint8_t fill_value)
{
  /* validate the start address */
  if((FRAM_CONF_SIZE + FRAM_CONF_START) <= start_address) {
    return 0;
  }
  if(!fram_acquire()) {
    return 0;
  }
  /* disable the write protection feature */
  spi_write_byte(FRAM_CONF_SPI, FRAM_OPCODE_WREN);
  fram_release();
  
  if(!fram_acquire()) {
    return 0;
  }
  /* send opcode */
  spi_write_byte(FRAM_CONF_SPI, FRAM_OPCODE_WRITE);
  /* send the 18-bit address (as 3 bytes, the first 6 bits are not used) */
  spi_write_byte(FRAM_CONF_SPI, (start_address >> 16) & 0xff);
  spi_write_byte(FRAM_CONF_SPI, (start_address >> 8) & 0xff);
  spi_write_byte(FRAM_CONF_SPI, start_address & 0xff);
#if FRAM_CONF_USE_DMA
  dma_set_dummy_byte_value(fill_value);
  dma_config_spi(FRAM_CONF_SPI, fram_release);
  dma_start(0, data, num_bytes);
#else
  /* transmit data */
  while(num_bytes) {
    spi_write_byte(FRAM_CONF_SPI, fill_value);
    num_bytes--;
  }
  fram_release();
  /* Note: the write protection feature is now enabled again! */
#endif /* FRAM_CONF_USE_DMA */
  return 1;
}
/*---------------------------------------------------------------------------*/
uint32_t
fram_alloc(uint16_t size)
{    
  uint32_t addr = fram_curr_offset;     /* word address */
  if(0 == size || 
    ((fram_curr_offset + size) >= (FRAM_CONF_ALLOC_SIZE + FRAM_CONF_START))) {
    /* fatal error: do not proceed */
    DEBUG_PRINT_FATAL("ERROR: Memory allocation failed! "
                      "(requested block size: %db)", size);
    return FRAM_ALLOC_ERROR;
  }
  printf(" memory allocated (block: %d, len: %db, ofs: %lu)\r\n",
         fram_num_alloc_blocks, size, fram_curr_offset);
  fram_curr_offset += size;
  fram_num_alloc_blocks++;
  return addr;
}
/*---------------------------------------------------------------------------*/
inline uint8_t xmem_init(void)
{
  return fram_init();
}
/*---------------------------------------------------------------------------*/
inline uint8_t xmem_sleep(void)
{
  return fram_sleep();
}
/*---------------------------------------------------------------------------*/
inline uint8_t xmem_wakeup(void)
{
  return fram_wakeup();
}
/*---------------------------------------------------------------------------*/
inline uint32_t xmem_alloc(uint32_t size)
{
  return fram_alloc(size);
}
/*---------------------------------------------------------------------------*/
inline uint8_t xmem_read(uint32_t start_address, uint16_t num_bytes, 
                         uint8_t *out_data) 
{ 
  return fram_read(start_address, num_bytes, out_data); 
}
/*---------------------------------------------------------------------------*/
inline uint8_t xmem_write(uint32_t start_address, uint16_t num_bytes,
                          const uint8_t *data) 
{
  return fram_write(start_address, num_bytes, data);
}
/*---------------------------------------------------------------------------*/
inline uint8_t xmem_erase(uint32_t start_address, uint16_t num_bytes) 
{
  return fram_fill(start_address, num_bytes, 0);
}
/*---------------------------------------------------------------------------*/

#endif

/*---------------------------------------------------------------------------*/
uint16_t 
crc16(const uint8_t* data, uint8_t num_bytes, uint16_t init_value) 
{
  uint16_t crc  = init_value,
           mask = 0xa001;
  while(num_bytes) {
    uint8_t ch = *data;
    int8_t bit = 0;
    while(bit < 8) {
      if((crc & 1) ^ (ch & 1)) {
        crc = (crc >> 1) ^ mask;
      } else {
        crc >>= 1;
      }
      ch >>= 1; 
      bit += 1;
    }
    data++;
    num_bytes--;
  }
  return crc;
}
/*---------------------------------------------------------------------------*/
