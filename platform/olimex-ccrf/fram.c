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

#include "platform.h"

/*---------------------------------------------------------------------------*/
/* hide the following settings/definitions from the rest of the code */

/* adjust the following figures according to the used FRAM chip */
#define FRAM_START          0x00000         /* first byte of the ext. FRAM */
#define FRAM_END            0x3ffff         /* last byte of the ext. FRAM */

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
#ifdef MUX_SEL
#define FRAM_RECONFIG_SPI \
  { \
    if(FRAM_CONF_SPI == USCI_A0) { \
      PIN_CLEAR(MUX_SEL); \
      spi_a0_reinit(FRAM_CONF_SPI); \
    } \
  }  
#else /* MUX_SEL */
#define FRAM_RECONFIG_SPI
#endif /* MUX_SEL */
/*---------------------------------------------------------------------------*/
static volatile uint8_t fram_sleeps = 0;
static uint16_t fram_num_alloc_blocks = 0;
static uint32_t fram_curr_offset = 0;
static volatile uint8_t fram_fill_value = 0;
static uint8_t fram_initialized = 0;
/*---------------------------------------------------------------------------*/
/**
 * @brief release the external memory chip, i.e. set the control/select pin
 *(FRAM_CTRL_PIN) high and disable the SPI
 */
void 
fram_release(void)
{
  if(!PIN_GET_INPUT_BIT(FRAM_CTRL_PIN)) { 
    SPI_WAIT_BUSY(FRAM_CONF_SPI); 
    SPI_CLEAR_RXBUF(FRAM_CONF_SPI); 
    PIN_SET(FRAM_CTRL_PIN); 
    SPI_DISABLE(FRAM_CONF_SPI); 
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
  if(PIN_GET_INPUT_BIT(FRAM_CTRL_PIN)) {
    /* make sure the module is in SPI mode */
    FRAM_RECONFIG_SPI;
    SPI_ENABLE(FRAM_CONF_SPI);     /* make sure the SPI is enabled */
    /* clock must be low before pulling the select line low */
    while((FRAM_CONF_SPI == USCI_B0) ? PIN_GET_INPUT_BIT(SPI_B0_CLK) :
        PIN_GET_INPUT_BIT(SPI_A0_CLK));
    PIN_CLEAR(FRAM_CTRL_PIN);     /* pull select line low */
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
fram_init(void)
{
  char dev_id[32];
  uint8_t c = 0;

  if(!fram_initialized) {
#ifdef MUX_SEL
    PIN_SET_AS_OUTPUT(MUX_SEL);
    PIN_CLEAR(MUX_SEL);
#endif
    PIN_SET(FRAM_CTRL_PIN);
    PIN_SET_AS_OUTPUT(FRAM_CTRL_PIN);
    if(FRAM_CONF_SPI == USCI_B0) {
      spi_b0_init(FRAM_CONF_SCLK_SPEED);
    } else {
      spi_a0_init(FRAM_CONF_SCLK_SPEED);
    }
#if FRAM_CONF_USE_DMA
    dma_config_spi(FRAM_CONF_SPI, fram_release);
#endif /* FRAM_CONF_USE_DMA */
    fram_num_alloc_blocks = 0;
    fram_curr_offset = 0;

    /* make sure that at least 1ms has passed since power-up before using 
     * the FRAM */
    __delay_cycles(MCLK_SPEED / 1000);      

    /* assume the FRAM is in sleep mode after power-up of the MCU */
    fram_sleeps = 1;
    /* read and verify the device ID (this implicitly checks if the external
       FRAM is available and working) */
    fram_get_id(dev_id, 0);
    while(c < 6) {
      if(0x7f != dev_id[c]) {
        DEBUG_PRINT_NOW("ERROR: fram_init failed (disconnected?)\n");
        return 0;
      }
      c++;
    }
    fram_initialized = 1;
  }
  return 1;
}
/*---------------------------------------------------------------------------*/
const char *
fram_get_id(char *const out_buffer, uint8_t formatted)
{
  static uint8_t count = 0;
  static uint8_t rcv_buffer[9];

  if(!fram_acquire()) {
    return 0;
  }

  /* send opcode */
  SPI_TRANSMIT_BYTE(FRAM_CONF_SPI, FRAM_OPCODE_RDID);
  SPI_RECEIVE_BYTE(FRAM_CONF_SPI, *rcv_buffer);

  /* receive the ID */
  count = 0;
  while(count < 9) {
    /* dummy write to generate the clock */
    SPI_TRANSMIT_BYTE(FRAM_CONF_SPI, 0x00); 
    SPI_RECEIVE_BYTE(FRAM_CONF_SPI, rcv_buffer[count]);
    count++;
  }
  fram_release();

  if(formatted) {
    sprintf(out_buffer,
            "%02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x",
            rcv_buffer[0],
            rcv_buffer[1],
            rcv_buffer[2],
            rcv_buffer[3],
            rcv_buffer[4],
            rcv_buffer[5],
            rcv_buffer[6],
            rcv_buffer[7],
            rcv_buffer[8]);
  } else {
    memcpy(out_buffer, rcv_buffer, 9);
  }
  return out_buffer;
}
/*---------------------------------------------------------------------------*/
uint8_t
fram_sleep(void)
{
  if(fram_sleeps) {
    return 1;
  }
  if(!fram_acquire()) {
    return 0;
  }
  SPI_TRANSMIT_BYTE(FRAM_CONF_SPI, FRAM_OPCODE_SLEEP);      /* send opcode */
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
  if(FRAM_END < start_addr) {
    return 0;
  }
  if(!fram_acquire()) {
    return 0;
  }

  /* send opcode */
  SPI_TRANSMIT_BYTE(FRAM_CONF_SPI, FRAM_OPCODE_READ);

  /* send the 18-bit address (as 3 bytes, the first 6 bits are not used) */
  SPI_TRANSMIT_BYTE(FRAM_CONF_SPI, (start_addr >> 16) & 0xff);
  SPI_TRANSMIT_BYTE(FRAM_CONF_SPI, (start_addr >> 8) & 0xff);
  SPI_TRANSMIT_BYTE(FRAM_CONF_SPI, start_addr & 0xff);
  SPI_WAIT_BUSY(FRAM_CONF_SPI);
  SPI_CLEAR_RXBUF(FRAM_CONF_SPI);

  /* receive data */
#if FRAM_CONF_USE_DMA
  /* set up a DMA transfer */
  dma_config_spi(FRAM_CONF_SPI, fram_release);
  dma_start(out_data, 0, num_bytes);
#else /* FRAM_CONF_USE_DMA */
 #if SPI_CONF_FAST_READ
  /* transmit 1 byte ahead to reach faster read speed!
   * note that 1 excess byte will be transmitted/read */
  SPI_TRANSMIT_BYTE(FRAM_CONF_SPI, 0x00);            
 #endif /* SPI_CONF_FAST_READ */
  while(num_bytes) {
    SPI_TRANSMIT_BYTE(FRAM_CONF_SPI, 0x00);     /* dummy write to generate the
                                                   clock */
    SPI_RECEIVE_BYTE(FRAM_CONF_SPI, *out_data); /* blocking call, waits until
                                                   RX buffer is not empty */
    out_data++;
    num_bytes--;
  }
  fram_release();
#endif /* FRAM_CONF_USE_DMA */

  return 1;
}
/*---------------------------------------------------------------------------*/
uint8_t
fram_write(uint32_t start_address, uint16_t num_bytes, const uint8_t *data)
{
  /* validate the start address */
  if(FRAM_END < start_address) {
    return 0;
  }
  if(!fram_acquire()) {
    return 0;
  }

  /* disable the write protection feature */
  SPI_TRANSMIT_BYTE(FRAM_CONF_SPI, FRAM_OPCODE_WREN);
  SPI_WAIT_BUSY(FRAM_CONF_SPI);
  PIN_SET(FRAM_CTRL_PIN);
  while((FRAM_CONF_SPI ==
        USCI_B0) ? PIN_GET_INPUT_BIT(SPI_B0_CLK) : PIN_GET_INPUT_BIT(
          SPI_A0_CLK)) ;
  /* wait for the clock signal to go low */
  PIN_CLEAR(FRAM_CTRL_PIN);

  /* send opcode */
  SPI_TRANSMIT_BYTE(FRAM_CONF_SPI, FRAM_OPCODE_WRITE);

  /* send the 18-bit address (as 3 bytes, the first 6 bits are not used) */
  SPI_TRANSMIT_BYTE(FRAM_CONF_SPI, (start_address >> 16) & 0xff);
  SPI_TRANSMIT_BYTE(FRAM_CONF_SPI, (start_address >> 8) & 0xff);
  SPI_TRANSMIT_BYTE(FRAM_CONF_SPI, start_address & 0xff);

#if FRAM_CONF_USE_DMA
  dma_config_spi(FRAM_CONF_SPI, fram_release);
  dma_start(0, data, num_bytes);
#else
  /* transmit data */
  while(num_bytes) {
    SPI_TRANSMIT_BYTE(FRAM_CONF_SPI, *(uint8_t *)data);
    data++;
    num_bytes--;
  }
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
  if(FRAM_END < start_address) {
    return 0;
  }
  if(!fram_acquire()) {
    return 0;
  }

  /* disable the write protection feature */
  SPI_TRANSMIT_BYTE(FRAM_CONF_SPI, FRAM_OPCODE_WREN);
  SPI_WAIT_BUSY(FRAM_CONF_SPI);
  PIN_SET(FRAM_CTRL_PIN);
  while((FRAM_CONF_SPI ==
        USCI_B0) ? PIN_GET_INPUT_BIT(SPI_B0_CLK) : PIN_GET_INPUT_BIT(
          SPI_A0_CLK)) ;
  /* wait for the clock signal to go low */
  PIN_CLEAR(FRAM_CTRL_PIN);

  /* send opcode */
  SPI_TRANSMIT_BYTE(FRAM_CONF_SPI, FRAM_OPCODE_WRITE);

  /* send the 18-bit address (as 3 bytes, the first 6 bits are not used) */
  SPI_TRANSMIT_BYTE(FRAM_CONF_SPI, (start_address >> 16) & 0xff);
  SPI_TRANSMIT_BYTE(FRAM_CONF_SPI, (start_address >> 8) & 0xff);
  SPI_TRANSMIT_BYTE(FRAM_CONF_SPI, start_address & 0xff);

#if FRAM_CONF_USE_DMA
  dma_set_dummy_byte_value(fill_value);
  dma_config_spi(FRAM_CONF_SPI, fram_release);
  dma_start(0, data, num_bytes);
#else
  /* transmit data */
  while(num_bytes) {
    SPI_TRANSMIT_BYTE(FRAM_CONF_SPI, fill_value);
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
  if(0 == size || ((fram_curr_offset + size) > FRAM_END)) {
    /* use printf to make sure this error message is printed out immediately */
    printf("ERROR: Memory allocation failed! (requested block size: %d B)",
           size);                                                                       
    return FRAM_ALLOC_ERROR;
  }
  DEBUG_PRINT_INFO("memory allocated (block: %d, length: %d, offset: %lu)",
                   fram_num_alloc_blocks,
                   size,
                   fram_curr_offset);
  fram_curr_offset += size;
  fram_num_alloc_blocks++;
  return addr;
}
/*---------------------------------------------------------------------------*/
