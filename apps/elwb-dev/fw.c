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

#include "main.h"

/*
 * FW updater (to update the own firmware, both source and host node)
 * Does not act as a FW master!
 * 
 * Notes:
 * - firmware updater requires an external FRAM chip
 * - FW file size for the CC430 is 32kB (FW_FILE_SIZE)
 * - the only function that may be called is fw_process_msg()
 * - sends only 2 types of messages: FW_TYPE_DATA_REQ and FW_TYPE_READY
 */

#if FW_UPDATE_CONF_ON

#define FW_FILE_SIZE          32768   /* don't change! (power of 2) */
#define FW_FILE_BLOCK_CNT     ((FW_FILE_SIZE + DPP_FW_BLOCK_SIZE - 1) / \
                               DPP_FW_BLOCK_SIZE)

#if FW_UPDATE_CONF_ON && !FRAM_CONF_ON
#error "FW_UPDATE_CONF_ON requires FRAM!"
#endif

/*---------------------------------------------------------------------------*/

typedef enum
{
  FW_STATE_INVALID,
  FW_STATE_INIT,        /* initialized */
  FW_STATE_RECEIVING,   /* in the process of receiving the new fw data */
  FW_STATE_READY,       /* fw is ready for the update */
} fw_state_t;

typedef struct
{
  uint16_t    version;    /* FW version */
  uint16_t    block_cnt;  /* received blocks */
  uint32_t    data_crc;   /* 32bit CRC over the complete FW data */
  uint32_t    fram_addr;
  fw_state_t  state;
  uint8_t     blocks[(FW_FILE_BLOCK_CNT + 7) / 8];  /* 64 bytes */
} fw_info_t;

typedef struct
{
  dpp_fw_type_t type : 8;
  uint8_t       component_id;
  uint16_t      version;
} dpp_fw_hdr_t;

/*---------------------------------------------------------------------------*/

static fw_info_t fwinfo;                          /* struct size is 75 bytes */

/*---------------------------------------------------------------------------*/

/* helper functions */
uint32_t crc32(uint8_t* data, uint16_t len);
uint32_t crc32_w_seed(uint8_t* data, uint16_t len, uint32_t seed);

/* prototypes for local functions, do NOT call any of these directly */
retval_t fw_store_data(const dpp_fw_t* fwpkt);
retval_t fw_verify(const dpp_fw_t* fwpkt);
retval_t fw_update(void);
void fw_request_data(void);
void fw_copy(void);

/*---------------------------------------------------------------------------*/
retval_t
fw_init(void)
{
  /* clear the info struct */
  memset(&fwinfo, 0, sizeof(fw_info_t));
  
  /* allocate memory for the FW image file */
  uint32_t addr = fram_alloc(32768);
  /* block start address must be 0x0, otherwise adjust fw_copy()! */
  if(addr != 0x0) { 
    return FAILED;
  }
  fwinfo.fram_addr = addr;
  fwinfo.state = FW_STATE_INIT;
  return SUCCESS;
}
/*---------------------------------------------------------------------------*/
/* process a received FW message */
retval_t
fw_process_msg(dpp_message_t* msg)
{
  /* assume CRC of the message has been verified at this point */
  
  /* check the state */
  if(fwinfo.state == FW_STATE_INVALID) {
    return FAILED;                     /* fw_init() not yet called! */
  }
  
  /* only process further if length is ok and component ID matches! */
  if(msg->header.type != DPP_MSG_TYPE_FW ||
     msg->header.payload_len < DPP_FW_HDR_LEN ||
     msg->firmware.component_id != DPP_COMPONENT_ID_CC430) {
    return FAILED;
  }  
  /* inspect the message type */
  switch (msg->firmware.type)
  {
  case DPP_FW_TYPE_DATA:
    /* check the block size */
    if(msg->header.payload_len != (6 + DPP_FW_BLOCK_SIZE)) {
      return FAILED;
    }
    DEBUG_PRINT_INFO("processing FW block %u", msg->firmware.data.ofs);
    return fw_store_data(&msg->firmware);

  case DPP_FW_TYPE_CHECK:
    DEBUG_PRINT_INFO("checking FW file");
    fw_verify(&msg->firmware);
    return SUCCESS;   /* always return success for this msg type */

  case DPP_FW_TYPE_UPDATE:
    DEBUG_PRINT_MSG_NOW("FW update in progress...");
    return fw_update();
        
  default:
    DEBUG_PRINT_WARNING("unknown FW msg type");
    return FAILED;
  }
  return SUCCESS;
}
/*---------------------------------------------------------------------------*/
/* pass the function a FW data block of length DPP_FW_BLOCK_SIZE */
retval_t 
fw_store_data(const dpp_fw_t* fwpkt)
{
  /* check block offset */
  if(fwpkt->data.ofs > FW_FILE_BLOCK_CNT) {
    return FAILED;
  }
  /* first packet of this fw version? */
  if(FW_STATE_INIT == fwinfo.state || fwpkt->version != fwinfo.version) {
    /* initialize the meta data (FW info) */
    memset(&fwinfo, 0, sizeof(fw_info_t));
    fwinfo.state     = FW_STATE_RECEIVING;
    fwinfo.version   = fwpkt->version;
  }
  if(fwinfo.state == FW_STATE_READY) {
    return SUCCESS; /* no need to continue, we already have all data */
  }
  /* check whether we already have this block */
  uint16_t blkid   = fwpkt->data.ofs;                    /* offset = block ID */
  uint16_t bitmask = (1 << (blkid & 0x7));
  if(!(fwinfo.blocks[blkid >> 3] & bitmask)) {
    /* store the received data */
    if(!fram_write(fwinfo.fram_addr + 
                   (uint32_t)blkid * DPP_FW_BLOCK_SIZE, 
                   DPP_FW_BLOCK_SIZE, fwpkt->data.data)) {
      return FAILED;
    }
    /* mark the block as 'existing' */
    fwinfo.blocks[blkid >> 3] |= bitmask;
    fwinfo.block_cnt++;
  }
  return SUCCESS;
}
/*---------------------------------------------------------------------------*/
retval_t 
fw_verify(const dpp_fw_t* fwpkt)
{
  /* verify the FW data */
  if(fwinfo.state != FW_STATE_READY) {
    if(fwpkt->info.len != FW_FILE_SIZE) {   /* must be exactly 32kB */
      return FAILED;
    }    
    /* fill in the missing info */
    fwinfo.data_crc = fwpkt->info.crc;
    
    /* first, check whether all blocks are in the memory */
    if(fwinfo.block_cnt < FW_FILE_BLOCK_CNT) {
      fw_request_data();
      EVENT_INFO(EVENT_CC430_FW_PROGRESS, ((uint32_t)fwinfo.block_cnt * 100 / FW_FILE_BLOCK_CNT));      
      return FAILED;
    }
    /* data is in place, verify checksum */
    uint32_t crc = 0;
    uint32_t addr = fwinfo.fram_addr;
    uint16_t num_blocks = FW_FILE_BLOCK_CNT;
    uint8_t read_buffer[DPP_FW_BLOCK_SIZE];
    while(num_blocks) {
      fram_read(addr, DPP_FW_BLOCK_SIZE, read_buffer);
      crc = crc32_w_seed(read_buffer, DPP_FW_BLOCK_SIZE, crc);
      addr += DPP_FW_BLOCK_SIZE;
      num_blocks--;
    }
    if(fwinfo.data_crc != crc) {
      DEBUG_PRINT_ERROR("invalid FW file CRC");
      fwinfo.state = FW_STATE_INIT;  /* drop all FW info and data */
      return FAILED;
    }
    /* checksum is ok */
    fwinfo.state = FW_STATE_READY;
    DEBUG_PRINT_INFO("FW file verified");
  }
  /* notify */
  dpp_fw_hdr_t notification;
  notification.type         = DPP_FW_TYPE_READY;
  notification.component_id = DPP_COMPONENT_ID_CC430;
  notification.version      = fwinfo.version;
  send_msg(0, DPP_MSG_TYPE_FW, (uint8_t*)&notification, 4, IS_HOST);

  return SUCCESS;
}
/*---------------------------------------------------------------------------*/
/* request the missing FW data blocks (pass the total # blocks) */
void
fw_request_data(void)
{
  /* NOTE uses the global variable msg_tx instead to save stack space */
  //dpp_fw_t request;
  uint16_t blkid, 
           cnt = 0;
  for(blkid = 0; blkid < FW_FILE_BLOCK_CNT; blkid++) {
    /* is the bit for this block cleared? */
    if((fwinfo.blocks[blkid >> 3] & (1 << (blkid & 7))) == 0) {
      /* store this ID in the output buffer */
      msg_tx.firmware.req.block_ids[cnt++] = blkid;
      if(cnt >= ((DPP_MSG_PAYLOAD_LEN - 2 - DPP_FW_HDR_LEN) / 2)) {
        break;    /* no more space in the buffer for additional IDs */
      }
    }
  }
  if(cnt == 0) {
    return;       /* no need to send a block request */
  }
  /* send request */
  msg_tx.firmware.type         = DPP_FW_TYPE_DATAREQ;
  msg_tx.firmware.component_id = DPP_COMPONENT_ID_CC430;
  msg_tx.firmware.version      = fwinfo.version;
  msg_tx.firmware.req.num      = cnt;
  send_msg(0, DPP_MSG_TYPE_FW, 0, 6 + cnt * 2,
           IS_HOST);
  
  DEBUG_PRINT_INFO("%u FW data blocks requested", cnt);
}
/*---------------------------------------------------------------------------*/
retval_t
fw_update(void)
{
  /* initiate the firmware update (for host nodes: node's own firmware) */
  if (FW_STATE_READY != fwinfo.state) {
    return FAILED;
  }
  fram_wakeup();

  /* this will overwrite the heap! */
  uint16_t var = 0;
  /* make sure it doesn't collide with the stack */
  void (*execute_from_sram)(void) = (void*)(&var - 512);
  memcpy(execute_from_sram, (uint8_t*)fw_copy, 0xb3ee - 0xb2ca);
  execute_from_sram();
  
  return SUCCESS;
} 
/*---------------------------------------------------------------------------*/
uint32_t 
crc32(uint8_t* data, uint16_t len)
{
  uint32_t crc = 0xffffffff;
  while(len) {
    crc ^= (*data);
    uint16_t i = 8;
    while(i) {
      uint32_t mask = -(crc & 1);
      crc  = (crc >> 1) ^ (0xedb88320 & mask);
      i--;
    }
    data++;
    len--;
  }
  return ~crc;
}
/*---------------------------------------------------------------------------*/
uint32_t 
crc32_w_seed(uint8_t* data, uint16_t len, uint32_t seed)
{
  uint32_t crc = ~seed;
  while(len) {
    crc ^= (*data);
    uint16_t i = 8;
    while(i) {
      uint32_t mask = -(crc & 1);
      crc  = (crc >> 1) ^ (0xedb88320 & mask);
      i--;
    }
    data++;
    len--;
  }
  return ~crc;
}
/*---------------------------------------------------------------------------*/
/* NOTE a valid firmware file (32k) is required at FRAM address 0x0, 
 * no further checks are performed at this stage! */
void
fw_copy(void)
{
  /* make sure this function does not contain any calls, inline everything!
   * this function fits into 2 info memory pages (256 bytes) */
#define SPI_B0_SOMI             PORT1, PIN2
#define SPI_B0_SIMO             PORT1, PIN3
#define SPI_B0_CLK              PORT1, PIN4
#define SPI_INIT \
    P1SEL |= BIT2 | BIT3 | BIT4; \
    UCB0CTL1 = (UCSWRST | UCSSEL__SMCLK); \
    UCB0CTL0 = (UCMSB | UCMST | UCSYNC | UCCKPH); \
    UCB0BRW = 1;  /* use clock 1:1 */ \
    UCB0IFG &= ~(UCRXIFG + UCTXIFG);     
#define SPI_ENABLE \
    UCB0CTL1 &= ~UCSWRST; \
    while(PIN_GET(SPI_B0_CLK))
#define SPI_WRITE_BYTE(b) \
    while(!(UCB0IFG & UCTXIFG)); \
    UCB0TXBUF = (b)
#define SPI_READ_BYTE(b) \
    while(!(UCB0IFG & UCTXIFG)); \
    UCB0TXBUF = 0x0; \
    while(!(UCB0IFG & UCRXIFG)); \
    (b) = UCB0RXBUF
#define SPI_CLR_RXBUF \
    while(UCB0STAT & UCBUSY); \
    (void)UCB0RXBUF
    
  //config_t cfg_backup = cfg;

  WDTCTL = WDTPW + WDTHOLD;            /* stop watchdog */
  __dint(); __nop();                   /* disable all interrupts */
  SPI_INIT;                            /* init SPI */
  
  /* init FRAM */
  PIN_SET(FRAM_CONF_CTRL_PIN);
  PIN_CFG_OUT(FRAM_CONF_CTRL_PIN);
 
  /* mass erase program flash memory */
  FCTL3 = FWPW;                        /* unlock */
  while(FCTL3 & 1);                    /* wait for BUSY flag */
  FCTL1 = FWPW + MERAS;                /* set mass erase bit (this will also
                                          erase the info memory!) */
  *((uint16_t*)FLASH_START) = 0;       /* dummy write */
  while(FCTL3 & 1);                    /* wait for BUSY flag */
  FCTL1 = FWPW;                        /* clear mass erase bit */
  FCTL3 = FWPW + LOCK;                 /* lock the module */
 
  FCTL3 = FWPW + LOCKA;                /* enable write for all flash seg. */
  FCTL1 = FWPW + WRT;
  /* keep the flash unlocked */
  
  /* read the new firmware from the external FRAM, block after block */
  SPI_ENABLE;
  PIN_CLR(FRAM_CONF_CTRL_PIN);         /* pull select line low */
  __delay_cycles(MCLK_SPEED / 2000);   /* just in case the FRAM was asleep */
  
  SPI_WRITE_BYTE(0x03);                /* send opcode READ */
  /* send the 18-bit address (as 3 bytes, the first 6 bits are not used) */
  SPI_WRITE_BYTE(0x0);
  SPI_WRITE_BYTE(0x0);
  SPI_WRITE_BYTE(0x0);
  SPI_CLR_RXBUF;
  
  /* receive data */
  uint16_t num_bytes  = FW_FILE_SIZE;
  uint8_t* flash_addr = (uint8_t*)FLASH_START;
  while(num_bytes) {
    uint8_t b;
    SPI_READ_BYTE(b);
        
    while(!(FCTL3 & 0x08));  /* bit 3 set? -> ready for next write operation */
    *flash_addr++ = b;            /* write 1 byte to the flash memory */
    num_bytes--;
  } 
  //TODO: flash verify!
  
  /* copy the config into the info memory */
  /*num_bytes = sizeof(config_t);
  uint8_t* cfg_addr = (uint8_t*)&cfg;
  flash_addr = (uint8_t*)(INFO_START + INFO_SEG_SIZE);
  while(num_bytes) {
    while(!(FCTL3 & 0x08));
    *flash_addr++ = *cfg_addr++;
    num_bytes--;
  }*/
    
  /* lock the flash memory */
  while(!(FCTL3 & 0x08));              /* wait until write complete */
  FCTL1 = FWPW;
  FCTL3 = FWPW + LOCK + LOCKA;         /* lock the module */
  
  /* release the FRAM */
  PIN_SET(FRAM_CONF_CTRL_PIN);
    
  /* reset! */
  PMM_TRIGGER_POR;
}
/*---------------------------------------------------------------------------*/
/*#define UART_PRINT_CHAR(c) \
  while(UCA0STAT & UCBUSY); \
  UCA0TXBUF = (c)
void
uart_println(char* str) {
  UCA0CTL1 &= ~UCSWRST;
  while(*str) {
    UART_PRINT_CHAR(*str);
    str++;
  }
  UART_PRINT_CHAR('\r');
  UART_PRINT_CHAR('\n');
}
void
uart_print_hex(uint8_t* start_addr, uint16_t len) {
  UCA0CTL1 &= ~UCSWRST;
  while(len) {
    uint16_t hex_val = *start_addr >> 4;
    if(hex_val > 9) {
      hex_val = 'a' + (hex_val - 10);
    } else {
      hex_val = '0' + hex_val;
    }
    UART_PRINT_CHAR(hex_val);
    hex_val = *start_addr & 0x0f;
    if(hex_val > 9) {
      hex_val = 'a' + (hex_val - 10);
    } else {
      hex_val = '0' + hex_val;
    }
    UART_PRINT_CHAR(hex_val);
    UART_PRINT_CHAR(' ');
    start_addr++;
    len--;
  }
}
void
check_mem_content(void)
{
  uart_println("\r\nfram:");
  uint8_t buffer[128];
  fram_read(0x0, 128, buffer);
  uart_print_hex(buffer, 128);
  uart_println("\r\ninfo:");
  uart_print_hex((uint8_t*)INFO_START, 128);
  uart_println("\r\ninfo 2:");
  uart_print_hex((uint8_t*)(INFO_START + INFO_SEG_SIZE), 128);
  uart_println("\r\nint vector table:");
  fram_read(32640, 128, buffer);
  uart_print_hex(buffer, 128);
}*/
/*---------------------------------------------------------------------------*/
#endif /* FW_UPDATE_CONF_ON */
