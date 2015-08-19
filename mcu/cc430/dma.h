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

/**
 * @addtogroup  Platform
 * @{
 *
 * @defgroup    dma DMA
 * @{
 *
 * @file
 *
 * @brief Provides a setup function for the DMA (CH0 and CH1, to be used for
 * the SPI and CH2 for the timer CI). 
 * @note  this library only works if the device is SPI master
 */

#ifndef __DMA_H__
#define __DMA_H__

#define DMA_NUM_CHANNELS                3

/**
 * @brief returns the number of remaining (not yet received) bytes for DMA
 * channel 0
 */
#define DMA_REMAINING_BYTES_RX          (DMA0SZ)


/**
 * @brief DMA trigger sources for the dma_init_timer function (DMA_CH2)
 */
typedef enum {
  DMA_TRGSRC_TA0CCR0 = 1,
  DMA_TRCSRC_TA0CCR2,
  DMA_TRCSRC_TA1CCR0,
  DMA_TRGSRC_TA1CCR2,
} dma_triggersrc_t;

/**
 * @brief callback function declaration for the DMA TC interrupt
 */
typedef void (*dma_callback_t)(void);

/**
 * @brief configure the DMA CH0 and CH1 for SPI A0 or SPI B0 (RX and TX)
 * @param[in] spi_addr base address of the USCI module (USCI_A0 or USCI_B0)
 * @param[in] callback_func the callback function for the DMA TC interrupt
 * @remark The DMA is configured in single transfer, byte-to-byte mode.
 */
void dma_config_spi(uint16_t spi_addr,
                    dma_callback_t callback_func);

/**
 * @brief configure the DMA CH2 for timer TA0 or TA1 to copy num_bytes bytes
 * from src_addr to dest_addr upon input capture
 * @param[in] trigger_src the trigger source for the DMA transfer (e.g. 
 * DMA_TRGSRC_TA1CCR0)
 * @param[in] src_addr address of the reception buffer (pass zero if you
 * want to set this later by using DMA_SETRXBUF_ADDR)
 * @param[in] dest_addr address of the transmission buffer (pass zero if
 * you want to set this later by using DMA_SETTXBUF_ADDR)
 * @param[in] num_bytes transfer size (must be at least 2 bytes)
 * @remark The DMA is configured in single transfer, byte-to-byte mode.
 */
void dma_config_timer(dma_triggersrc_t trigger_src, 
                      uint16_t src_addr, uint16_t dest_addr, uint8_t num_bytes);

/**
 * @brief start a data transfer on CH0 and CH1 (SPI RX and TX)
 * @param[in] rx_buf_addr address of the reception buffer (set this to zero if
 * the received data is not of interest)
 * @param[in] tx_buf_addr address of the transmission buffer (set this to zero 
 * if the sent data is not of interest)
 * @param[in] num_bytes the number of bytes to transfer from/to SPI RX/TXBuffer
 * @return 1 if successful, 0 otherwise
 */
uint8_t dma_start(uint16_t rx_buf_addr, uint16_t tx_buf_addr, uint16_t num_bytes);


#endif /* __DMA_H__ */

/**
 * @}
 * @}
 */
