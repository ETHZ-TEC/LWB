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
 * @author
 *              Reto Da Forno
 *
 * @brief Provides a setup function for the DMA (CH 0 and 1, to be used for the
 * SPI)
 * @note  the SPI must be configured in master mode
 */

#ifndef __DMA_H__
#define __DMA_H__

/**
 * @brief disables DMA channel 0 (used for SPI data reception)
 */
#define DMA_DISABLE_RX                  (DMA0CTL &= ~(DMAEN))
/**
 * @brief enables DMA channel 0 (used for SPI data reception)
 */
#define DMA_ENABLE_RX                   (DMA0CTL |= DMAEN)
/**
 * @brief disables DMA channel 1 (used for SPI data transmission)
 */
#define DMA_DISABLE_TX                  (DMA1CTL &= ~(DMAEN))
/**
 * @brief enables DMA channel 1 (used for SPI data transmission)
 */
#define DMA_ENABLE_TX                   (DMA1CTL |= DMAEN)

#define DMA_ENABLEINTERRUPT_RX          (DMA0CTL |= DMAIE)
#define DMA_ENABLEINTERRUPT_TX          (DMA1CTL |= DMAIE)

#define DMA_DISABLEINTERRUPT_RX         (DMA0CTL &= ~DMAIE)
#define DMA_DISABLEINTERRUPT_TX         (DMA1CTL &= ~DMAIE)

/**
 * @brief checks if an interrupt is pending for DMA channel 0
 */
#define DMA_INTERRUPTACTIVE_RX          (DMA0CTL & DMAIFG)
/**
 * @brief checks if an interrupt is pending for DMA channel 1
 */
#define DMA_INTERRUPTACTIVE_TX          (DMA1CTL & DMAIFG)

/**
 * @brief enable source address incrementation for DMA channel 1
 */
#define DMA_ENABLESRCADDRINC_TX         (DMA1CTL |= DMASRCINCR_3)
/**
 * @brief disable source address incrementation for DMA channel 1 (the same
 * byte will be transmitted over and over)
 */
#define DMA_DISABLESRCADDRINC_TX        ( DMA1CTL = (DMA1CTL & ~(DMASRCINCR_3))\
                                                    | DMASRCINCR_0 )

/**
 * @brief set the transfer size for DMA channel 0 (number of bytes to be
 * received)
 */
#define DMA_SETTRANSFERSIZE_RX(size)    (DMA0SZ = (uint16_t)(size))
/**
 * @brief set the transfer size for DMA channel 1 (number of bytes to be
 * transmitted)
 */
#define DMA_SETTRANSFERSIZE_TX(size)    (DMA1SZ = (uint16_t)(size))


/**
 * @brief returns the number of remaining (not yet received) bytes for DMA
 * channel 0
 */
#define DMA_REMAINING_BYTES             (DMA0SZ)


/**
 * @brief set up and start a DMA transfer (RX)
 *
 * This macro configures the DMA for the data reception over the SPI.
 *
 * @remark One DMA channel is used to generate the clock (send dummy bytes) and
 * a second channel is used to copy the received data bytes into the
 *destination
 * buffer.
 * @param dest_addr the address of the destination data buffer
 * @param size      the number of bytes to receive
 */
#define DMA_START_RCV(dest_addr, size) { \
    DMA_SETTRANSFERSIZE_RX(size); \
    DMA_SETTRANSFERSIZE_TX(size - 1); \
    DMA_SETRXBUF_ADDR((uint16_t)dest_addr); \
    DMA_SETTXBUF_ADDR((uint16_t)&dummy_byte); \
    DMA_DISABLESRCADDRINC_TX; \
    DMA_DISABLEINTERRUPT_TX; \
    DMA_ENABLEINTERRUPT_RX; \
    DMA_ENABLE_RX; \
    DMA_ENABLE_TX; \
    SPI_WRITE_BYTE(dummy_byte); /* write the frist byte to trigger the DMA\
                                   (TXE) */ \
}

/**
 * @brief set up and start a DMA transfer (TX)
 *
 * This macro configures the DMA for the data transmission over the SPI.
 *
 * @param src_addr the address of the source data buffer
 * @param size     the number of bytes to transmit
 * @param src_inc  specifies whether the source address shall be increased
 * after the transmission of each byte (zero means the same byte will be sent
 * [size] times)
 */
#define DMA_START_SEND(src_addr, size, src_inc) { \
    src_inc ? DMA_ENABLESRCADDRINC_TX : DMA_DISABLESRCADDRINC_TX; \
    DMA_SETTRANSFERSIZE_TX(size - 1); \
    DMA_SETTXBUF_ADDR(src_inc ? ((uint16_t)src_addr + 1) : src_addr); \
    DMA_DISABLEINTERRUPT_RX; \
    DMA_ENABLEINTERRUPT_TX; \
    DMA_ENABLE_TX; \
    SPI_WRITE_BYTE(*(uint8_t *)src_addr);  /* write the frist byte to trigger\
                                              the DMA (TXE) */ \
}

/**
 * @brief operating modes for the DMA
 *
 * There are two predefined operating modes for the DMA, i.e. it is either
 * configured to operate with the FRAM or the asynchronous interface.
 */
typedef enum {
  DMA_OPMODE_FRAM = 0,
  DMA_OPMODE_ASYNCINT,
  NUM_OF_SPI_MODES
} dma_mode_t;

/**
 * @brief callback function declaration for the DMA TC interrupt
 */
typedef void (*dma_callback_t)(void);

/**
 * @brief configure the DMA CH0 and CH1 for SPI A0 or SPI B0 (RX and TX)
 * @param[in] rx_buffer_addr address of the reception buffer (pass zero if you
 * want to set this later by using DMA_SETRXBUF_ADDR)
 * @param[in] tx_buffer_addr address of the transmission buffer (pass zero if
 * you want to set this later by using DMA_SETTXBUF_ADDR)
 * @param[in] callback_func the callback function for the DMA TC interrupt
 * @note Call DMA_CFGFOR_SPIA0 after dma_init() to configure the DMA for
 * SPI A0.
 * @remark The DMA is configured in single transfer, byte-to-byte mode.
 */
void dma_init_spi(uint16_t spi_addr,
                  uint16_t rx_buffer_addr, 
                  uint16_t tx_buffer_addr,
                  dma_callback_t callback_func);

/**
 * @brief configure the DMA CH2 for timer TA0 or TA1
 * @param[in] rx_buffer_addr address of the reception buffer (pass zero if you
 * want to set this later by using DMA_SETRXBUF_ADDR)
 * @param[in] tx_buffer_addr address of the transmission buffer (pass zero if
 * you want to set this later by using DMA_SETTXBUF_ADDR)
 * @param[in] callback_func the callback function for the DMA TC interrupt
 * @note Call DMA_CFGFOR_SPIA0 after dma_init() to configure the DMA for
 * SPI A0.
 * @remark The DMA is configured in single transfer, byte-to-byte mode.
 */
void dma_init_timer(uint16_t src_addr, uint16_t dest_addr);

extern dma_mode_t dma_mode;
extern uint8_t dummy_byte;

#endif /* __DMA_H__ */

/**
 * @}
 * @}
 */
