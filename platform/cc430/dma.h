 /**
  * @file
  * @brief provides functionality to configure the DMA
  * @author rdaforno
  */

#ifndef __DMA_H__
#define __DMA_H__

/**
 * @brief disables DMA channel 0 (used for SPI data reception)
 */
#define DMA_DISABLE_RX                  ( REGVAL16(DMA_CH0CTL) &= ~(DMAEN) ) 
/**
 * @brief enables DMA channel 0 (used for SPI data reception)
 */
#define DMA_ENABLE_RX                   ( REGVAL16(DMA_CH0CTL) |= DMAEN )
/**
 * @brief disables DMA channel 1 (used for SPI data transmission)
 */
#define DMA_DISABLE_TX                  ( REGVAL16(DMA_CH1CTL) &= ~(DMAEN) )
/**
 * @brief enables DMA channel 1 (used for SPI data transmission)
 */
#define DMA_ENABLE_TX                   ( REGVAL16(DMA_CH1CTL) |= DMAEN )

#define DMA_ENABLEINTERRUPT_RX          ( REGVAL16(DMA_CH0CTL) |= DMAIE )
#define DMA_ENABLEINTERRUPT_TX          ( REGVAL16(DMA_CH1CTL) |= DMAIE )

#define DMA_DISABLEINTERRUPT_RX         ( REGVAL16(DMA_CH0CTL) &= ~DMAIE )
#define DMA_DISABLEINTERRUPT_TX         ( REGVAL16(DMA_CH1CTL) &= ~DMAIE )

/**
 * @brief checks if an interrupt is pending for DMA channel 0
 */
#define DMA_INTERRUPTACTIVE_RX          ( REGVAL16(DMA_CH0CTL) & DMAIFG )
/**
 * @brief checks if an interrupt is pending for DMA channel 1
 */
#define DMA_INTERRUPTACTIVE_TX          ( REGVAL16(DMA_CH1CTL) & DMAIFG )

/**
 * @brief clears pending interrupts for DMA channel 0
 */
#define DMA_CLEARIFG_RX                 ( REGVAL16(DMA_CH0CTL) &= ~DMAIFG )
/**
 * @brief clears pending interrupts for DMA channel 1
 */
#define DMA_CLEARIFG_TX                 ( REGVAL16(DMA_CH1CTL) &= ~DMAIFG )

/**
 * @brief enable source address incrementation for DMA channel 1
 */
#define DMA_ENABLESRCADDRINC_TX         ( REGVAL16(DMA_CH1CTL) |= DMASRCINCR_3 )
/**
 * @brief disable source address incrementation for DMA channel 1 (the same byte will be transmitted over and over)
 */
#define DMA_DISABLESRCADDRINC_TX        { REGVAL16(DMA_CH1CTL) &= ~(DMASRCINCR_3); REGVAL16(DMA_CH1CTL) |= DMASRCINCR_0; }

/**
 * @brief set the transfer size for DMA channel 0 (number of bytes to be received)
 */
#define DMA_SETTRANSFERSIZE_RX(size)    ( REGVAL16(DMA_CH0SZ) = (uint16_t)(size) )
/**
 * @brief set the transfer size for DMA channel 1 (number of bytes to be transmitted)
 */
#define DMA_SETTRANSFERSIZE_TX(size)    ( REGVAL16(DMA_CH1SZ) = (uint16_t)(size) )

/**
 * @brief set the destination address for DMA channel 0 (where received bytes will be stored)
 */
#define DMA_SETRXBUF_ADDR(addr)         ( REGVAL16(DMA_CH0DA) = (uint16_t)(addr) )
/**
 * @brief set the source address for DMA channel 1 (data to be transmitted)
 */
#define DMA_SETTXBUF_ADDR(addr)         ( REGVAL16(DMA_CH1SA) = (uint16_t)(addr) )

/**
 * @brief returns the number of remaining (not yet received) bytes for DMA channel 0
 */
#define DMA_REMAINING_BYTES             ( REGVAL16(DMA_CH0SZ) )

/**
 * @brief configure the DMA to be used with SPI A0
 */
#define DMA_CFGFOR_SPIA0                {\
                                            REGVAL16(DMA_CH0SA) = SPI_A0_RXBUF; /* set source addr */\
                                            REGVAL16(DMA_BASE) |= DMA_TRIGGERSRC_A0RX; /* set trigger source */\
                                            REGVAL16(DMA_BASE) |= (DMA_TRIGGERSRC_A0TX << 8);\
                                            REGVAL16(DMA_CH1DA) = SPI_A0_TXBUF;\
                                        }
                                        
/**
 * @brief configure the DMA to be used with SPI B0
 */
#define DMA_CFGFOR_SPIB0                {\
                                            REGVAL16(DMA_CH0SA) = SPI_B0_RXBUF; /* set source addr */\
                                            REGVAL16(DMA_BASE) |= DMA_TRIGGERSRC_B0RX; /* set trigger source */\
                                            REGVAL16(DMA_BASE) |= (DMA_TRIGGERSRC_B0TX << 8);\
                                            REGVAL16(DMA_CH1DA) = SPI_B0_TXBUF;\
                                        }


 /**
  * @brief set up and start a DMA transfer (RX)
  *
  * This macro configures the DMA for the data reception over the SPI. 
  *
  * @remark One DMA channel is used to generate the clock (send dummy bytes) and a second channel is used to copy the received data bytes into the destination buffer.
  * @param dest_addr the address of the destination data buffer
  * @param size      the number of bytes to receive
  */
#define DMA_START_RCV(dest_addr, size)  {\
                                            DMA_SETTRANSFERSIZE_RX(size);\
                                            DMA_SETTRANSFERSIZE_TX(size - 1);\
                                            DMA_SETRXBUF_ADDR((uint16_t)dest_addr);\
                                            DMA_SETTXBUF_ADDR((uint16_t)&dummy_byte);\
                                            DMA_DISABLESRCADDRINC_TX;\
                                            DMA_DISABLEINTERRUPT_TX;\
                                            DMA_ENABLE_RX;\
                                            DMA_ENABLE_TX;\
                                            SPI_WRITE_BYTE(dummy_byte); /* write the frist byte to trigger the DMA (TXE) */\
                                        }
                                        
 /**
  * @brief set up and start a DMA transfer (TX)
  *
  * This macro configures the DMA for the data transmission over the SPI.
  *
  * @param src_addr the address of the source data buffer
  * @param size     the number of bytes to transmit
  * @param src_inc  specifies whether the source address shall be increased after the transmission of each byte (zero means the same byte will be sent [size] times)
  */
#define DMA_START_SEND(src_addr, size, src_inc)  {\
                                            src_inc ? DMA_ENABLESRCADDRINC_TX : DMA_DISABLESRCADDRINC_TX;\
                                            DMA_SETTRANSFERSIZE_TX(size - 1);\
                                            DMA_SETTXBUF_ADDR(src_inc ? ((uint16_t)src_addr + 1) : src_addr);\
                                            DMA_ENABLEINTERRUPT_TX; \
                                            DMA_ENABLE_TX;\
                                            SPI_WRITE_BYTE(*(uint8_t*)src_addr); /* write the frist byte to trigger the DMA (TXE) */\
                                        }
                                        

 /**
  * @brief operating modes for the DMA
  *
  * There are two predefined operating modes for the DMA, i.e. it is either configured to operate with the FRAM or the asynchronous interface.
  */
typedef enum {
    DMA_OPMODE_FRAM = 0,
    DMA_OPMODE_ASYNCINT,
    NUM_OF_SPI_MODES
} dma_mode_t;


void dma_init(uint16_t rx_buffer_addr, uint16_t tx_buffer_addr);
void dma_init_ch2(uint16_t src_addr, uint16_t dest_addr);


extern dma_mode_t   dma_mode;
extern uint8_t      dummy_byte;


#endif /* __DMA_H__ */
