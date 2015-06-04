/**
 * Provides a setup function for the DMA (CH 0 and 1, to be used for the SPI)
 *
 * @note   the SPI must be configured in master mode
 */
  
#include "contiki.h"

dma_mode_t  dma_mode;           // DMA can only be used by one interface at a time, i.e. either the async interface or the FRAM
uint8_t     dummy_byte = 0x00;


/**
 * @brief initializes the DMA
 *
 * This function configures the DMA channel 0 and 1 for SPI B0 RX and TX.
 *
 * @param[in] rx_buffer_addr address of the reception buffer (pass zero if you want to set this later by using DMA_SETRXBUF_ADDR)
 * @param[in] tx_buffer_addr address of the transmission buffer (pass zero if you want to set this later by using DMA_SETTXBUF_ADDR)
 * @note      Call DMA_CFGFOR_SPIA0 after dma_init() to configure the DMA for SPI A0.
 * @remark    The DMA is configured in single transfer, byte-to-byte mode.
 */
void dma_init(uint16_t rx_buffer_addr, uint16_t tx_buffer_addr) {
    // use channel 0 for RX (reception) and channel 1 for TX (transmission)

    // channel 0 (RX)
    
    REGVAL16(DMA_CH0CTL) = DMADT_0 + DMASRCBYTE + DMADSTBYTE;   // single transfer, byte-to-byte, rising edge triggers
    REGVAL16(DMA_CH0SZ)  = MESSAGE_SIZE;                        // set transfer size to a default value
    
    // set DMA control 0 register
    REGVAL16(DMA_BASE) &= 0xFF00;                  // reset trigger select
    REGVAL16(DMA_BASE) |= DMA_TRIGGERSRC_B0RX;
 
    // set source address
    REGVAL16(DMA_CH0SA) = SPI_B0_RXBUF;
    // reset bits before setting them
    REGVAL16(DMA_CH0CTL) &= ~DMASRCINCR_3;
    REGVAL16(DMA_CH0CTL) |= DMASRCINCR_0;          // do not increment source address
    // set destination address
    DMA_SETRXBUF_ADDR(rx_buffer_addr);
    // reset bits before setting them
    //REGVAL16(DMA_CH0CTL) &= ~DMADSTINCR_3;
    REGVAL16(DMA_CH0CTL) |= DMADSTINCR_3;   // increment destination address
    
    // channel 1 (TX)
    
    REGVAL16(DMA_CH1CTL) = DMADT_0 + DMASRCBYTE + DMADSTBYTE;   // single transfer, byte-to-byte, rising edge triggers
    REGVAL16(DMA_CH1SZ)  = MESSAGE_SIZE;
    
    // set DMA control 0 register
    REGVAL16(DMA_BASE) &= 0x00FF;   // reset trigger select
    REGVAL16(DMA_BASE) |= DMA_TRIGGERSRC_B0TX << 8;
    
    // set source address
    DMA_SETTXBUF_ADDR(tx_buffer_addr + 1);
    //REGVAL16(DMA_CH1CTL) &= ~(DMASRCINCR_3);     // no need to clear the bits first
    REGVAL16(DMA_CH1CTL) |= DMASRCINCR_3;          // increment address
    // set destination address
    REGVAL16(DMA_CH1DA)   = SPI_B0_TXBUF;
    // reset bits before setting them
    REGVAL16(DMA_CH1CTL) &= ~DMADSTINCR_3;
    REGVAL16(DMA_CH1CTL) |= DMADSTINCR_0;   // do not increment address
    
    // enable DMA interrupts
    DMA_CLEARIFG_RX;        // clear the interrupt flag 
    DMA_CLEARIFG_TX;
    DMA_ENABLEINTERRUPT_RX; // RX interrupt always enabled
}


void dma_init_ch2(uint16_t src_addr, uint16_t dest_addr) {
    // use channel 2 to take a snapshot of the sw extension of the timer TA1
    
    DMA2CTL &= ~(DMASRCBYTE + DMADSTBYTE + DMALEVEL + DMASRCINCR_3 + DMADSTINCR_3 + DMAEN + DMAIE + DMAIFG);     // disable DMA and reset bits
    DMA2CTL  = DMADT_1;                 // block transfer, word-to-word, rising edge triggers
    DMA2SZ   = 4;                       // set transfer size
    DMACTL1 &= ~0x00FF;                 // reset trigger select
    DMACTL1 |= 0x0003;                  // set trigger in DMA control 1 register
    DMA2SA   = src_addr;                // set source address
    DMA2DA   = dest_addr;               // set destination address
    DMA2CTL |= (DMASRCINCR_3 + DMADSTINCR_3);   // increment source and destination address
    DMA2CTL |= DMAEN;                   // enable DMA
}


ISR(DMA, dma_interrupt) {

    ENERGEST_ON(ENERGEST_TYPE_CPU);
    
    if (DMA_INTERRUPTACTIVE_RX) {
        DMA_CLEARIFG_RX;        
    } else if (DMA_INTERRUPTACTIVE_TX) {
        DMA_CLEARIFG_TX;
    }
#ifdef ASYNC_INT_USE_DMA
    if (DMA_OPMODE_ASYNCINT == dma_mode) {
        async_int_release();
        DEBUG_PRINT_VERBOSE("DMA transfer terminated, async interface released");
    }
#endif  // ASYNC_INT_USE_DMA
#ifdef FRAM_USE_DMA
    if (DMA_OPMODE_FRAM == dma_mode) {
        FRAM_RELEASE;
        DEBUG_PRINT_VERBOSE("DMA transfer terminated, async interface released");
    }
#endif  // FRAM_USE_DMA

    ENERGEST_OFF(ENERGEST_TYPE_CPU);
}

