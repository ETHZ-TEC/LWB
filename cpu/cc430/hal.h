
/**
 * @file
 * @author rdaforno
 * Hardware Abstraction Layer for the CC430F5137
 * (all HW-related definitions)
 */

#ifndef __HAL_H__
#define __HAL_H__



// --- dedicated GPIO pins, board-independent ---

#define SPI_B0_SOMI         PORT1, PIN2
#define SPI_B0_SIMO         PORT1, PIN3
#define SPI_B0_CLK          PORT1, PIN4
#define SPI_B0_STE          PORT1, PIN7         // enable / select pin   
#define SPI_A0_SOMI         PORT1, PIN5
#define SPI_A0_SIMO         PORT1, PIN6
#define SPI_A0_CLK          PORT1, PIN7
#define UART_A0_RX          PORT1, PIN5         // input
#define UART_A0_TX          PORT1, PIN6         // output
#define RF_GDO1             PORT3, PIN6
#define RF_SMCLK            PORT3, PIN7


// --- hardware addresses, board-independent ---

#define GPIO_P1P2_BASE      0x0200      // base address of port 1 and port 2
#define GPIO_P3P4_BASE      0x0220      
#define GPIO_P5_BASE        0x0240      
#define GPIO_POUT_ODD_OFS   0x0002      // offset of the output byte from the base address for odd port numbers (1, 3, 5)
#define GPIO_POUT_EVEN_OFS  0x0003
#define GPIO_PSEL_ODD_OFS   0x000A
#define GPIO_PSEL_EVEN_OFS  0x000B
#define GPIO_PDIR_ODD_OFS   0x0004
#define GPIO_PDIR_EVEN_OFS  0x0005
#define GPIO_PREN_ODD_OFS   0x0006
#define GPIO_PREN_EVEN_OFS  0x0007
#define GPIO_P1IES_OFS      0x0018
#define GPIO_P2IES_OFS      0x0019
#define GPIO_P1IE_OFS       0x001A
#define GPIO_P2IE_OFS       0x001B
#define GPIO_P1IFG_OFS      0x001C
#define GPIO_P2IFG_OFS      0x001D
#define GPIO_PIN_ODD_OFS    0x0000
#define GPIO_PIN_EVEN_OFS   0x0001

#define SPI_A0_BASE         0x05C0
#define SPI_B0_BASE         0x05E0
#define SPI_CTL0_OFS        0x0001
#define SPI_CTL1_OFS        0x0000
#define SPI_BR_OFS          0x0006      // baud-rate offset
#define SPI_TXBUF_OFS       0x000E
#define SPI_IE_OFS          0x001C      // interrupt enable register offset
#define SPI_IFG_OFS         0x001D      // interrupt flag register offset
#define SPI_STAT_OFS        0x000A      // status register offset
#define SPI_RXBUF_OFS       0x000C
#define SPI_A0_TXBUF        (SPI_A0_BASE + SPI_TXBUF_OFS)
#define SPI_A0_RXBUF        (SPI_A0_BASE + SPI_RXBUF_OFS)
#define SPI_B0_TXBUF        (SPI_B0_BASE + SPI_TXBUF_OFS)
#define SPI_B0_RXBUF        (SPI_B0_BASE + SPI_RXBUF_OFS)

#define UCS_BASE            0x0160      // unified clocking system
#define UCS_CTL4_OFS        0x0008
#define UCS_CTL5_OFS        0x000A

#define DMA_BASE            0x0500      // this is also the location of the DMA module control 0 register
#define DMA_CH0_OFS         0x0000
#define DMA_CH1_OFS         0x0010
#define DMA_CH2_OFS         0x0020
#define DMA_CTL_OFS         0x0010                                      // channel control register offset
#define DMA_CH0CTL          (DMA_BASE + DMA_CH0_OFS + DMA_CTL_OFS)      // channel 0 control register
#define DMA_CH1CTL          (DMA_BASE + DMA_CH1_OFS + DMA_CTL_OFS)
#define DMA_SZ_OFS          0x001A                                      // channel 0 transfer size offset
#define DMA_CH0SZ           (DMA_BASE + DMA_CH0_OFS + DMA_SZ_OFS)       // channel 0 transfer size register
#define DMA_CH1SZ           (DMA_BASE + DMA_CH1_OFS + DMA_SZ_OFS)
#define DMA_SA_OFS          0x0012                                      // source address offset
#define DMA_DA_OFS          0x0016                                      // destination address offset
#define DMA_CH0SA           (DMA_BASE + DMA_CH0_OFS + DMA_SA_OFS)       // channel 0 source address register
#define DMA_CH0DA           (DMA_BASE + DMA_CH0_OFS + DMA_DA_OFS)       // channel 0 destination address
#define DMA_CH1SA           (DMA_BASE + DMA_CH1_OFS + DMA_SA_OFS)       // channel 0 source address
#define DMA_CH1DA           (DMA_BASE + DMA_CH1_OFS + DMA_DA_OFS)       // channel 0 destination address
#define DMA_TRIGGERSRC_A0RX 0x10
#define DMA_TRIGGERSRC_A0TX 0x11
#define DMA_TRIGGERSRC_B0RX 0x12
#define DMA_TRIGGERSRC_B0TX 0x13


// memory definitions
#define FLASH_START         0x8000
#define FLASH_END           0xff7f      // last byte of the flash memory (note: the last 128 bytes are used for the interrupt vectors!)
#define FLASH_LAST_SEG      0xfe00      // last erasable segment (do not touch the very last segment!)
#define FLASH_SIZE          0x7f80      // 32640 B
#define RAM_START           0x1c00
#define RAM_END             0x2bff
#define RAM_SIZE            0x1000      // 4 kB
#define INFO_START          0x1800
#define INFO_END            0x19ff
#define INFO_SIZE           0x0200      // 512 B
#define BSL_START           0x1000
#define BSL_END             0x17ff
#define BSL_SIZE            0x0800      // 8 * 256 = 2 kB

#define MCU_TYPE            "CC430F5137"

#endif /* __HAL_H__ */