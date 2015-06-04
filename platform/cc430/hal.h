
/**
 * @file
 * @author rdaforno
 * Hardware Abstraction Layer for the CC430F5137
 * (all HW-related definitions)
 */

#ifndef __HAL_H__
#define __HAL_H__

 
// --- board-independent configuration and macros ---

#define UART0_BAUDRATE      115200LU            // do not change this without modifying uart0.c
#define ASYNC_INT_SPEED     (SMCLK_SPEED / 1)   // desired SPI bit clock speed for the asynchronous interface (note: must not be higher than 4 MHz)
#define FRAM_SPEED          (SMCLK_SPEED / 1)   // desired SPI bit clock speed for the external FRAM, mustn't be higher than 13 MHz
#define SPI_A0_CPOL         0
#define SPI_A0_CPHA         0
#define SPI_B0_CPOL         0                   // clock polarity (default: 0 = inactive low, 1 = inactive high)
#define SPI_B0_CPHA         0                   // clock phase (see note below)
//#define DEBUG_PIN1          FLOCKLAB_LED1
//#define DEBUG_PIN2          FLOCKLAB_LED2
//#define DEBUG_PIN3          FLOCKLAB_LED3
//#define DEBUG_PIN4          FLOCKLAB_INT1
//#define DEBUG_PIN5          FLOCKLAB_INT2
// glossy debug pins
//#define GLOSSY_RX_PIN       PORT3, PIN5         // high when glossy is receiving
//#define GLOSSY_TX_PIN       PORT3, PIN4         // high when glossy is transmitting
//#define LWB_TASK_ACT_PIN    PORT3, PIN6         // show lwb task activity on this pin (comment this line to disable it)
#if !defined(FLOCKLAB) && defined(BOARD_COMM_V1)
  #define DEBUG_TASK_ACT_PIN  LED_3             // show debug task activity on this pin (comment this line to disable it)
  #define GLOSSY_START_PIN    LED_1             // high when glossy is active (comment this line to disable it)
#endif
#define STOP_WATCHDOG       ( WDTCTL = WDTPW + WDTHOLD )
#define LED_STATUS          LED_0


/*
 * Note on the SPI phase and polarity:
 * For CPOL = 0 and ...
 * ... CPHA = 0, data is captured on the clock's rising edge and changed (latched) on the falling edge.
 * ... CPHA = 1, data is captured on the clock's falling edge and changed (latched) on the rising edge.
 * For CPOL = 1 and ...
 * ... CPHA = 0, data is captured on the clock's falling edge and changed (latched) on the rising edge.
 * ... CPHA = 1, data is captured on the clock's rising edge and changed (latched) on the falling edge.
 */

 
// --- board-specific configuration (pin mapping) ---

#ifdef BOARD_EM430F5137RF900   
    #define HAS_FRAM
    #define HAS_ASYNC_INT
    #define SPI_ASYNC_INT       SPI_A0_BASE     // base address of the SPI module used to communicate with the asynchronous interface
    #define SPI_FRAM            SPI_B0_BASE     // base address of the SPI module used to communicate with the external memory (FRAM)
    #define LED_GREEN           PORT1, PIN0
    #define LED_RED             PORT3, PIN6     // don't use when using radio module
    #define LED_0               LED_GREEN
    #define AI_CTRL_IND         PORT2, PIN0
    #define AI_CTRL_MODE        PORT2, PIN1
    #define AI_CTRL_REQ         PORT2, PIN2
    #define AI_CTRL_ACK         PORT2, PIN3
    #define AI_CTRL_PWRSEL      PORT2, PIN4
    #define FRAM_CTRL           PORT2, PIN0     // control line for the external FRAM (SPI chip select / enable line)
    #define RF_GDO2_PIN         1, 1
    // flocklab pins
    #define FLOCKLAB_LED1       PORT1, PIN0
    #define FLOCKLAB_LED2       PORT1, PIN1
    #define FLOCKLAB_LED3       PORT1, PIN2     // -> Note: this pin is already used by the SPI
    #define FLOCKLAB_INT1       PORT3, PIN6
    #define FLOCKLAB_INT2       PORT3, PIN7
#endif // BOARD_EM430F5137RF900


#ifdef BOARD_MSP430CCRF  // (red) Olimex Board
    #define HAS_FRAM
    #define SPI_ASYNC_INT       SPI_A0_BASE     // base address of the SPI module used to communicate with the asynchronous interface
    #define SPI_FRAM            SPI_B0_BASE     // base address of the SPI module used to communicate with the external memory (FRAM)
    #define LED_RED             PORT1, PIN0     // Note: this board has only 1 LED
    #define LED_0               LED_RED
    #define FRAM_CTRL           PORT1, PIN7     // control line for the external FRAM (SPI chip select / enable line)
    //#define RF_GDO2_PIN         1, 1
    #define DEBUG_TASK_ACT_PIN  PORT2, PIN6
    // flocklab pins
    #define FLOCKLAB_LED1       PORT1, PIN0
    #define FLOCKLAB_LED2       PORT1, PIN1
    #define FLOCKLAB_LED3       PORT1, PIN2     // -> Note: this pin is already used by the SPI
    #define FLOCKLAB_INT1       PORT3, PIN6
    #define FLOCKLAB_INT2       PORT3, PIN7
#endif // BOARD_MSP430CCRF 


#ifdef BOARD_COMM_V1
    #define HAS_FRAM
    #define HAS_ASYNC_INT
    #define SPI_ASYNC_INT       SPI_B0_BASE     // base address of the SPI module used to communicate with the asynchronous interface
    #define SPI_FRAM            SPI_A0_BASE     // base address of the SPI module used to communicate with the external memory (FRAM)
    #define LED_0               PORT3, PIN0     // all LEDs are green
    #define LED_1               PORT3, PIN1
    #define LED_2               PORT3, PIN2
    #define LED_3               PORT3, PIN3
    #define PUSH_BUTTON         PORT1, PIN0     // new on this board, a debug push-button
    #define AI_CTRL_TIMEREQ     PORT2, PIN1     // timestamp request pin
    #define AI_CTRL_IND         PORT2, PIN2
    #define AI_CTRL_MODE        PORT2, PIN3
    #define AI_CTRL_REQ         PORT2, PIN4
    #define AI_CTRL_ACK         PORT2, PIN5
    #define AI_CTRL_PWRSEL      PORT2, PIN6
    #define DEBUG_PIN           PORT1, PIN1     // use SPI STE pin as debug pin
    #define MUX_SEL             PORT2, PIN7     // select multiplexer channel (high = UART, low = SPI)
    #define FRAM_CTRL           PORT2, PIN0     // control line for the external FRAM (SPI chip select / enable line)
    #define RF_GDO2_PIN         LED_2           // comment this line to disable RF_GDO2 debug output
    // flocklab pins
    #define FLOCKLAB_LED1       PORT3, PIN3
    #define FLOCKLAB_LED2       PORT3, PIN4
    #define FLOCKLAB_LED3       PORT3, PIN5 
    #define FLOCKLAB_INT1       PORT3, PIN6
    #define FLOCKLAB_INT2       PORT3, PIN7
#endif // BOARD_COMM_V1 


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