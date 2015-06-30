/**
 * @file
 * @brief simplifies access to the external memory (FRAM)
 * @author rdaforno
 * @note this library is intended to be used with the 2 Mbit FRAM chip from Cypress (tested with the FM25V20)
 */

#ifndef __FRAM_H__
#define __FRAM_H__

#include "platform.h"


// adjust the following figures according to the used FRAM chip
#define FRAM_START          0x00000         // first byte of the external FRAM
#define FRAM_END            0x3ffff         // last byte of the external FRAM

// operation codes
#define FRAM_OPCODE_WREN    0x06            // disable write protection
#define FRAM_OPCODE_WRDI    0x04            // enable write protection
#define FRAM_OPCODE_RDSR    0x05            // read status register
#define FRAM_OPCODE_WRSR    0x01            // write status register
#define FRAM_OPCODE_READ    0x03            // read from FRAM
#define FRAM_OPCODE_WRITE   0x02            // write to FRAM
#define FRAM_OPCODE_SLEEP   0xb9            // enter sleep mode
#define FRAM_OPCODE_RDID    0x9f            // read ID

#define FRAM_ALLOC_ERROR    0xffffffff      // this address indicates a memory allocation error


/**
 * @brief checks whether the control pin is high
 */
#define FRAM_TRANSFER_IN_PROGRESS   (!PIN_GET_INPUT_BIT(FRAM_CTRL))
/**
 * @brief wait until the data transfer is complete and the control pin is high
 */
#define FRAM_WAIT_COMPLETE          while (!PIN_GET_INPUT_BIT(FRAM_CTRL))  // wait until transfer complete

/**
 * @brief access the FRAM by pulling the control pin low
 * @note  This macro returns zero in case the control pin is already low (not in idle state).
 */
#define FRAM_ACQUIRE    \
    {\
        if (PIN_GET_INPUT_BIT(FRAM_CTRL)) {  /* FRAM control/select pin must be high */\
            SPI_SELECT(SPI_FRAM);   /* make sure the module is in SPI mode */\
            SPI_ENABLE(SPI_FRAM);   /* make sure the SPI is enabled */\
            while (SPI_FRAM == SPI_B0_BASE ? PIN_GET_INPUT_BIT(SPI_B0_CLK) : PIN_GET_INPUT_BIT(SPI_A0_CLK));   /* clock must be low before pulling the select line low */\
            PIN_CLEAR(FRAM_CTRL);   /* pull select line low */\
            if (fram_sleeps) {  /* in LPM? -> wait ~0.5ms */\
                __delay_cycles(MCLK_SPEED / 2000);\
                fram_sleeps = 0;\
            }\
        } else {\
            return 0;\
        }\
    }

#ifdef FRAM_USE_DMA
    /**
     * @brief release the external memory chip, i.e. set the control/select pin (FRAM_CTRL) high and disable the SPI
     */
    #define FRAM_RELEASE    \
    {\
        if (!PIN_GET_INPUT_BIT(FRAM_CTRL)) {  /* FRAM control/select pin must be low */\
            DMA_DISABLE_TX;\
            DMA_DISABLE_RX;\
            SPI_WAIT_BUSY(SPI_FRAM);\
            SPI_CLEAR_RXBUF(SPI_FRAM);\
            PIN_SET(FRAM_CTRL);\
            SPI_DISABLE(SPI_FRAM);\
        }\
    }
#else
    /**
     * @brief release the external memory chip, i.e. set the control/select pin (FRAM_CTRL) high and disable the SPI
     */
    #define FRAM_RELEASE    \
    {\
        if (!PIN_GET_INPUT_BIT(FRAM_CTRL)) { /* FRAM control/select pin must be low */\
            SPI_WAIT_BUSY(SPI_FRAM);\
            SPI_CLEAR_RXBUF(SPI_FRAM);\
            PIN_SET(FRAM_CTRL);\
            SPI_DISABLE(SPI_FRAM);\
        }\
    }
#endif // FRAM_USE_DMA


uint8_t fram_init(void);
const char* fram_get_id(char* const out_buffer, uint8_t formatted);
uint8_t fram_sleep(void);
uint8_t fram_wakeup(void);
uint8_t fram_read(uint32_t start_address, uint16_t num_bytes, uint8_t* out_data);
uint8_t fram_write(uint32_t start_address, uint16_t num_bytes, const uint8_t* data);
uint8_t fram_fill(uint32_t start_address, uint16_t num_bytes, const uint8_t fill_value);

// memory manager is only available if DMA is not being used for the data transfer into the FRAM
#ifndef FRAM_USE_DMA
uint32_t fram_alloc(uint16_t size);
uint32_t fram_get_address(uint16_t block_id, uint16_t* const out_length);
#endif // FRAM_USE_DMA


#endif /* __FRAM_H__ */
