/**
 * This lib provides the interface towards external serial memory.
 *
 * @author rdaforno
 * @note   This driver library is intended to be used with the serial FRAM chips from Cypress (tested with the FM25V20).
 *         All operations can either be synchronous (blocking) or asynchronous (non-blocking, DMA-driven).
 * @remark The write latency is approximately 16 + num_bytes * 2.5us for an SPI clock speed of 3.25 MHz. The read latency is approx. 6us shorter.
 */

#include "fram.h"


static volatile uint8_t fram_sleeps = 0;
static uint16_t         fram_num_alloc_blocks = 0;
static uint32_t         fram_curr_offset = 0;
static volatile uint8_t fram_fill_value = 0;
static uint8_t          fram_initialized = 0;

/**
 * @brief initializes the external memory
 *
 * This function ensures that the external FRAM chip (connected to the MCU over the SPI bus) is available and operates properly.
 *
 * @note      Due to an added safety delay, it takes more than 1ms for this function to complete.
 * @return    zero if an error occurred, one otherwise
 * @remark    This function checks the external memory by reading and verifying the device ID (all device IDs start with the manufacturer ID). Therefore, a non-zero value will only be returned for FRAM chips from Cypress.
 */
uint8_t fram_init(void) {
        
    char dev_id[32];
    uint8_t c = 0;
        
    if (!fram_initialized) {
        PIN_SET(FRAM_CTRL);
        PIN_SET_AS_OUTPUT(FRAM_CTRL);
        
        if (SPI_FRAM == SPI_B0_BASE) {
            spi_b0_init(FRAM_SPEED);
        } else {
            spi_a0_init(FRAM_SPEED);        
        }    
    #ifdef FRAM_USE_DMA
        dma_init(0, 0);
    #endif // FRAM_USE_DMA
        fram_num_alloc_blocks = 0;
        fram_curr_offset = 0;
        
        __delay_cycles(MCLK_SPEED / 1000);  // make sure that at least 1ms has passed since power-up before using the FRAM
        
        // assume the FRAM is in sleep mode after power-up of the MCU
        fram_sleeps = 1;
        // read and verify the device ID (this implicitly checks if the external FRAM is available and working)
        fram_get_id(dev_id, 0); 
        while (c < 6) {
            if (0x7f != dev_id[c]) {
                DEBUG_PRINT_NOW("ERROR: fram_init failed (disconnected?)\n");
                return 0;
            }
            c++;
        }
        fram_initialized = 1;
    }    
    return 1;
}


/**
 * @brief returns the device ID of the external memory chip
 *
 * This function returns the device ID of the external FRAM chip, which is connected to the MCU over the SPI bus.
 *
 * @param[out] out_buffer pointer to the output buffer (allocated memory block must be at least 27 bytes long)
 * @param[in]  formatted if unequal zero, the output will be a string (ID coded in hex format), otherwise the output is just an array of 9 bytes
 * @return     the address of the output buffer
 */
const char* fram_get_id(char* const out_buffer, uint8_t formatted) {
    
    static uint8_t count = 0;
    static uint8_t rcv_buffer[9];
        
    FRAM_ACQUIRE;
    
    // send opcode
    SPI_TRANSMIT_BYTE(SPI_FRAM, FRAM_OPCODE_RDID);
    SPI_RECEIVE_BYTE(SPI_FRAM, *rcv_buffer);

    // receive the ID
    count = 0;
    while (count < 9) {
        SPI_TRANSMIT_BYTE(SPI_FRAM, 0x00);   // dummy write to generate the clock
        SPI_RECEIVE_BYTE(SPI_FRAM, rcv_buffer[count]);
        count++;
    }
    FRAM_RELEASE;
    
    if (formatted) {
        sprintf(out_buffer, "%02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x", rcv_buffer[0], rcv_buffer[1], rcv_buffer[2], rcv_buffer[3], rcv_buffer[4], rcv_buffer[5], rcv_buffer[6], rcv_buffer[7], rcv_buffer[8]);
    } else {
        memcpy(out_buffer, rcv_buffer, 9);
    }
    return out_buffer;
}


/**
 * @brief puts the external memory into sleep mode
 *
 * This function puts the external FRAM chip into a low-power mode.
 *
 * @remark This function does nothing in case the external memory already is in LPM.
 * @return zero if an error occurred, one otherwise
 */
uint8_t fram_sleep(void) {    
    if (fram_sleeps) {
        return 0;
    }
    FRAM_ACQUIRE;
    SPI_TRANSMIT_BYTE(SPI_FRAM, FRAM_OPCODE_SLEEP);    // send opcode
    FRAM_RELEASE;    
    fram_sleeps = 1;
    return 1;
}


/**
 * @brief wakes the external memory up
 *
 * This function puts the external FRAM chip back into active mode.
 *
 * @remark This function does nothing in case the external memory already is in active mode.
 * @return zero if an error occurred, one otherwise
 */
uint8_t fram_wakeup(void) {    
    if (!fram_sleeps) {
        return 0;
    }
    FRAM_ACQUIRE;
    FRAM_RELEASE;    
    fram_sleeps = 0;
    return 1;
}


/**
 * @brief read data from the external memory
 *
 * This function sequentially reads a specified amount of bytes from the external FRAM, starting at the given address.
 *
 * @param[in] start_addr start address (first byte to be read)
 * @param[in] num_bytes the number of bytes to be read
 * @param[out] out_data pointer to the output buffer, must be sufficiently large to hold all read bytes
 * @return    zero if an error occurred, one otherwise
 */
uint8_t fram_read(uint32_t start_addr, uint16_t num_bytes, uint8_t* out_data) {

    // validate the start address
    if (FRAM_END < start_addr) {
        return 0;
    }
    FRAM_ACQUIRE;
        
    // send opcode
    SPI_TRANSMIT_BYTE(SPI_FRAM, FRAM_OPCODE_READ);
    
    // send the 18-bit address (as 3 bytes, the first 6 bits are not used)
    SPI_TRANSMIT_BYTE(SPI_FRAM, (start_addr >> 16) & 0xff);
    SPI_TRANSMIT_BYTE(SPI_FRAM, (start_addr >> 8) & 0xff);
    SPI_TRANSMIT_BYTE(SPI_FRAM, start_addr & 0xff);
    SPI_WAIT_BUSY(SPI_FRAM);
    SPI_CLEAR_RXBUF(SPI_FRAM);

#ifdef FRAM_USE_DMA
    dma_mode = DMA_OPMODE_FRAM;    // set DMA mode
    DMA_START_RCV(out_data, num_bytes);
#else    
    // receive data
  #ifdef SPI_FAST_READ
    SPI_TRANSMIT_BYTE(SPI_FRAM, 0x00);          // transmit 1 byte ahead -> faster read speed! (with this approach, 1 excess byte will be transmitted/read
  #endif
    while (num_bytes) {
        SPI_TRANSMIT_BYTE(SPI_FRAM, 0x00);      // dummy write to generate the clock
        SPI_RECEIVE_BYTE(SPI_FRAM, *out_data);  // blocking call, waits until RX buffer is not empty
        out_data++;
        num_bytes--;
    }
    FRAM_RELEASE;
#endif // FRAM_USE_DMA
    
    return 1;
}


/**
 * @brief write data to the external memory
 *
 * This function sequentially writes a specified amount of bytes to the external FRAM, starting at the given address.
 *
 * @param[in] start_addr start address (first byte to be written to)
 * @param[in] num_bytes the number of bytes to be written
 * @param[in] data pointer to the input data, must contain [num_bytes] bytes
 * @return    zero if an error occurred, one otherwise
 */
uint8_t fram_write(uint32_t start_address, uint16_t num_bytes, const uint8_t* data) {
    
    // validate the start address
    if (FRAM_END < start_address) {
        return 0;
    }    
    FRAM_ACQUIRE;
    
    // disable the write protection feature
    SPI_TRANSMIT_BYTE(SPI_FRAM, FRAM_OPCODE_WREN);
    SPI_WAIT_BUSY(SPI_FRAM);
    PIN_SET(FRAM_CTRL);
    while (SPI_FRAM == SPI_B0_BASE ? PIN_GET_INPUT_BIT(SPI_B0_CLK) : PIN_GET_INPUT_BIT(SPI_A0_CLK));  // wait for the clock signal to go low
    PIN_CLEAR(FRAM_CTRL);
    
    // send opcode
    SPI_TRANSMIT_BYTE(SPI_FRAM, FRAM_OPCODE_WRITE);
    
    // send the 18-bit address (as 3 bytes, the first 6 bits are not used)
    SPI_TRANSMIT_BYTE(SPI_FRAM, (start_address >> 16) & 0xff);
    SPI_TRANSMIT_BYTE(SPI_FRAM, (start_address >> 8) & 0xff);
    SPI_TRANSMIT_BYTE(SPI_FRAM, start_address & 0xff);

#ifdef FRAM_USE_DMA    
    dma_mode = DMA_OPMODE_FRAM;    // set DMA mode
    DMA_START_SEND(data, num_bytes);
#else  
    // transmit data
    while (num_bytes) {
        SPI_TRANSMIT_BYTE(SPI_FRAM, *(uint8_t*)data);
        data++;
        num_bytes--;
    }
    FRAM_RELEASE;
    // Note: the write protection feature is now enabled again!
#endif // FRAM_USE_DMA
    return 1;
}


/**
 * @brief erases parts of the external memory
 *
 * This function overwrites the specified area of the external FRAM with the given (constant) value.
 *
 * @param[in] start_addr start address (first byte to be overwritten)
 * @param[in] num_bytes size of the memory area to be filled with the constant value
 * @param[in] fill_value the value to be used
 * @return    zero if an error occurred, one otherwise
 */
uint8_t fram_fill(uint32_t start_address, uint16_t num_bytes, const uint8_t fill_value) {
    
    // validate the start address
    if (FRAM_END < start_address) {
        return 0;
    }    
    FRAM_ACQUIRE;
    
    // disable the write protection feature
    SPI_TRANSMIT_BYTE(SPI_FRAM, FRAM_OPCODE_WREN);
    SPI_WAIT_BUSY(SPI_FRAM);
    PIN_SET(FRAM_CTRL);
    while (SPI_FRAM == SPI_B0_BASE ? PIN_GET_INPUT_BIT(SPI_B0_CLK) : PIN_GET_INPUT_BIT(SPI_A0_CLK));  // wait for the clock signal to go low
    PIN_CLEAR(FRAM_CTRL);
    
    // send opcode
    SPI_TRANSMIT_BYTE(SPI_FRAM, FRAM_OPCODE_WRITE);
    
    // send the 18-bit address (as 3 bytes, the first 6 bits are not used)
    SPI_TRANSMIT_BYTE(SPI_FRAM, (start_address >> 16) & 0xff);
    SPI_TRANSMIT_BYTE(SPI_FRAM, (start_address >> 8) & 0xff);
    SPI_TRANSMIT_BYTE(SPI_FRAM, start_address & 0xff);

#ifdef FRAM_USE_DMA
    SPI_SETMODE_FRAM;
    fram_fill_value = fill_value;
    DMA_START_SEND(&fram_fill_value, num_bytes);
#else  
    // transmit data
    while (num_bytes) {
        SPI_TRANSMIT_BYTE(SPI_FRAM, fill_value); 
        num_bytes--;
    }
    FRAM_RELEASE;
    // Note: the write protection feature is now enabled again!
#endif // FRAM_USE_DMA
    return 1;
}


/**
 * @brief provides a simple allocation scheme for the external memory
 *
 * This function returns the address of the first free memory block of the specified size on the external FRAM.
 *
 * @param[in] size the desired size of the memory block to be allocated
 * @return    FRAM_ALLOC_ERROR if the memory is full and the start address of the allocated memory block otherwise
 * @note      Once allocated memory cannot be freed! 
 */
uint32_t fram_alloc(uint16_t size) {

    uint32_t addr = fram_curr_offset;   // word address
    if (0 == size || ((fram_curr_offset + size) > FRAM_END)) {
        printf("ERROR: Memory allocation failed! (requested block size: %d B)", size);  // use printf to make sure this error message is printed out immediately!
        return FRAM_ALLOC_ERROR;
    }
    DEBUG_PRINT_INFO("memory allocated (block: %d, length: %d, offset: %lu)", fram_num_alloc_blocks, size, fram_curr_offset);
    fram_curr_offset += size;
    fram_num_alloc_blocks++;    
    return addr;
}


