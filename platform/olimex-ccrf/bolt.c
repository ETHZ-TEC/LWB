/**
 * This library provides functionality to configure and use the asynchronous data interface (connected to the MCU over the SPI bus).
 *
 * @author rdaforno
 * @remark The data transfer over the SPI can either be synchronous (blocking, polling/busy wait) or asynchronous (interrupt/DMA-driven).
 */
  
#include "contiki.h"


#ifdef HAS_ASYNC_INT

volatile ai_state async_int_state = STATE_IDLE;
#ifdef ASYNC_INT_TIMEREQ_POLLING
    static rtimer_clock_t ta1_timestamp = 0;
#endif

#ifdef ASYNC_INT_USE_DMA
    static uint16_t   async_int_rx_buffer = 0;
    static uint16_t   async_int_tx_buffer = 0;
#endif // ASYNC_INT_USE_DMA



/**
 * @brief initializes all required GPIO pins and peripherals to use the asynchronous data interface
 *
 * Configures the GPIO pins AI_CTRL_IND, AI_CTRL_MODE, AI_CTRL_REQ and AI_CTRL_ACK as well as the peripheral modules SPI_ASYNC_INT and the DMA (if ASYNC_INT_USE_DMA is defined).
 *
 * @param[in] rx_buffer_addr address of the destination buffer, i.e. start address of the allocated memory block to store the received bytes (in DMA mode only) 
 * @param[in] tx_buffer_addr address of the source buffer, i.e. start address of the data to be transmitted (in DMA mode only) 
 * @note      if ASYNC_INT_USE_DMA is not defined, async_int_init() does not expect any parameters.
 */
#ifdef ASYNC_INT_USE_DMA
  #ifdef ASYNC_INT_TIMEREQ_POLLING
    void async_int_init(uint16_t rx_buffer_addr, uint16_t tx_buffer_addr) {
  #else
    void async_int_init(uint16_t rx_buffer_addr, uint16_t tx_buffer_addr, rtimer_ta1_callback_t func) {  
  #endif // ASYNC_INT_TIMEREQ_POLLING
#else
  #ifdef ASYNC_INT_TIMEREQ_POLLING
    void async_int_init(void) {
  #else
    void async_int_init(rtimer_ta1_callback_t func) {
  #endif // ASYNC_INT_TIMEREQ_POLLING
#endif // ASYNC_INT_USE_DMA

    // control signals
    PIN_SET_AS_INPUT(AI_CTRL_IND);
    PIN_RESISTOR_EN(AI_CTRL_IND);       // enable resistor to prevent floating input
    //PIN_CFG_PORT_INT(AI_CTRL_IND);   // don't enable the port interrupt for this pin
    PIN_UNSELECT(AI_CTRL_MODE);
    PIN_CLEAR(AI_CTRL_MODE);
    PIN_SET_AS_OUTPUT(AI_CTRL_MODE);
    PIN_UNSELECT(AI_CTRL_REQ);
    PIN_CLEAR(AI_CTRL_REQ);
    PIN_SET_AS_OUTPUT(AI_CTRL_REQ);
    PIN_SET_AS_INPUT(AI_CTRL_ACK);
    PIN_RESISTOR_EN(AI_CTRL_ACK);
    //PIN_CFG_PORT_INT(AI_CTRL_ACK);   // don't enable interrupts for this pin, use busy wait (polling) instead!
        
    // SPI
    if (SPI_ASYNC_INT == SPI_B0_BASE) {
        spi_b0_init(ASYNC_INT_SPEED);
    } else {
        spi_a0_init(ASYNC_INT_SPEED);
    }    
    
#ifdef AI_CTRL_TIMEREQ
    PIN_SET_AS_MODULE_FUNC(AI_CTRL_TIMEREQ);
    PIN_SET_AS_INPUT(AI_CTRL_TIMEREQ);
    PIN_RESISTOR_EN(AI_CTRL_TIMEREQ);
    // configure TA1 CCR0 to capture the timestamp on an edge change on pin 2.1 (do NOT enable interrupts!)
    TA1CCTL0 &= ~(CM_3 + CCIS_3 + SCS + CCIE + OUT + COV + CCIFG);      // clear all bits
  #ifdef ASYNC_INT_TIMEREQ_POLLING
    TA1CCTL0 |= (CAP + CM_1 + SCS + CCIS_0 + OUTMOD_0);                 // rising edge, synchronize the capture with the next timer clock to prevent race conditions, capture input select
    dma_init_ch2((uint16_t)&ta0_sw_ext, (uint16_t)&ta1_timestamp);      // use the DMA to take a snapshot of the 64-bit sw timer extension
  #else
    TA1CCTL0 |= (CAP + CM_1 + SCS + CCIS_0 + OUTMOD_0 + CCIE);          // enable interrupt, don't use DMA
    // set the rtimer callback function
    rtimer_set_callback(RTIMER_TA1_0, func);
  #endif
#endif
 
#ifdef ASYNC_INT_USE_DMA
    if (0 == rx_buffer_addr || 0 == tx_buffer_addr) {
        DEBUG_PRINT_WARNING("invalid parameters for async_int_init");
    }
    async_int_rx_buffer = rx_buffer_addr;
    async_int_tx_buffer = tx_buffer_addr;
    dma_init(rx_buffer_addr, tx_buffer_addr);         // maybe embed this call into asnyc_int_init()
#endif 
}


#ifdef ASYNC_INT_TIMEREQ_POLLING
/**
 * @brief process a timestamp request
 *
 * Checks if there is a pending timestamp request and returns the timestamp if so.
 *
 * @return one if there is a timestamp request pending, zero otherwise
 */
uint8_t async_int_timereq_pending(uint8_t* out_buffer) {
    if (DMA2CTL & DMAIFG || TA1CCTL0 & CCIFG) {     // interrupt flag set?
        ta1_timestamp  = (ta1_timestamp << 16) | TA1CCR0;
        DEBUG_PRINT_INFO("timestamp: %llu (now: %llu)", ta1_timestamp, rtimer_now());
        TA1CCTL0 &= ~(CCIFG + COV);
        DMA2CTL  &= ~DMAIFG;
        DMA2CTL  |= DMAEN;      // re-enable the DMA
        // send the timestamp to the application processor
        if (out_buffer) {
            memcpy(out_buffer, &ta1_timestamp, 8);
            ta1_timestamp = 0;
        }
        return 1;
    }
    return 0;
}
#endif // ASYNC_INT_TIMEREQ_POLLING


 /**
 * @brief release the asynchronous data interface and clean up
 *
 * Resets the REQ pin to put the asynchronous interface back into idle state and disables the DMA and SPI.
 * @note Any ongoing operation on SPI_ASYNC_INT will be terminated immediately.
 */
void async_int_release(void) {
    // --- 1. stop DMA ---
#ifdef ASYNC_INT_USE_DMA
    DMA_DISABLE_TX; 
    DMA_DISABLE_RX;
#endif
	// --- 2. wait for BUSY flag ---
	SPI_WAIT_BUSY(SPI_ASYNC_INT);
	// --- 3. set REQ = 0 ---
	PIN_CLEAR(AI_CTRL_REQ);
    //REGVAL8(GPIO_P1P2_BASE + GPIO_POUT_EVEN_OFS) &= ~PIN2;
	// --- 4. empty the RX buffer ---
	SPI_CLEAR_RXBUF(SPI_ASYNC_INT);
    SPI_DISABLE(SPI_ASYNC_INT);    // disable SPI (optional)
    
#ifdef ASYNC_INT_USE_DMA
    if (STATE_READ == async_int_state && 0 != async_int_rx_buffer) {
        *(uint8_t*)(async_int_rx_buffer + MESSAGE_SIZE - 1) = 0;
        DEBUG_PRINT_INFO("message received: '%s'", (char*)async_int_rx_buffer);
    }    
#endif // ASYNC_INT_USE_DMA

    // --- 5. wait for ACK to go down ---
	while (PIN_GET_INPUT_BIT(AI_CTRL_ACK));
    async_int_state = STATE_IDLE;
    DEBUG_PRINT_VERBOSE("back in idle state");
}


/**
 * @brief requests an operation on the asynchronous data interface
 *
 * Prepares a data transfer over the asynchronous interface by enabling the SPI, setting up the DMA (if ASYNC_INT_USE_DMA is defined) and acquiring a lock (set request pin high).
 *
 * @param[in] mode the operating mode, either OP_READ or OP_WRITE
 * @param[in] num_bytes number of bytes to transmit (in DMA mode only)
 * @return    one if the request was successful (REQ pin was set), zero otherwise
 */
#ifdef ASYNC_INT_USE_DMA
uint8_t async_int_req_op(op_mode mode, uint16_t num_bytes) {
#else
uint8_t async_int_req_op(op_mode mode) {
#endif // ASYNC_INT_USE_DMA

    if (PIN_GET_INPUT_BIT(AI_CTRL_REQ) || PIN_GET_INPUT_BIT(AI_CTRL_ACK)) {
        DEBUG_PRINT_ERROR("request failed (REQ or ACK still high)");
        return 0;
    }
    if (STATE_IDLE != async_int_state) {
        DEBUG_PRINT_ERROR("not in idle state, operation skipped");
        return 0;
    }
    // make sure the SPI is enabled
    SPI_ENABLE(SPI_ASYNC_INT);
        
    // --- 1. set MODE ---
    if (OP_READ == mode) {
        if (!ASYNC_INT_DATA_AVAILABLE) {
            DEBUG_PRINT_WARNING("no data available, read operation skipped");
            return 0;
        }
        PIN_CLEAR(AI_CTRL_MODE);    // 0 = READ
        async_int_state = STATE_PREPREAD;
        DEBUG_PRINT_VERBOSE("requesting read operation...");
        
        // --- 2. set up DMA (or use polling) ---
    #ifdef ASYNC_INT_USE_DMA
        dma_mode = DMA_OPMODE_ASYNCINT;
        //DMA_SETRXBUF_ADDR(async_int_rx_buffer); 
        //DMA_SETTRANSFERSIZE_RX(MESSAGE_SIZE); 
        DMA_DISABLEINTERRUPT_TX; 
        DMA_SETTRANSFERSIZE_TX(MESSAGE_SIZE - 1);
    #endif    
    } else {    
        PIN_SET(AI_CTRL_MODE);      // 1 = WRITE
        async_int_state = STATE_PREPWRITE;
        DEBUG_PRINT_VERBOSE("requesting write operation...");
        
        // --- 2. set up DMA (or use polling) ---
    #ifdef ASYNC_INT_USE_DMA        
        dma_mode = DMA_OPMODE_ASYNCINT;
        DMA_ENABLEINTERRUPT_TX; 
        //DMA_SETTXBUF_ADDR(async_int_tx_buffer); 
        DMA_SETTRANSFERSIZE_TX(num_bytes - 1);
    #endif    
    }
    // --- 3. set a timeout (~ 0.5 ms) ---  -> or just wait a few cycles
    //rtimer_schedule(ASYNC_INT_RTIMER_ID, rtimer_now() + RTIMER_SECOND / 2000, 0, (rtimer_callback_t)async_int_release);
    
    // --- 4. set REQ = 1 ---
    PIN_SET(AI_CTRL_REQ);
    
    // now wait for the ACK to go high (Note: this is only needed if ACK port interrupt disabled)
    //while (STATE_IDLE != async_int_state && !ASYNC_INT_ACK_STATUS);
    //if (STATE_IDLE == async_int_state) {
    //    // the timeout occurred -> failed
    //    return 0;
    //}
    __delay_cycles(MCLK_SPEED / 20000);     // wait 50 us
    if (!ASYNC_INT_ACK_STATUS) {
        // ack is still low -> failed
        async_int_state = STATE_IDLE;
        PIN_CLEAR(AI_CTRL_REQ);
        return 0;
    }
    async_int_state = (mode == OP_READ) ? STATE_READ : STATE_WRITE;
    //rtimer_stop(ASYNC_INT_RTIMER_ID);
    
    return 1;
}



#ifdef ASYNC_INT_USE_DMA
/**
 * @brief start an operation on the asynchronous data interface
 *
 * Starts the data transfer (read or write). 
 *
 */
void async_int_start_op(void) {
#else
/**
 * @brief start an operation on the asynchronous data interface
 *
 * Starts the data transfer (read or write). 
 *
 * @param[in,out] data a pointer to the data buffer (an input in write mode and an output in read mode)
 * @param[in,out] num_bytes the number of bytes to transmit (in write mode); the number of transmitted bytes will be stored in num_bytes 
 */
void async_int_start_op(uint8_t* data, uint16_t* num_bytes) {
    
    if (0 == num_bytes) {
        return;
    }
#endif // ASYNC_INT_USE_DMA

    DEBUG_PRINT_VERBOSE("starting data transfer... ");
    
    if (STATE_WRITE == async_int_state) {
        if (0 == *num_bytes) {
            return;
        }
#ifdef ASYNC_INT_USE_DMA
        DMA_ENABLE_TX;
        SPI_WRITE_BYTE(SPI_ASYNC_INT, *(uint8_t*)async_int_tx_buffer); // write the frist byte to trigger the DMA (TXE)
#else
        uint16_t to_transmit = *num_bytes;
        *num_bytes = 0;
        while ((*num_bytes) < to_transmit) {
            SPI_TRANSMIT_BYTE(SPI_ASYNC_INT, *data);
            data++;
            (*num_bytes)++;
            if (!ASYNC_INT_ACK_STATUS) {
                // aborted
                DEBUG_PRINT_WARNING("transfer aborted by ADI!");
                return;
            }
        }
        DEBUG_PRINT_VERBOSE("message written to ADI (%d bytes)", to_transmit);
#endif // ASYNC_INT_USE_DMA

    } else if (STATE_READ == async_int_state) {
#ifdef ASYNC_INT_USE_DMA
        DMA_ENABLE_RX; 
        DMA_ENABLE_TX;
        SPI_WRITE_BYTE(SPI_ASYNC_INT, *(uint8_t*)async_int_tx_buffer); // write the frist byte to trigger the DMA (TXE)
#else
        *num_bytes = 0;
        // first, clear the RX buffer
        SPI_CLEAR_RXBUF(SPI_ASYNC_INT);
      #ifdef SPI_FAST_READ
        SPI_TRANSMIT_BYTE(SPI_ASYNC_INT, 0x00);            // transmit 1 byte ahead (faster read speed!) 
      #endif
        while ((*num_bytes < MESSAGE_SIZE) && ASYNC_INT_ACK_STATUS) {
            SPI_TRANSMIT_BYTE(SPI_ASYNC_INT, 0x00);        // dummy write to generate the clock
            SPI_RECEIVE_BYTE(SPI_ASYNC_INT, *data);
            data++;
            (*num_bytes)++;
        }
        // how many bytes received?
        DEBUG_PRINT_VERBOSE("message read from ADI (%d bytes)", *num_bytes);
#endif // ASYNC_INT_USE_DMA
    }
}



ISR(PORT2, port2_interrupt) {
 
    //ENERGEST_ON(ENERGEST_TYPE_CPU);
        
    if (PIN_IFG(AI_CTRL_IND)) {
        DEBUG_PRINT_VERBOSE("port 2 interrupt: IND pin");  
        PIN_IES_TOGGLE(AI_CTRL_IND);
        PIN_CLEAR_IFG(AI_CTRL_IND);
    } else if (PIN_IFG(AI_CTRL_ACK)) {
        DEBUG_PRINT_VERBOSE("port 2 interrupt: ACK pin");  
        PIN_CLEAR_IFG(AI_CTRL_ACK);
        if (STATE_IDLE != async_int_state) {
            PIN_IES_TOGGLE(AI_CTRL_ACK);
            //rtimer_stop(ASYNC_INT_RTIMER_ID);
            if (PIN_GET_INPUT_BIT(AI_CTRL_ACK))
            {
                if (STATE_PREPWRITE == async_int_state) {
                    async_int_state = STATE_WRITE;
                } else if (STATE_PREPREAD == async_int_state) {
                    async_int_state = STATE_READ; 
                }
            } else {
                // abort or transmission complete!
                if (STATE_READ == async_int_state) {
                    uint16_t rcv_bytes = (DMA_REMAINING_BYTES == (MESSAGE_SIZE - 1) ? MESSAGE_SIZE : (MESSAGE_SIZE - DMA_REMAINING_BYTES));
                    async_int_release();               
                    DEBUG_PRINT_VERBOSE("Async interface transfer complete (%d bytes received)", rcv_bytes);
                }
            }
        } else {
            DEBUG_PRINT_VERBOSE("async interface not in IDLE state"); 
        }
    }
    
    //ENERGEST_OFF(ENERGEST_TYPE_CPU);
}


#endif /* HAS_ASYNC_INT */
