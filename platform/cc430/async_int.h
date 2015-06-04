/**
 * @file
 * @author rdaforno
 * @remark The data transfer over the SPI can either be synchronous (blocking, polling/busy wait) or asynchronous (interrupt/DMA-driven).
 */
  
#ifndef __ASYNC_INT_H__
#define __ASYNC_INT_H__


#define ASYNC_INT_IS_TIMEREQ_ENABLED        ( (TA1CCTL0 & CCIE) > 0 )
#define ASYNC_INT_TIMEREQ_ENABLE            ( TA1CCTL0 |= CCIE )
#define ASYNC_INT_TIMEREQ_DISABLE           ( TA1CCTL0 &= ~CCIE )


#ifdef HAS_ASYNC_INT        // async interface not available on Olimex board


#define ASYNC_INT_DATA_AVAILABLE            PIN_GET_INPUT_BIT(AI_CTRL_IND)
#define ASYNC_INT_ACK_STATUS                PIN_GET_INPUT_BIT(AI_CTRL_ACK)
//#define ASYNC_INT_RTIMER_ID                 1             // -> removed, not needed after all

#define ASYNC_INT_WAIT_COMPLETED            while (STATE_IDLE != async_int_state)


#ifdef ASYNC_INT_USE_DMA
    /**
     * @brief writes one message (num_bytes bytes) to the asynchronous interface (non-blocking)
     */
    #define ASYNC_INT_WRITE(num_bytes)      \
        {\
            if (async_int_req_op(OP_WRITE, num_bytes)) {\
                async_int_start_op();\
            }\
        }
        
    /**
     * @brief reads one message from the asynchronous interface (non-blocking)
     */
    #define ASYNC_INT_READ  \
        {\
            if (async_int_req_op(OP_READ, MESSAGE_SIZE)) {\
                async_int_start_op();\
            }\
        }
#else
    /**
     * @brief writes one message (num_bytes bytes of the buffer data) to the asynchronous interface (blocking call)
     */
    #define ASYNC_INT_WRITE(data, num_bytes) \
        {\
            if (async_int_req_op(OP_WRITE)) {\
                async_int_start_op(data, &num_bytes);\
                async_int_release();\
            }\
        } 
        
    /**
     * @brief reads one message from the asynchronous interface (blocking call)
     */
    #define ASYNC_INT_READ(data, rcvd_bytes)  \
        {\
            if (async_int_req_op(OP_READ)) {\
                async_int_start_op(data, &rcvd_bytes);\
                async_int_release();\
            }\
        }
#endif // ASYNC_INT_USE_DMA



/**
 * @brief all the possible states of the finite state machine that controls the interaction with the asynchronous interface
 */
typedef enum {
    STATE_IDLE = 0,
    STATE_PREPREAD,
    STATE_PREPWRITE,
    STATE_READ,
    STATE_WRITE,
    NUM_OF_STATES
} ai_state;

/**
 * @brief the two possible data operations: read or write
 */
typedef enum {
    OP_READ = 0,
    OP_WRITE,
    NUM_OF_OPS
} op_mode;



extern volatile ai_state async_int_state;


#ifdef ASYNC_INT_USE_DMA

  #ifdef ASYNC_INT_TIMEREQ_POLLING
    void async_int_init(uint16_t rx_buffer_addr, uint16_t tx_buffer_addr);
  #else // ASYNC_INT_TIMEREQ_POLLING
    void async_int_init(uint16_t rx_buffer_addr, uint16_t tx_buffer_addr, rtimer_ta1_callback_t func);  
  #endif // ASYNC_INT_TIMEREQ_POLLING
    uint8_t async_int_req_op(op_mode mode, uint16_t num_bytes);
    void async_int_start_op(void);
    
#else // ASYNC_INT_USE_DMA
    
  #ifdef ASYNC_INT_TIMEREQ_POLLING
    void async_int_init(void);
  #else  
    void async_int_init(rtimer_ta1_callback_t func);
  #endif
    uint8_t async_int_req_op(op_mode mode);
    void async_int_start_op(uint8_t* data, uint16_t* num_bytes);
    
#endif // ASYNC_INT_USE_DMA
    
#ifdef ASYNC_INT_TIMEREQ_POLLING
    uint8_t async_int_timereq_pending(uint8_t* out_buffer);
#endif // ASYNC_INT_TIMEREQ_POLLING
    
void async_int_release(void);



#endif /* HAS_ASYNC_INT */

#endif /* __ASYNC_INT_H__ */
