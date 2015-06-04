
// ------------------------------------------------------------------------------------------------------
// mm.h
// ------------------------------------------------------------------------------------------------------


/** 
 * @file
 * @ingroup LWB
 * @brief   Message manager, handles the buffering of incoming and outgoing messages from and to the ADI and the LWB
 * @author  rdaforno
 */

#ifndef MM_H
#define MM_H


#define MESSAGE_HEADER_SIZE     8
/**
 * @brief packet format of messages sent to and read from the asynchronous data interface
 */
typedef struct {
    struct {
        // be aware of structure alignment (8-bit are aligned to 8-bit, 16 to 16 etc.)
        uint16_t recipient; // target node
        uint8_t  stream_id; // message type and connection ID (used as stream ID in LWB); first bit is msg type
        uint8_t  len;       // payload length in bytes
        uint16_t pkt_id;    // packet ID
        uint16_t crc;       // CRC16 checksum
    } header;
    uint8_t payload[MESSAGE_SIZE - MESSAGE_HEADER_SIZE];
} message_t;


/**
 * @brief command codes (application layer)
 */
typedef enum {
    CMD_CODE_STATUS = 0,
    CMD_CODE_TOKEN,         // communicate the available buffer space for messages
    CMD_CODE_CREDIT,        // send credit to the other processor
    CMD_CODE_TIMESTAMP,     // timestamp (answer to a timestamp request)
    CMD_CODE_SRQ,           // stream request
    CMD_CODE_SACK,          // stream ACK
    CMD_CODE_SDEL,          // stream deleted
    CMD_CODE_DRQ,           // data request (request a specific data packet)
    CMD_CODE_DACK,          // acknowledge data packets
    NUM_OF_CMD_CODES
} cmd_code_t;


void mm_init(void);
uint8_t mm_out_queue_get(message_t* msg);
void mm_in_queue_flush(uint8_t* buffer);
void mm_in_queue_put(message_t* msg);
void mm_out_queue_fill(uint8_t* buffer);




#endif


// ------------------------------------------------------------------------------------------------------
// mm.c
// ------------------------------------------------------------------------------------------------------


#include "s-lwb.h"


static uint8_t token     = 0,   // credit
               n_dealloc = 0;
                   
MEMBX(in_msg_queue, sizeof(message_t), N_BUFFERED_IN_MSG);
MEMBX(out_msg_queue, sizeof(message_t), N_BUFFERED_OUT_MSG);

               
/**
 * @brief read and process all messages in the asynchronous data interface queue
 * @param buffer provide a data buffer that will be used by the function to load and process the messages
 */
uint16_t calc_crc16(const uint8_t* data, uint8_t num_bytes) {
	uint16_t crc  = 0,
             mask = 0xa001;
	while (num_bytes) {
		uint8_t ch = *data;
		int8_t bit = 0;
		while (bit < 8) {
            if ((crc & 1) ^ (ch & 1)) {
                crc = (crc >> 1) ^ mask;
            } else {
				crc >>= 1;
			}
			ch >>= 1; 
			bit += 1;
		}
        data++;
		num_bytes--;
	}
	return crc;    
    /*CRCINIRES = 0xffff;   // set seed (init value)
    while (num_bytes) {    
        CRCDI = *data;
        data++;
        num_bytes--;
    }
    return CRCINIRES;*/
}


#ifndef ASYNC_INT_TIMEREQ_POLLING             
/**
 * @brief process the timestamp request in the interrupt context (ISR)
 */
void process_timestamp_request(void) {
    message_t msg;
    command_t* cmd     = (command_t*)msg.payload;
    uint16_t ccr       = TA1CCR0;
    rtimer_clock_t* sw = (rtimer_clock_t*)&cmd->param[1];
    *sw = ta0_sw_ext;
    // note: timer update ISR will be executed before this ISR, therefore the sw extension might be invalid
    if ((TA1R < ccr) && !(TA0CTL & TAIFG)) { 
        // overflow has occurred after the capture of the timer value and before entering this ISR, and therefore the counter value is not correct
        (*sw)--;
        // Explanation - There are 4 possible cases:
        // - no overflow occurred, TAIFG not set and counter value (TA1R) > CCR value -> nothing to adjust
        // - overflow occurred before entering this ISR, Timer update ISR gets executed before this ISR is entered, TAIFG not set and counter value < CCR value -> subtract 1
        // - overflow occurred after entering this ISR and before counter value was read, TAIFG is set and counter value < CCR value -> nothing to adjust
        // - overflow occurred after entering this ISR and after the counter value was read, TAIFG is set and counter value > CCR value -> nothing to adjust
    }
    *sw = (*sw << 16) | ccr;
    
    DEBUG_PRINT_INFO("timestamp: %llu", *sw);
    
    // compose and buffer the message
    SET_CTRL_MSG(&msg);
    cmd->code            = CMD_CODE_TIMESTAMP;
    msg.header.len       = 10;
    msg.header.recipient = node_id;
    msg.header.crc       = 0;
    mm_in_queue_put(&msg);
    
    ASYNC_INT_TIMEREQ_DISABLE;      // disable the interrupt until the message has been processed (to prevent accumulation of timestamp messages
}
#endif // ASYNC_INT_TIMEREQ_POLLING  


/**
 * @brief read and process all messages in the asynchronous data interface queue
 * @param buffer provide a data buffer that will be used by the function to load and process the messages
 */
void mm_out_queue_fill(uint8_t* buffer) {
    // check if new data is available on the asynchronous interface
    if (ASYNC_INT_DATA_AVAILABLE) {
        message_t* msg = (message_t*)buffer;
        uint8_t rcvd_bytes = 0,
                count = 0;
        uint32_t next;
        // read all messages from the queue (important: read the whole queue before writing back into the queue)
        do {
            ASYNC_INT_READ(buffer, rcvd_bytes); // read one message from the asynchronous interface
            (void)rcvd_bytes;                   // get rid of compiler warning
            //exess bytes: rcvd_bytes - (msg->header.len + MESSAGE_HEADER_SIZE);                    
            // process the message
            if (msg->header.recipient == node_id) {
                // message is targeted at this node
                if (IS_CTRL_MSG(msg)) {
                    DEBUG_PRINT_VERBOSE("control message received");
                    command_t* cmd = (command_t*)msg->payload;
                    if (cmd->code == CMD_CODE_TOKEN) {
                        if (token != 0) {
                            DEBUG_PRINT_INFO("invalid token count detected (rebooted?)");
                        }
                        token = *cmd->param;
                        DEBUG_PRINT_INFO("token: %d", token);
                    } else if (cmd->code == CMD_CODE_CREDIT) {
                        token += *cmd->param;
                        DEBUG_PRINT_INFO("credit: %d/%d", *cmd->param, token);
                    } else if (cmd->code == CMD_CODE_SRQ) {     // request a new stream                    
                        msg->header.stream_id &= ~0x80;         // clear the control flag
                        stream_add(msg->header.stream_id, cmd->param[0], cmd->param[1]);
                    } 
                }
            } else {
                // write the message to the local external memory
                next = membx_alloc(&out_msg_queue);
                if (XMEM_INVALID_ADDR != next) {
                    fram_write(next, msg->header.len + MESSAGE_HEADER_SIZE, buffer);
                    //DEBUG_PRINT_VERBOSE("message buffered (l=%d ofs=%lu)", msg->header.len, next);
                } else {
                    DEBUG_PRINT_WARNING("out of memory, message dropped");  // should never happen
                }
            }
            count++;
        } while (ASYNC_INT_DATA_AVAILABLE);
        
        DEBUG_PRINT_INFO("%d message(s) read from ADI, %d buffered", count, out_msg_queue.n_alloc);
    }
}


/**
 * @brief buffer a message in the outgoing queue towards the ADI
 * @param msg the message to be written into the queue
 */
void mm_in_queue_put(message_t* msg) {
    // store the packet in the local external memory
    uint32_t next = membx_alloc(&in_msg_queue);
    if (XMEM_INVALID_ADDR != next) {
        if (msg->header.crc == 0 && msg->header.len != 0) {
            msg->header.crc = calc_crc16(msg->payload, msg->header.len);
        }
        fram_write(next, msg->header.len + MESSAGE_HEADER_SIZE, (uint8_t*)msg);
    } else {
        DEBUG_PRINT_WARNING("out of memory, message dropped");
    }
}


/**
 * @brief prepares a message for sending over the LWB
 * Loads the next packet from the external memory (outgoing queue towards the LWB)
 * @param msg output buffer to hold the message
 * @return the message size (or zero in case there is no message to send)
 */
uint8_t mm_out_queue_get(message_t* msg) {
    static uint16_t last_idx = 0;
    uint32_t next = membx_get_next(&out_msg_queue, last_idx);
    if (XMEM_INVALID_ADDR != next) {            // non-empty memory block found?
        fram_read(next, MESSAGE_SIZE, msg);     // load the packet from the external memory
        membx_free(&out_msg_queue, next);
        //DEBUG_PRINT_VERBOSE("message loaded from memory %lu (l=%d)", next, msg->header.len);
        n_dealloc++;
        last_idx++;        
        if (last_idx == out_msg_queue.num) {
            last_idx = 0;
        }
        return msg->header.len + MESSAGE_HEADER_SIZE;
    } 
    // no data to send
    return 0;
}


/**
 * @brief write pending messages to the asynchronous data interface (if enough credit available) and send new credit
 * @param buffer a data buffer to be used by the function as temporary storage
 */
void mm_in_queue_flush(uint8_t* buffer) {
    message_t* msg = (message_t*)buffer;
    uint8_t count = 0;
    static uint16_t start_idx = 0;     // improves fairness
    
    // go through all buffered messages
    uint32_t addr = XMEM_INVALID_ADDR;
    uint16_t i = start_idx;  
    uint8_t bit;
    do {        // loop through all data units in this memory block
        bit = (1 << (i & 0x07));
        if (in_msg_queue.count[i >> 3] & bit) {     // bit set?
            addr = in_msg_queue.mem + ((uint32_t)i * (uint32_t)in_msg_queue.size);
            fram_read(addr, MESSAGE_SIZE, buffer);
            if (IS_CTRL_MSG(msg) || token) {        // token only required for data messages
                membx_free(&in_msg_queue, addr);
                ASYNC_INT_WRITE(buffer, msg->header.len + MESSAGE_HEADER_SIZE);
                if (!IS_CTRL_MSG(msg)) {
                    token--;             
                }
                count++;
            }
        }
        i++;
        if (i == in_msg_queue.num) {
            i = 0;
        }
    } while (i != start_idx);
    
    start_idx++;
    if (start_idx == in_msg_queue.num) {
        start_idx = 0;
    }
    if (count) {
        DEBUG_PRINT_INFO("%d message(s) written to ADI (credit: %d)", count, token);
    }
        
    // send new credit: notify the appl. processor about the remaining empty buffer space (no token required to send this message)
    if (n_dealloc) {
        command_t* cmd = (command_t*)msg->payload;
        if (N_BUFFERED_IN_MSG == n_dealloc) {
            cmd->code = CMD_CODE_TOKEN;
        } else {
            cmd->code = CMD_CODE_CREDIT;
        }
        *cmd->param = n_dealloc;
        SET_CTRL_MSG(msg);
        msg->header.len = 2;
        msg->header.crc = calc_crc16(msg->payload, 2);
        ASYNC_INT_WRITE((uint8_t*)buffer, msg->header.len + MESSAGE_HEADER_SIZE);
        n_dealloc = 0;
        DEBUG_PRINT_VERBOSE("token sent");
    }
    
#ifndef ASYNC_INT_TIMEREQ_POLLING
    ASYNC_INT_TIMEREQ_ENABLE;   // re-enable the timestamp request interrupt
#endif
}

        
/**
 * @brief initialize the message manager (including all required components such as the FRAM or the ADI)
 */
void mm_init(void) {
    // initialize the FRAM if not already done
    fram_init();
    
    // asynchronous data interface
#ifndef ASYNC_INT_TIMEREQ_POLLING
    async_int_init(process_timestamp_request);
#else
    async_int_init();
#endif
    
    // allocate memory for buffering messages
    membx_init(&in_msg_queue);
    membx_init(&out_msg_queue);
    
    n_dealloc = N_BUFFERED_IN_MSG; // initial number of token to send
}


// ------------------------------------------------------------------------------------------------------

