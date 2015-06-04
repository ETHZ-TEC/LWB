
/**
 * Message manager, buffers incoming and outgoing messages. Informs the other component (A) when the local buffer reaches a certain fill level.
 * After the reception of a stop message, the component may still send messages (especially control messages), but the timely read-out by the other component is not guaranteed anymore.
 * @note the message buffer should be dimensioned to be twice the size of a queue of the ADI
 * @remark A deadlock will not occur under the assumption that all messages are delivered (no loss) and if the power goes out, both components are powered off.
 */

#include "s-lwb.h"

                   
MEMBX(in_msg_queue, sizeof(message_t), N_BUFFERED_IN_MSG);
MEMBX(out_msg_queue, sizeof(message_t), N_BUFFERED_OUT_MSG);

static uint8_t stop_msg_rcvd = 0;
static uint8_t stop_msg_sent = 0;

               
/**
 * @brief calculate a CRC16 checksum
 * @param data start address of the data
 * @param num_bytes data size in bytes
 * @return the CRC checksum
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
    msg.header.stream_id = 0;
    SET_CTRL_MSG(&msg);
    cmd->code            = CMD_CODE_TIMESTAMP;
    msg.header.len       = 10;
    msg.header.recipient = node_id;
    msg.header.crc       = 0;
    mm_put(&msg);
    
    ASYNC_INT_TIMEREQ_DISABLE;      // disable the interrupt until the message has been processed (to prevent accumulation of timestamp messages
}
#endif // ASYNC_INT_TIMEREQ_POLLING  


/**
 * @brief read and process all messages in the asynchronous data interface queue (fill the buffer)
 * @param buffer provide a data buffer that will be used by the function to load and process the messages
 */
void mm_fill(uint8_t* buffer) {
    // check if new data is available on the asynchronous interface
    if (ASYNC_INT_DATA_AVAILABLE) {
        message_t* msg = (message_t*)buffer;
        uint16_t rcvd_bytes = 0,
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
                    if (cmd->code == CMD_CODE_SRQ) {            // request a new stream                    
                        msg->header.stream_id &= ~0x80;         // clear the control flag
                        stream_add(msg->header.stream_id, cmd->param[0], cmd->param[1]);
                    } else if (cmd->code == CMD_CODE_LOCK) {
                        stop_msg_rcvd = 1; // we are not allowed to send more packets to the ADI
                        DEBUG_PRINT_VERBOSE("stop message received (queue jammed)");
                    } else if (cmd->code == CMD_CODE_UNLOCK) {
                        stop_msg_rcvd = 0; // we can resume sending packets to the ADI
                        DEBUG_PRINT_VERBOSE("resume message received");
                    } else if (cmd->code == CMD_CODE_RESETSTATS) {
                        stats_reset();                        
                        DEBUG_PRINT_VERBOSE("stats reset");
                    } else {
                        DEBUG_PRINT_VERBOSE("unknown command");
                    }
                }
            } else {
                // write the message to the local external memory
                next = membx_alloc(&out_msg_queue);
                if (XMEM_INVALID_ADDR != next) {
                    fram_write(next, msg->header.len + MESSAGE_HEADER_SIZE, (uint8_t*)buffer);
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
 * @return one if successful, zero otherwise
 */
uint8_t mm_put(message_t* msg) {
    // store the packet in the local external memory
    uint32_t next = membx_alloc(&in_msg_queue);
    if (XMEM_INVALID_ADDR != next) {
        if (msg->header.crc == 0 && msg->header.len != 0) {
            msg->header.crc = calc_crc16(msg->payload, msg->header.len);
        }
        return fram_write(next, msg->header.len + MESSAGE_HEADER_SIZE, (uint8_t*)msg);
    }
    DEBUG_PRINT_WARNING("out of memory, message dropped");
    return 0;
}


/**
 * @brief prepares a message for sending over the LWB
 * Loads the next packet from the external memory (outgoing queue towards the LWB)
 * @param msg output buffer to hold the message
 * @return the message size or zero (in case there was no message to read or an error had occurred)
 */
uint8_t mm_get(message_t* msg) {
    static uint16_t last_idx = 0;
    uint32_t next = membx_get_next(&out_msg_queue, last_idx);
    if (XMEM_INVALID_ADDR != next && fram_read(next, MESSAGE_SIZE, (uint8_t*)msg)) {     // non-empty memory block found and read operation successful?
        membx_free(&out_msg_queue, next);
        //DEBUG_PRINT_VERBOSE("message loaded from memory %lu (l=%d)", next, msg->header.len);
        last_idx++;        
        if (last_idx == out_msg_queue.num) {
            last_idx = 0;
        }
        return msg->header.len + MESSAGE_HEADER_SIZE;
    } 
    // no data to read
    return 0;
}


/**
 * @brief find out the buffer fill level
 * @return the number of messages in the buffer to be sent over the LWB
 */
uint16_t mm_status(void) {
    return out_msg_queue.n_alloc;    
}


/**
 * @brief write pending messages to the asynchronous data interface (flush out all messages from the buffer)
 * @param buffer a data buffer to be used by the function as temporary storage
 */
void mm_flush(uint8_t* buffer) {
    message_t* msg = (message_t*)buffer;
    uint8_t count = 0;
    static uint16_t start_idx = 0;     // improves fairness
    uint16_t msg_len = 0;
    
    // check the fill level of the internal buffer (outgoing messages)
    if (!stop_msg_sent && (out_msg_queue.n_alloc >= (N_BUFFERED_OUT_MSG / 2))) {
        // send a "stop" command
        msg->header.len = 1;
        msg->header.stream_id = 0x80;
        msg->header.recipient = node_id;
        msg->payload[0] = CMD_CODE_LOCK;
        msg_len = MESSAGE_HEADER_SIZE + 1;
        ASYNC_INT_WRITE(buffer, msg_len);
        if (msg_len) {
            // write was successful
            stop_msg_sent = 1;
            DEBUG_PRINT_INFO("stop message sent");
        } else {
            DEBUG_PRINT_WARNING("failed to send stop message");     // this should never happen
        }
    } else if (stop_msg_sent && (out_msg_queue.n_alloc < (N_BUFFERED_OUT_MSG / 2))) {
        // send a "resume" command
        msg->header.len = 1;
        msg->header.stream_id = 0x80;
        msg->header.recipient = node_id;
        msg->payload[0] = CMD_CODE_UNLOCK;
        msg_len = MESSAGE_HEADER_SIZE + 1;
        ASYNC_INT_WRITE(buffer, msg_len);
        if (msg_len) {
            // write was successful
            stop_msg_sent = 0;
            DEBUG_PRINT_INFO("stop message sent");
        } else {
            DEBUG_PRINT_WARNING("failed to send stop message");     // this should never happen
        }
    }
    
    if (!stop_msg_rcvd) {    
        // go through all buffered messages
        uint32_t addr = XMEM_INVALID_ADDR;
        uint16_t i = start_idx;  
        uint8_t bit;
        do {        // loop through all data units in this memory block
            bit = (1 << (i & 0x07));
            if (in_msg_queue.count[i >> 3] & bit) {     // bit set?
                addr = in_msg_queue.mem + ((uint32_t)i * (uint32_t)in_msg_queue.size);
                fram_read(addr, MESSAGE_SIZE, (uint8_t*)buffer);
                // write to the ADI until queue full or all messages written
                msg_len = msg->header.len + MESSAGE_HEADER_SIZE;
                ASYNC_INT_WRITE(buffer, msg_len);
                if (msg_len) {
                    // write was successful
                    membx_free(&in_msg_queue, addr);
                    count++;
                } else {
                    DEBUG_PRINT_VERBOSE("write to ADI failed (queue full?)");
                    break;      // do not continue
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
            DEBUG_PRINT_INFO("%d message(s) written to ADI", count);
        }
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
    stop_msg_rcvd = 0;
    stop_msg_sent = 0;
}

