
/** 
 * @file
 * @ingroup LWB
 * @brief   Message manager, handles the buffering of incoming and outgoing messages.
 * @author  rdaforno
 * @note    there are two queues: one for the incoming messages from the LWB bus and one for the outgoing messages to the LWB
 */

#ifndef MM_H
#define MM_H


/**
 * @brief command codes
 */
typedef enum {
    CMD_CODE_STATUS = 0,
    CMD_CODE_LOCK,          // do not allow the other component to send more packets over the ADI (pause, jammed)
    CMD_CODE_UNLOCK,        // allow the other component to resume sending packets over the ADI (resume)
    CMD_CODE_TIMESTAMP,     // timestamp (answer to a timestamp request)
    CMD_CODE_SRQ,           // stream request
    CMD_CODE_SACK,          // stream ACK
    CMD_CODE_SDEL,          // stream deleted
    CMD_CODE_DRQ,           // data request (request a specific data packet)
    CMD_CODE_DACK,          // acknowledge data packets
    CMD_CODE_RESETSTATS,
    NUM_OF_CMD_CODES
} cmd_code_t;



#define IS_CTRL_MSG(m)      ( ((message_t*)m)->header.stream_id & 0x80 )
#define IS_DATA_MSG(m)      ( !(((message_t*)m)->header.stream_id & 0x80) )
#define SET_CTRL_MSG(m)     ( ((message_t*)m)->header.stream_id |= 0x80 )



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
 * @brief max. size of a command packet
 */
#define CMD_PKT_SIZE_MAX   ( MESSAGE_SIZE - MESSAGE_HEADER_SIZE )
/**
 * @brief struct of a command on the application layer
 */
typedef struct {
    uint8_t  code;                            // command code
    uint8_t  param[CMD_PKT_SIZE_MAX - 1];     // parameters 
} command_t;


uint16_t calc_crc16(const uint8_t* data, uint8_t num_bytes);

void mm_init(void);
uint16_t mm_status(void);
uint8_t mm_get(message_t* msg);     // renamed from mm_out_queue_get
uint8_t mm_put(message_t* msg);     // renamed from mm_in_queue_put
void mm_flush(uint8_t* buffer);     // renamed from mm_in_queue_flush
void mm_fill(uint8_t* buffer);      // renamed from mm_out_queue_fill


#endif
