#ifndef __LOG_H__
#define __LOG_H__


#define LOG_MSG_SIZE    16      /* size in bytes */


/* leave the option to define the resolution of the timestamp, e.g. seconds, milliseconds or something else, maybe even the 'real' time */


// message severity levels
enum {
    LOG_MSG_INFO = 0,  
    LOG_MSG_WARNING = 1,
    LOG_MSG_ERROR = 2,
    LOG_MSG_CRITICAL = 3,
    LOG_MSG_DISASTER = 4,
    NUM_OF_LOG_MSG_LEVELS
} log_msg_level_t;



/* 
 * 16-bit error code, first 3 bit determine the message level
 */
#define LOG_MSG_BATTERY         0x0001

#define LOG_MSG_LWB_NO_COMM     0x2001


// 1 message is 16 bytes
#define LOG_MSG_HEADER_SIZE     8       /* size in bytes */
typedef struct log_msg {
    uint16_t code;
    uint16_t crc;
    uint32_t timestamp;
    char     data[LOG_MSG_SIZE - LOG_MSG_HEADER_SIZE];  /* buffer for additional info */
} log_msg_t;


void log_init(void);
void log_add(uint16_t code, char* data);

/*
 * @brief       get the last error message
 * @param[out]  msg a pointer to a valid memory block where the message will be stored
 */
void log_get_last(log_msg_t* msg);  
void log_remove_last(void);

/*
 * @brief       output all log messages over UART
 */
void log_print_uart(void);


#endif /* __LOG_H__ */
