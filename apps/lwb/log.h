/*
 * Copyright (c) 2015, Swiss Federal Institute of Technology (ETH Zurich).
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Author:  Reto Da Forno
 */

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
 * TODO
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
