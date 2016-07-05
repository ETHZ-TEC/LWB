/*
 * Copyright (c) 2016, Swiss Federal Institute of Technology (ETH Zurich).
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

/**
 * @addtogroup  dev
 * @{
 *
 * @defgroup    log logging
 * @{
 * 
 * @file
 *
 * @brief Event logging in non-volatile memory.
 */

#ifndef __LOG_H__
#define __LOG_H__

/* logging into non-volatile memory */

/* disabled by default */
#ifndef LOG_CONF_ON
#define LOG_CONF_ON                     0
#endif /* LOG_CONF_ON */

#ifndef LOG_CONF_START_ADDR
/* by default, start at byte 8192 */
#define LOG_CONF_START_ADDR             0x00002000
#endif /* LOG_CONF_START_ADDR */

#ifndef LOG_CONF_NUM_ENTRIES
#define LOG_CONF_NUM_ENTRIES            10000
#endif /* LOG_CONF_NUM_ENTRIES */

/* error check */
#if LOG_CONF_NUM_ENTRIES == 0
#error "LOG_CONF_NUM_ENTRIES must not be 0"
#endif

/* overwrite old entries if log full? */ 
#define LOG_CONF_ROLLOVER               1       
/* include a 64-bit timestamp in each log entry? */
#define LOG_CONF_INCL_TIMESTAMP         0
#define LOG_CONF_DATA_LEN               4       /* byte (must be even) */

/* keep the length constant for all entries in the log */
#if LOG_CONF_INCL_TIMESTAMP
  #define LOG_ENTRY_HDR_LEN             10
#else
  #define LOG_ENTRY_HDR_LEN             2
#endif /* LOG_ENTRY_HDR_LEN */
#define LOG_ENTRY_LEN                   (LOG_ENTRY_HDR_LEN + LOG_CONF_DATA_LEN)


/* meta data */
#define LOG_META_LEN                    14
typedef struct {
  uint32_t      n_entries;
  uint32_t      next_write;
  uint32_t      next_read;
  uint16_t      crc;
} log_t;


/* predefined event codes (16 bits = up to 65535 different events/types) */

typedef enum {
  LOG_EVENT_UNKNOWN = 0x0000,
  LOG_EVENT_INFO = 0x0001,
  LOG_EVENT_WARNING,
  LOG_EVENT_ERROR,   
  LOG_EVENT_LOG_INIT,
  LOG_EVENT_LWB_BOOTSTRAP,
  LOG_EVENT_LWB_SCHED_MISSED,
  LOG_EVENT_LWB_COMM_TIMEOUT,
  LOG_EVENT_LWB_DATA_SENT,
  LOG_EVENT_CONTEXT_SWITCH,
    
} log_event_t;


/* packet format */

typedef struct {
#if LOG_CONF_INCL_TIMESTAMP
  uint64_t      timestamp;
#endif /* LOG_CONF_INCL_TIMESTAMP */
  log_event_t   event : 16;  /* msg type */
  uint16_t      data[LOG_CONF_DATA_LEN / 2];
} log_entry_t;


/* macros */

#if LOG_CONF_ON
#define LOG(code, val1, val2)   log_write_simple(code, val1, val2)
#else /* LOG_CONF_ON */
#define LOG(code, val1, val2)   
#endif /* LOG_CONF_ON */



/* prototypes */

/**
 * @brief prepare the log
 * @note checks the state and consistency of the memory and prints out some
 * stats
 */
uint8_t log_init(void);

/**
 * @brief clear the entire log
 */
void log_clear(void);

/**
 * @brief print log entries to UART
 * @param num_msg the number of messages to print
 * (at most the number of available messages will be printed)
 */
void log_print(uint16_t num_msg);

/**
 * @brief read one entry from the log
 * @param out_entry output buffer for the event to read
 * @return 1 if successful, 0 otherwise
 */
uint8_t log_read(log_entry_t* out_entry); 

/**
 * @brief write one entry to the log
 * @param entry the event to be written
 * @return 1 if successful, 0 otherwise
 */
uint8_t log_write(const log_entry_t* entry);

/**
 * @brief write one entry to the log
 * @note same as log_write, but arguments of the struct are passed directly
 */
uint8_t log_write_simple(log_event_t event, uint16_t val1, uint16_t val2);



#endif /* __LOG_H__ */

/**
 * @}
 * @}
 */