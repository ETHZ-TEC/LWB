/*
 * Copyright (c) 2018, Swiss Federal Institute of Technology (ETH Zurich).
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
 * @addtogroup  Dev
 * @{
 *
 * @defgroup    debug-print Debug print task
 * @{
 *
 * @file
 * 
 * @brief Debug print task (print out debug messages over UART)
 */

#ifndef __DEBUG_PRINT_H__
#define __DEBUG_PRINT_H__

/* necessary to override the default settings in platform.h */
#include "platform.h"

/* enabled by default */
#ifndef DEBUG_PRINT_CONF_ON
#define DEBUG_PRINT_CONF_ON             1
#endif /* DEBUG_PRINT_CONF_ON */

#ifndef DEBUG_PRINT_CONF_MSG_LEN        /* max. num of chars per msg */
#define DEBUG_PRINT_CONF_MSG_LEN        100
#endif /* DEBUG_PRINT_CONF_MSG_LEN */

#ifndef DEBUG_PRINT_CONF_BUFFER_SIZE    /* total buffer size in bytes */
#define DEBUG_PRINT_CONF_BUFFER_SIZE    512
#endif /* DEBUG_PRINT_CONF_BUFFER_SIZE */

#ifndef DEBUG_PRINT_CONF_USE_XMEM
#define DEBUG_PRINT_CONF_USE_XMEM       0
#endif /* DEBUG_PRINT_CONF_USE_XMEM */

#if DEBUG_PRINT_CONF_USE_XMEM
  #ifndef DEBUG_PRINT_CONF_NUM_MSG
  #define DEBUG_PRINT_CONF_NUM_MSG      10
  #endif /* DEBUG_PRINT_CONF_NUM_MSG */
#else /* DEBUG_PRINT_CONF_USE_XMEM */
  #define DEBUG_PRINT_CONF_NUM_MSG      0   /* not supported */
#endif /* DEBUG_PRINT_CONF_USE_XMEM */

#ifndef DEBUG_PRINT_CONF_LEVEL
#define DEBUG_PRINT_CONF_LEVEL          DEBUG_PRINT_LVL_INFO
#endif /* DEBUG_PRINT_CONF_LEVEL */

#ifndef DEBUG_PRINT_CONF_PRINT_DIRECT   /* print directly, no queuing */
#define DEBUG_PRINT_CONF_PRINT_DIRECT   0
#endif /* DEBUG_PRINT_CONF_PRINT_DIRECT */

/* poll the debug task every time a message is schedule for print-out? */
#ifndef DEBUG_PRINT_CONF_POLL
#define DEBUG_PRINT_CONF_POLL           0    
#endif /* DEBUG_PRINT_CONF_POLL */

#ifndef DEBUG_PRINT_CONF_PRINT_TIMESTAMP
#define DEBUG_PRINT_CONF_PRINT_TIMESTAMP  1
#endif /* DEBUG_PRINT_CONF_PRINT_TIMESTAMP */

#ifndef DEBUG_PRINT_CONF_PRINT_NODEID
#define DEBUG_PRINT_CONF_PRINT_NODEID   0
#endif /* DEBUG_PRINT_CONF_PRINT_NODEID */

#ifndef DEBUG_PRINT_CONF_PRINT_DBGLEVEL
#define DEBUG_PRINT_CONF_PRINT_DBGLEVEL 0
#endif /* DEBUG_PRINT_CONF_PRINT_DBGLEVEL */

#ifndef DEBUG_PRINT_CONF_PRINT_FILENAME
#define DEBUG_PRINT_CONF_PRINT_FILENAME 0
#endif /* DEBUG_PRINT_CONF_PRINT_FILENAME */

#ifndef DEBUG_PRINT_CONF_EOL            /* end of line (newline character) */
#define DEBUG_PRINT_CONF_EOL            "\r\n"
#endif /* DEBUG_PRINT_CONF_EOL */

#if DEBUG_PRINT_CONF_PRINT_FILENAME && !defined(__FILENAME__)
#define __FILENAME__                    (strrchr(__FILE__, '/') ? \
                                         strrchr(__FILE__, '/') + 1 : __FILE__)
/* note: strrchr will be evaluated at compile time and the file name inlined */
#endif /* DEBUG_PRINT_CONF_PRINT_FILE && ... */

/* stack guard to detect stack overflows, recommended value is:
 * SRAM_START + bss section size */
#ifndef DEBUG_CONF_STACK_GUARD
  #ifdef SRAM_END
    #define DEBUG_CONF_STACK_GUARD      (SRAM_END - 0x01ff)
  #else  /* SRAM_END */
    #define DEBUG_CONF_STACK_GUARD      0   /* disable stack guard */
  #endif /* SRAM_END */
#else
  #if DEBUG_CONF_STACK_GUARD && (DEBUG_CONF_STACK_GUARD < SRAM_START || \
                                 DEBUG_CONF_STACK_GUARD > SRAM_END)
    #error "Invalid value for DEBUG_CONF_STACK_GUARD"
  #endif /* if DEBUG_CONF_STACK_GUARD */
#endif /* ifndef DEBUG_CONF_STACK_GUARD */

#ifdef DEBUG_CONF_ISR_IND_PIN
  #define DEBUG_ISR_ENTRY               PIN_SET(DEBUG_CONF_ISR_IND_PIN)
  #define DEBUG_ISR_EXIT                PIN_CLR(DEBUG_CONF_ISR_IND_PIN)
#else /* DEBUG_CONF_ISR_IND_PIN */
  #define DEBUG_ISR_ENTRY
  #define DEBUG_ISR_EXIT
#endif /* DEBUG_CONF_ISR_IND_PIN */

/**
 * @brief set DEBUG_PRINT_DISABLE_CONF_UART to 1 to disable UART after each
 * print out (& re-enable it before each print out)
 */
#ifndef DEBUG_PRINT_CONF_DISABLE_UART
#define DEBUG_PRINT_CONF_DISABLE_UART   1
#endif /* DEBUG_PRINT_CONF_DISABLE_UART */

#ifdef LED_ERROR                        /* don't use LED_ON */
#define DEBUG_PRINT_ERROR_LED_ON        PIN_SET(LED_ERROR)                                        
#else
#define DEBUG_PRINT_ERROR_LED_ON
#endif

/* set debugging level for each module (0 = no debug prints) */
#define DEBUG_PRINT(level, time, title, ...)   \
  if(DEBUG_PRINT_CONF_LEVEL >= level) { \
    DEBUG_PRINT_MSG(time, title, __VA_ARGS__); }

/*
 * severity levels:
 * - VERBOSE: debug information, not important
 * - INFO:    information about the system, might be of interest
 * - WARNING: critical event, something the user should be aware of (e.g.
 *            buffer full)
 * - ERROR:   unexpected event or a (recoverable) failure
 * - FATAL:   unrecoverable failure, system reset required (eg. stack overflow)
 */

#define DEBUG_PRINT_ERROR(...) \
  if(DEBUG_PRINT_CONF_LEVEL >= DEBUG_PRINT_LVL_ERROR) { \
    DEBUG_PRINT_MSG(0, DEBUG_PRINT_LVL_ERROR, __VA_ARGS__); \
    DEBUG_PRINT_ERROR_LED_ON; }
#define DEBUG_PRINT_WARNING(...) \
  if(DEBUG_PRINT_CONF_LEVEL >= DEBUG_PRINT_LVL_WARNING) { \
    DEBUG_PRINT_MSG(0, DEBUG_PRINT_LVL_WARNING, __VA_ARGS__); }
#define DEBUG_PRINT_INFO(...) \
  if(DEBUG_PRINT_CONF_LEVEL >= DEBUG_PRINT_LVL_INFO) { \
    DEBUG_PRINT_MSG(0, DEBUG_PRINT_LVL_INFO, __VA_ARGS__); }
#define DEBUG_PRINT_VERBOSE(...) \
  if(DEBUG_PRINT_CONF_LEVEL >= DEBUG_PRINT_LVL_VERBOSE) { \
    DEBUG_PRINT_MSG(0, DEBUG_PRINT_LVL_VERBOSE, __VA_ARGS__); }
/* always enabled: highest severity level errors that require a reset */
#define DEBUG_PRINT_FATAL(...) {\
  DEBUG_PRINT_MSG_NOW(__VA_ARGS__); \
  PIN_SET(LED_ERROR); \
  __delay_cycles(MCLK_SPEED); \
  PMM_TRIGGER_POR; \
}


/* defines how the debug print function looks like */
#if DEBUG_PRINT_CONF_PRINT_DIRECT
  #define DEBUG_PRINT_FUNCTION(l, msg)   debug_print_msg_now(msg)
#else /* DEBUG_PRINT_CONF_PRINT_DIRECT */
 #if DEBUG_PRINT_CONF_PRINT_FILENAME
  #define DEBUG_PRINT_FUNCTION(l, msg)   debug_print_msg(rtimer_now_lf(), \
                                           l, msg, __FILENAME__, __LINE__) 
 #else /* DEBUG_PRINT_CONF_PRINT_FILENAME */
  #define DEBUG_PRINT_FUNCTION(l, msg)   debug_print_msg(rtimer_now_lf(), \
                                           l, msg)
 #endif /* DEBUG_PRINT_CONF_PRINT_FILENAME */
#endif /* DEBUG_PRINT_CONF_PRINT_DIRECT */


#if DEBUG_PRINT_CONF_ON
  #define DEBUG_PRINT_MSG(t, l, ...) \
    snprintf(debug_print_buffer, DEBUG_PRINT_CONF_MSG_LEN, __VA_ARGS__);\
    DEBUG_PRINT_FUNCTION(l, debug_print_buffer)
  #define DEBUG_PRINT_SIMPLE(s, l) DEBUG_PRINT_FUNCTION(l, s)
  #define DEBUG_PRINT_MSG_NOW(...) \
    snprintf(debug_print_buffer, DEBUG_PRINT_CONF_MSG_LEN, __VA_ARGS__); \
    debug_print_msg_now(debug_print_buffer)
  #define DEBUG_PRINT_SIMPLE_NOW(s)  debug_print_msg_now(s)
#else /* DEBUG_PRINT_CONF_ON */
  #define DEBUG_PRINT_MSG(t, l, ...)
  #define DEBUG_PRINT_MSG_NOW(...) 
  #define DEBUG_PRINT_SIMPLE(s)
  #define DEBUG_PRINT_SIMPLE_NOW(s)
#endif /* DEBUG_PRINT_CONF_ON */

/* immediately print out a debug marker (file name and line) */
#define DEBUG_PRINT_MARKER        \
  snprintf(debug_print_buffer, DEBUG_PRINT_CONF_MSG_LEN, "%s %u", \
           __FILENAME__, __LINE__); \
  debug_print_msg_now(debug_print_buffer)


/* debug levels (severity level) */
typedef enum {  
  DEBUG_PRINT_LVL_QUIET = 0,     /* no logging */
  DEBUG_PRINT_LVL_ERROR = 1,     /* alert, something went wrong */
  DEBUG_PRINT_LVL_WARNING = 2,   /* something unexpected happend */
  DEBUG_PRINT_LVL_INFO = 3,      /* status message */
  DEBUG_PRINT_LVL_VERBOSE = 4,   /* debug output */
  NUM_OF_DEBUG_PRINT_LEVELS
} debug_level_t;

/* +1 for the trailing \0 character */
extern char debug_print_buffer[DEBUG_PRINT_CONF_MSG_LEN];   

typedef struct debug_print_t {
  struct debug_print_t *next;
  uint32_t time;
  uint8_t level;
  char content[DEBUG_PRINT_CONF_MSG_LEN];
} debug_print_t;

/**
 * @brief start the debug print process 
 */
void debug_print_init(void);

/**
 * @brief poll the debug print process
 */
void debug_print_poll(void);

/**
 * @brief schedule a message for print out over UART
 */
#if DEBUG_PRINT_CONF_PRINT_FILE_AND_LINE
void debug_print_msg(uint32_t timestamp,
                     debug_level_t level,
                     char *data,
                     char *filename);
#else /* DEBUG_PRINT_CONF_PRINT_FILE_AND_LINE */
void debug_print_msg(uint32_t timestamp,
                     debug_level_t level,
                     char *data);
#endif /* DEBUG_PRINT_CONF_PRINT_FILE_AND_LINE */

/**
 * @brief print out a message immediately over UART (blocking call)
 */
void debug_print_msg_now(char *data);


uint16_t debug_print_get_max_stack_size(void);

/**
 * @brief put a string into the debug print buffer
 */
void debug_print_put(const char* str);

/* helper functions */
uint16_t sprint_hex16(uint16_t val, char* out_buffer);
uint16_t sprint_uint16(uint16_t val, char* out_buffer);
uint16_t sprint_uint32(uint32_t val, char* out_buffer, uint16_t min_chars);


#endif /* __DEBUG_PRINT_H__ */

/**
 * @}
 * @}
 */
