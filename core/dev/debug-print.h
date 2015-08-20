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
 *          Federico Ferrari
 */

#ifndef __DEBUG_PRINT_H__
#define __DEBUG_PRINT_H__

/* necessary to override the default settings in platform.h */
#include "platform.h"   

/* enabled by default */
#ifndef DEBUG_PRINT_CONF_ON
#define DEBUG_PRINT_CONF_ON             1
#endif /* DEBUG_PRINT_CONF_ON */

#ifndef DEBUG_PRINT_CONF_NUM_MSG        /* number of messages to store */
#define DEBUG_PRINT_CONF_NUM_MSG        8     
#endif /* DEBUG_PRINT_CONF_NUM_MSG */

#ifndef DEBUG_PRINT_CONF_MSG_LEN        /* max. num of chars per msg */
#define DEBUG_PRINT_CONF_MSG_LEN        80    
#endif /* DEBUG_PRINT_CONF_MSG_LEN */

#ifndef DEBUG_PRINT_MODULE_INFO_LEN     /* num chars for module designator */
#define DEBUG_PRINT_MODULE_INFO_LEN     12    
#endif /* DEBUG_PRINT_MODULE_INFO_LEN */

#ifndef DEBUG_PRINT_CONF_LEVEL
#define DEBUG_PRINT_CONF_LEVEL          DEBUG_PRINT_LVL_INFO
#endif /* DEBUG_PRINT_CONF_LEVEL */

#ifndef DEBUG_PRINT_CONF_USE_XMEM
#define DEBUG_PRINT_CONF_USE_XMEM       0
#endif /* DEBUG_PRINT_CONF_USE_XMEM */

#ifndef DEBUG_PRINT_CONF_PRINT_DIRECT   /* print directly, no queuing */
#define DEBUG_PRINT_CONF_PRINT_DIRECT   0
#endif /* DEBUG_PRINT_CONF_PRINT_DIRECT */

/* poll the debug task every time a message is schedule for print-out? */
#ifndef DEBUG_PRINT_CONF_POLL
#define DEBUG_PRINT_CONF_POLL           0    
#endif /* DEBUG_PRINT_CONF_POLL */

/**
 * @brief set DEBUG_PRINT_DISABLE_CONF_UART to 1 to disable UART after each
 * print out (& re-enable it before each print out)
 */
#ifndef DEBUG_PRINT_CONF_DISABLE_UART
#define DEBUG_PRINT_CONF_DISABLE_UART   1
#endif /* DEBUG_PRINT_DISABLE_UART */

#ifdef LED_ERROR                        /* don't use LED_ON */
#define DEBUG_PRINT_ERROR_LED_ON        PIN_SET(LED_ERROR)                                        
#else
#define DEBUG_PRINT_ERROR_LED_ON
#endif

/* set debugging level for each module (0 = no debug prints) */
#define DEBUG_PRINT(level, time, title, ...)   \
  if(DEBUG_PRINT_CONF_LEVEL >= level) { \
    DEBUG_PRINT_MSG(time, title, __VA_ARGS__); }

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
  DEBUG_PRINT_ERROR_LED_ON; watchdog_start(); \
  while(1); \
}
    

#if DEBUG_PRINT_CONF_ON
  #if DEBUG_PRINT_CONF_PRINT_DIRECT
    #define DEBUG_PRINT_MSG(t, p, ...) \
      snprintf(debug_print_buffer, DEBUG_PRINT_CONF_MSG_LEN + 1, __VA_ARGS__);\
      debug_print_msg_now(__FILE__, debug_print_buffer)
  #else /* DEBUG_PRINT_CONF_PRINT_DIRECT */
    #define DEBUG_PRINT_MSG(t, p, ...) \
      snprintf(debug_print_buffer, DEBUG_PRINT_CONF_MSG_LEN + 1, __VA_ARGS__);\
      debug_print_msg(RTIMER_LF_TO_MS(rtimer_now_lf()), p, __FILE__, \
                      debug_print_buffer)  
  #endif /* DEBUG_PRINT_CONF_PRINT_DIRECT */
  #define DEBUG_PRINT_MSG_NOW(...) \
    snprintf(debug_print_buffer, DEBUG_PRINT_CONF_MSG_LEN + 1, __VA_ARGS__); \
    debug_print_msg_now(__FILE__, debug_print_buffer)
#else /* DEBUG_PRINT_CONF_ON */
  #define DEBUG_PRINT_MSG(t, p, ...)
  #define DEBUG_PRINT_MSG_NOW(...) 
#endif /* DEBUG_PRINT_CONF_ON */

#define DEBUG_PRINT_STACK_ADDRESS { \
  UART_ENABLE; \
  uint8_t pos = 16; \
  uint16_t addr = (uint16_t)&pos; /* or use: READ_SP */\
  while (pos) { \
      pos = pos - 4; \
      uint8_t c = ((addr >> pos) & 0x000f); \
      if (c > 9) { c += ('a' - '0' - 10); } \
      putchar('0' + c); \
  } \
  putchar(' '); \
  UART_DISABLE; \
}

#define DEBUG_PRINT_STACK_SIZE { \
  UART_ENABLE; \
  uint16_t div = 1000; \
  uint16_t addr = 0x2c00 - (uint16_t)&div; /* or use: READ_SP */ \
  while (div) { \
      uint8_t c = addr/div; \
      putchar('0' + c); \
      addr -= c * div; \
      div = div/10; \
  } \
  putchar(' '); \
  UART_DISABLE; \
}


/* debug levels (severity level) */
typedef enum {  
  DEBUG_PRINT_LVL_EMERGENCY = 0, /* critical, requires immediate action */
  DEBUG_PRINT_LVL_ERROR = 1,     /* alert, something went wrong */
  DEBUG_PRINT_LVL_WARNING = 2,   /* something unexpected happend */
  DEBUG_PRINT_LVL_INFO = 3,      /* status message */
  DEBUG_PRINT_LVL_VERBOSE = 4,   /* debug output */
  NUM_OF_DEBUG_PRINT_LEVELS
} debug_level_t;

/* +1 for the trailing \0 character */
extern char debug_print_buffer[DEBUG_PRINT_CONF_MSG_LEN + 1];   

typedef struct debug_print_t {
  struct debug_print_t *next;
  rtimer_clock_t time;
  uint8_t level;
  char module[DEBUG_PRINT_MODULE_INFO_LEN + 1]; /* src module of the message */
  char content[DEBUG_PRINT_CONF_MSG_LEN + 1];
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
void debug_print_msg(uint64_t time, 
                     char level, 
                     char *module, 
                     char *data);

/**
 * @brief print out a message immediately over UART (blocking call)
 */
void debug_print_msg_now(char *module, char *data);


#endif /* __DEBUG_PRINT_H__ */
