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
 */

#include "contiki.h"
//#include "platform.h"

#if DEBUG_PRINT_CONF_ON
/*---------------------------------------------------------------------------*/
#ifdef DEBUG_PRINT_TASK_ACT_PIN
#define DEBUG_PRINT_TASK_ACTIVE       LED_ON(DEBUG_PRINT_TASK_ACT_PIN)
#define DEBUG_PRINT_TASK_SUSPENDED    LED_OFF(DEBUG_PRINT_TASK_ACT_PIN)
#else
#define DEBUG_PRINT_TASK_ACTIVE
#define DEBUG_PRINT_TASK_SUSPENDED
#endif
/*---------------------------------------------------------------------------*/
const char* debug_print_lvl_to_string[NUM_OF_DEBUG_PRINT_LEVELS] = { \
  "CRITICAL", "ERROR", "WARNING", "INFO", "VERBOSE" };
/* global buffer, required to compose the messages */
char debug_print_buffer[DEBUG_PRINT_CONF_MAX_LEN + 1];   
#if DEBUG_PRINT_CONF_USE_XMEM
static uint8_t n_buffered_msg = 0;
static uint32_t start_addr_msg = MEMBX_INVALID_ADDR;
static debug_print_t msg;
#else
MEMB(debug_print_memb, debug_print_t, DEBUG_PRINT_CONF_NUM_MSG);
LIST(debug_print_list);
#endif /* DEBUG_PRINT_CONF_USE_XMEM */
/*---------------------------------------------------------------------------*/
PROCESS(debug_print_process, "Debug Print Task");
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(debug_print_process, ev, data) {
  PROCESS_BEGIN();

#if DEBUG_PRINT_CONF_USE_XMEM
  while (1);
  static uint32_t next_msg = MEMBX_INVALID_ADDR;
  n_buffered_msg = 0;
  start_addr_msg = MEMBX_INVALID_ADDR;     /* this line is necessary! */
  if (!xmem_init()) {          /* init if not already done */
    DEBUG_PRINT_FATAL("DEBUG-PRINT ERROR: failed to initialize FRAM");
  }
  start_addr_msg =
    xmem_alloc(DEBUG_PRINT_CONF_NUM_MSG * sizeof(debug_print_t));
  msg.content[DEBUG_PRINT_CONF_MAX_LEN] = 0; /* enforce a proper string */
#else  /* DEBUG_PRINT_CONF_USE_XMEM */
  memb_init(&debug_print_memb);
  list_init(debug_print_list);
#endif /* DEBUG_PRINT_CONF_USE_XMEM */
  
  while(1) {
    DEBUG_PRINT_TASK_SUSPENDED;
    PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_POLL);
    /* wait until we are polled by somebody */
    DEBUG_PRINT_TASK_ACTIVE;
    
#if DEBUG_PRINT_CONF_USE_XMEM
    
    next_msg = start_addr_msg;
    while(n_buffered_msg > 0) {
      /* load the message from the external memory */
      xmem_read(next_msg, sizeof(debug_print_t), (uint8_t *)&msg);
  #ifdef DEBUG_PRINT_DISABLE_UART
      UART_ENABLE;
  #endif /* DEBUG_PRINT_DISABLE_UART */
      printf("%3u %7llu %s %s: %s\r\n", node_id, RTIMER_TO_MS(
             msg.time), msg.module, debug_print_lvl_to_string[msg.level],
             msg.content);
  #ifdef DEBUG_PRINT_DISABLE_UART
      UART_DISABLE;
  #endif /* DEBUG_PRINT_DISABLE_UART */
      next_msg += sizeof(debug_print_t);
      n_buffered_msg--;
      /* do not pause process between the print-outs (otherwise a circular
         buffer / list data structure will be necessary!) */
      /* only let this task run when no other work is to do */
    }
    xmem_sleep();
    
#else /* DEBUG_PRINT_CONF_USE_XMEM */
    
    while(list_length(debug_print_list) > 0) {
      DEBUG_PRINT_TASK_SUSPENDED;
  #ifdef WITH_RADIO
    #ifdef WITH_GLOSSY
      /* do not try to print anything over the UART while Glossy is active */
      while(glossy_is_active()) {
        PROCESS_PAUSE();
      }
    #else
      /* do not try to print anything over the UART while the radio is busy */
      while(rf1a_is_busy()) {
        PROCESS_PAUSE();
      }
    #endif /* WITH_GLOSSY */
  #endif /* WITH_RADIO */
      DEBUG_PRINT_TASK_ACTIVE;
      /* print the first message in the queue */
      debug_print_t *msg = list_head(debug_print_list);
  #ifdef DEBUG_PRINT_DISABLE_UART
      UART_ENABLE;
  #endif /* DEBUG_PRINT_DISABLE_UART */
      printf("%3u %7llu %s %s: %s\r\n", node_id, RTIMER_TO_MS(
             msg->time), msg->module, debug_print_lvl_to_string[msg->level],
             msg->content);
  #ifdef DEBUG_PRINT_DISABLE_UART
      UART_DISABLE;
  #endif /* DEBUG_PRINT_DISABLE_UART */
      /* remove it from the queue */
      list_remove(debug_print_list, msg);
      memb_free(&debug_print_memb, msg);
      DEBUG_PRINT_TASK_SUSPENDED;
      PROCESS_PAUSE();
      DEBUG_PRINT_TASK_ACTIVE;
    }
    
#endif /* DEBUG_PRINT_CONF_USE_XMEM */

  }
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
char*
debug_print_format_mod_string(const char* module, char* out_buffer)
{
  uint8_t i = 0;  
  char* res = strrchr(module, '/');
  
#if !DEBUG_PRINT_CONF_USE_XMEM
  static char buffer[DEBUG_PRINT_MODULE_INFO_LEN + 1];
  if(!out_buffer) {
    out_buffer = buffer;
  }  
#else
  if(!out_buffer) {
    out_buffer = msg.module;
  }
#endif

  if(res) {
    module = res + 1;
  }
  while((i < DEBUG_PRINT_MODULE_INFO_LEN) && 
        (*module != 0) && (*module != '.')) {
    out_buffer[i++] = *module;
    module++;
  }
  out_buffer[i] = 0;  /* close the string */
  return out_buffer;
}
/*---------------------------------------------------------------------------*/
void
debug_print_init(void)
{
  printf("Starting '%s'\r\n", debug_print_process.name);
  process_start(&debug_print_process, NULL);
}
/*---------------------------------------------------------------------------*/
void
debug_print_poll(void)
{
  process_poll(&debug_print_process);
}
/*---------------------------------------------------------------------------*/
void
debug_print_msg(rtimer_clock_t *time, char level, char *module, char *data)
{  
#if DEBUG_PRINT_CONF_USE_XMEM
  if(n_buffered_msg < DEBUG_PRINT_CONF_NUM_MSG &&
     MEMBX_INVALID_ADDR != start_addr_msg) {
    /* construct the message struct */
    if(time == NULL) {
      msg.time = rtimer_now();
    } else {
      msg.time = *time;
    }
    msg.level = level;
    debug_print_format_mod_string(module, msg.module);
    memcpy(msg.content, data, DEBUG_PRINT_CONF_MAX_LEN);
    /* write to external memory */
    xmem_write(start_addr_msg + n_buffered_msg * sizeof(debug_print_t),
               sizeof(debug_print_t), (uint8_t *)&msg);
    n_buffered_msg++;
    /* do NOT poll the debug print process here! */
  }
#else
  debug_print_t *msg = memb_alloc(&debug_print_memb);
  if(msg != NULL) {
    /* construct the message struct */
    if(time == NULL) {
      msg->time = rtimer_now();
    } else {
      msg->time = *time;
    }
    msg->level = level;    
    debug_print_format_mod_string(module, msg->module);
    memcpy(msg->content, data, DEBUG_PRINT_CONF_MAX_LEN);
    /* add it to the list of messages ready to print */
    list_add(debug_print_list, msg);
    /* poll the debug print process */
    process_poll(&debug_print_process);
  }
#endif /* DEBUG_PRINT_CONF_USE_XMEM */
}
/*---------------------------------------------------------------------------*/
inline void
debug_print_msg_now(char *module, char *data)
{
  UART_ENABLE;
  printf(debug_print_format_mod_string(module, 0));
  putchar(' ');
  printf(data);
  printf("\r\n");
  UART_DISABLE;
}
/*---------------------------------------------------------------------------*/
#else /* DEBUG_PRINT_CONF_ON */
/*---------------------------------------------------------------------------*/
void
debug_print_init(void)
{
}
/*---------------------------------------------------------------------------*/
void
debug_print_msg(rtimer_clock_t *time, char level, char *module, char *content)
{
}
/*---------------------------------------------------------------------------*/
void
debug_print_msg_now(char *content)
{
}
/*---------------------------------------------------------------------------*/
#endif /* DEBUG_PRINT_CONF_ON */
