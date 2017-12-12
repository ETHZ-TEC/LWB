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
 *          Federico Ferrari
 */

#include "contiki.h"

#if DEBUG_PRINT_CONF_ON
/*---------------------------------------------------------------------------*/
#ifdef DEBUG_PRINT_CONF_TASK_ACT_PIN
#define DEBUG_PRINT_TASK_ACTIVE       PIN_CLR(DEBUG_PRINT_CONF_TASK_ACT_PIN);\
                                      PIN_SET(DEBUG_PRINT_CONF_TASK_ACT_PIN)
#define DEBUG_PRINT_TASK_SUSPENDED    PIN_CLR(DEBUG_PRINT_CONF_TASK_ACT_PIN)
#else
#define DEBUG_PRINT_TASK_ACTIVE
#define DEBUG_PRINT_TASK_SUSPENDED
#endif
/*---------------------------------------------------------------------------*/
/* helper macros */
#if DEBUG_PRINT_CONF_DISABLE_UART
  #define DEBUG_PRINT_UART_ENABLE     uart_enable(1)
  #define DEBUG_PRINT_UART_DISABLE    uart_enable(0)
#else /* DEBUG_PRINT_CONF_DISABLE_UART */
  #define DEBUG_PRINT_UART_ENABLE
  #define DEBUG_PRINT_UART_DISABLE
#endif /* DEBUG_PRINT_CONF_DISABLE_UART */
/*---------------------------------------------------------------------------*/
struct printbuf
{
  uint8_t *data;
  uint16_t size;
  uint16_t put_idx, get_idx;
  uint16_t cnt;
};
void printbuf_put(struct printbuf* p, const char* str);
void printbuf_flush(struct printbuf* p);
/*---------------------------------------------------------------------------*/
const char* debug_print_lvl_to_string[NUM_OF_DEBUG_PRINT_LEVELS + 1] = { \
  "CRITICAL", "ERROR", "WARNING", "INFO", "VERBOSE", "" };
/* global buffer, required to compose the messages */
char debug_print_buffer[DEBUG_PRINT_CONF_MSG_LEN + 1]; 
static uint8_t buffer_full = 0;  
#if DEBUG_PRINT_CONF_USE_XMEM
  static uint8_t n_buffered_msg = 0;
  static uint32_t start_addr_msg = MEMBX_INVALID_ADDR;
  static debug_print_t msg;
#else /* DEBUG_PRINT_CONF_USE_XMEM */
  #if DEBUG_PRINT_CONF_USE_RINGBUFFER
    static struct  printbuf dbg_printbuf = { 0 };
    static uint8_t dbg_printbuf_data[DEBUG_PRINT_CONF_BUFFER_SIZE];
  #else /* DEBUG_PRINT_CONF_USE_RINGBUFFER */
    MEMB(debug_print_memb, debug_print_t, DEBUG_PRINT_CONF_NUM_MSG);
    LIST(debug_print_list);
  #endif /* DEBUG_PRINT_CONF_USE_RINGBUFFER */
#endif /* DEBUG_PRINT_CONF_USE_XMEM */
/*---------------------------------------------------------------------------*/
PROCESS(debug_print_process, "Debug Print Task");
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(debug_print_process, ev, data)
{
  PROCESS_BEGIN();
        
#if DEBUG_PRINT_CONF_USE_XMEM
  static uint32_t next_msg = MEMBX_INVALID_ADDR;
  n_buffered_msg = 0;
  start_addr_msg = MEMBX_INVALID_ADDR;     /* this line is necessary! */
  if (!xmem_init()) {          /* init if not already done */
    DEBUG_PRINT_FATAL("ERROR: fram init failed");
  }
  start_addr_msg = xmem_alloc(DEBUG_PRINT_CONF_NUM_MSG *sizeof(debug_print_t));
#else  /* DEBUG_PRINT_CONF_USE_XMEM */
 #if DEBUG_PRINT_CONF_USE_RINGBUFFER
  dbg_printbuf.data = dbg_printbuf_data;
  dbg_printbuf.size = DEBUG_PRINT_CONF_BUFFER_SIZE;
 #else /* DEBUG_PRINT_CONF_USE_RINGBUFFER */
  memb_init(&debug_print_memb);
  list_init(debug_print_list);
 #endif /* DEBUG_PRINT_CONF_USE_RINGBUFFER */
#endif /* DEBUG_PRINT_CONF_USE_XMEM */
  
#ifdef DEBUG_PRINT_CONF_TASK_ACT_PIN
  PIN_CFG_OUT(DEBUG_PRINT_CONF_TASK_ACT_PIN);
#endif
    
  uart_enable(1);       /* make sure UART is enabled */
  
#if DEBUG_CONF_STACK_GUARD
  *(uint16_t*)DEBUG_CONF_STACK_GUARD = 0xaaaa;
  *(uint16_t*)(DEBUG_CONF_STACK_GUARD + 2) = 0xaaaa;
  *(uint16_t*)(DEBUG_CONF_STACK_GUARD + 4) = 0xaaaa;
  *(uint16_t*)(DEBUG_CONF_STACK_GUARD + 6) = 0xaaaa;
  printf("Debug buffer size %ub, stack size=%ub\r\n",
         DEBUG_PRINT_CONF_NUM_MSG * DEBUG_PRINT_CONF_MSG_LEN + 
         DEBUG_PRINT_CONF_BUFFER_SIZE,
         (SRAM_END - DEBUG_CONF_STACK_GUARD - 7));
#else
  printf("Debug buffer size %ub\r\n",
         DEBUG_PRINT_CONF_NUM_MSG * DEBUG_PRINT_CONF_MSG_LEN + 
         DEBUG_PRINT_CONF_BUFFER_SIZE);
#endif /* DEBUG_CONF_STACK_GUARD */
  
  while(1) {
    /* suspend this task */
    PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_POLL);
    /* wait until we get polled by another thread */
    DEBUG_PRINT_TASK_ACTIVE;
        
#if DEBUG_PRINT_CONF_USE_XMEM
    next_msg = start_addr_msg;
    while(n_buffered_msg > 0) {
      /* load the message from the external memory */
      xmem_read(next_msg, sizeof(debug_print_t), (uint8_t *)&msg);
      DEBUG_PRINT_UART_ENABLE;
      msg.content[DEBUG_PRINT_CONF_MSG_LEN] = 0;
      printf("%u %7lu %s: %s\r\n", node_id, msg.time,
             debug_print_lvl_to_string[msg.level], msg.content);
      DEBUG_PRINT_UART_DISABLE;
      next_msg += sizeof(debug_print_t);
      n_buffered_msg--;
      /* do not pause process between the print-outs (otherwise a circular
         buffer / list data structure will be necessary!) */
      /* only let this task run when no other work is to do */
    }
    xmem_sleep();

#else /* DEBUG_PRINT_CONF_USE_XMEM */
    
    DEBUG_PRINT_UART_ENABLE;
  #if DEBUG_PRINT_CONF_USE_RINGBUFFER
    printbuf_flush(&dbg_printbuf);
  #else /* DEBUG_PRINT_CONF_USE_RINGBUFFER */
    while(list_length(debug_print_list) > 0) {
      debug_print_t *msg = list_head(debug_print_list);
      msg->content[DEBUG_PRINT_CONF_MSG_LEN] = 0;
      printf("%u %5lu %s: %s\r\n", node_id, msg->time,
             debug_print_lvl_to_string[msg->level], msg->content);
      /* remove it from the queue */
      list_remove(debug_print_list, msg);
      memb_free(&debug_print_memb, msg);
      /*DEBUG_PRINT_TASK_SUSPENDED;
        PROCESS_PAUSE();*/
    }
  #endif /* DEBUG_PRINT_CONF_USE_RINGBUFFER */
    DEBUG_PRINT_UART_DISABLE;
#endif /* DEBUG_PRINT_CONF_USE_XMEM */

    if(buffer_full) { 
      DEBUG_PRINT_UART_ENABLE;
      printf("WARNING: Debug messages dropped (buffer full)!\r\n");
      DEBUG_PRINT_UART_DISABLE;
      buffer_full = 0;
    }

#if DEBUG_CONF_STACK_GUARD
    /* check if the stack might be corrupt (check 8 bytes) */
    if(*(uint16_t*)DEBUG_CONF_STACK_GUARD != 0xaaaa       || 
       *(uint16_t*)(DEBUG_CONF_STACK_GUARD + 2) != 0xaaaa || 
       *(uint16_t*)(DEBUG_CONF_STACK_GUARD + 4) != 0xaaaa || 
       *(uint16_t*)(DEBUG_CONF_STACK_GUARD + 6) != 0xaaaa) {
      DEBUG_PRINT_FATAL("FATAL ERROR: Stack overflow detected");
    }    
#endif /* DEBUG_CONF_STACK_GUARD */

    DEBUG_PRINT_TASK_SUSPENDED;
  }
  PROCESS_END();
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
debug_print_msg(rtimer_clock_t timestamp, debug_level_t level, char *data
#if DEBUG_PRINT_CONF_PRINT_FILE_AND_LINE
                , char *filename, uint16_t line)
#else /* DEBUG_PRINT_CONF_PRINT_FILE_AND_LINE */
                )
#endif /* DEBUG_PRINT_CONF_PRINT_FILE_AND_LINE */
{  
#if DEBUG_PRINT_CONF_USE_XMEM
  if(n_buffered_msg < DEBUG_PRINT_CONF_NUM_MSG &&
     MEMBX_INVALID_ADDR != start_addr_msg) {
    /* compose the message struct */
    msg.time = timestamp / RTIMER_SECOND_LF;
    msg.level = level;
    memcpy(msg.content, data, DEBUG_PRINT_CONF_MSG_LEN);
    /* write to external memory */
    xmem_write(start_addr_msg + n_buffered_msg * sizeof(debug_print_t),
               sizeof(debug_print_t), (uint8_t *)&msg);
    n_buffered_msg++;
    /* do NOT poll the debug print process here! */
  } else {
    buffer_full = 1;
  }
#else
 #if DEBUG_PRINT_CONF_USE_RINGBUFFER
  char tmp[32];
  #if DEBUG_PRINT_CONF_PRINT_NODEID
  snprintf(tmp, 32, "%u ", node_id);
  printbuf_put(&dbg_printbuf, tmp);
  #endif /* DEBUG_PRINT_CONF_PRINT_NODEID */
  #if DEBUG_PRINT_CONF_PRINT_TIMESTAMP
  snprintf(tmp, 32, "%5lu ", (uint32_t)(timestamp / RTIMER_SECOND_LF));
  printbuf_put(&dbg_printbuf, tmp);
  #endif /* DEBUG_PRINT_CONF_PRINT_TIMESTAMP */
  #if DEBUG_PRINT_CONF_PRINT_DBGLEVEL
  printbuf_put(&dbg_printbuf, debug_print_lvl_to_string[level]);
  printbuf_put(&dbg_printbuf, " ");
  #endif /* DEBUG_PRINT_CONF_PRINT_DBGLEVEL */
  #if DEBUG_PRINT_CONF_PRINT_FILE_AND_LINE
  snprintf(tmp, 32, "%s %d ", filename, line);
  printbuf_put(&dbg_printbuf, tmp);
  #endif /* DEBUG_PRINT_CONF_PRINT_FILE_AND_LINE */
  printbuf_put(&dbg_printbuf, data);
  printbuf_put(&dbg_printbuf, "\r\n");
 #else /* DEBUG_PRINT_CONF_USE_RINGBUFFER */
  debug_print_t *msg = memb_alloc(&debug_print_memb);
  if(msg != NULL) {
    /* compose the message struct */
    msg->time = timestamp / RTIMER_SECOND_LF;
    msg->level = level;
    memcpy(msg->content, data, DEBUG_PRINT_CONF_MSG_LEN);
    /* add it to the list of messages ready to print */
    list_add(debug_print_list, msg);
    /* poll the debug print process */
  #if DEBUG_PRINT_CONF_POLL
    process_poll(&debug_print_process);
  #endif /* DEBUG_PRINT_CONF_POLL */
  } else {
    buffer_full = 1;
  }
 #endif /* DEBUG_PRINT_CONF_USE_RINGBUFFER */
#endif /* DEBUG_PRINT_CONF_USE_XMEM */
}
/*---------------------------------------------------------------------------*/
void
debug_print_msg_now(char *data)
{
  if(data) {
    DEBUG_PRINT_UART_ENABLE;
    printf(data);
    printf("\r\n");
    DEBUG_PRINT_UART_DISABLE;
  }
}
/*---------------------------------------------------------------------------*/
void
printbuf_put(struct printbuf* p, const char* str)
{
  if(p->cnt == p->size)
    return;
  uint16_t lim = p->size - 3;
  while(*str && p->cnt < lim) {
    p->data[p->put_idx++] = *str++;
    if(p->put_idx == p->size) {
      p->put_idx = 0;
    }
    p->cnt++;
  }
  if(*str) {
    /* no more space -> add special character + newline */
    static const char line_cut[4] = "~\r\n";
    str = line_cut;
    while(*str) {
      p->data[p->put_idx++] = *str++;
      if(p->put_idx == p->size) {
        p->put_idx = 0;
      }
    }
    p->cnt += 3;
  }
}
/*---------------------------------------------------------------------------*/
void
printbuf_flush(struct printbuf* p)
{
  while(p->cnt)
  {
    uint8_t c = p->data[p->get_idx++];
    if(p->get_idx == p->size) {
      p->get_idx = 0;
    }
    putchar(c);
    p->cnt--;
  }
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
debug_print_poll(void)
{
}
/*---------------------------------------------------------------------------*/
void
debug_print_msg(rtimer_clock_t timestamp, debug_level_t level, char *data)
{
}
/*---------------------------------------------------------------------------*/
void
debug_print_msg_now(char *data)
{
}
/*---------------------------------------------------------------------------*/
#endif /* DEBUG_PRINT_CONF_ON */
