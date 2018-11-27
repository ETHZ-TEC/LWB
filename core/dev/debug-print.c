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
  #define DEBUG_PRINT_UART_DISABLE    while(UART_ACTIVE)
#endif /* DEBUG_PRINT_CONF_DISABLE_UART */
/*---------------------------------------------------------------------------*/
struct printbuf {
  uint8_t *data;
  uint16_t size;
  uint16_t put_idx, get_idx;
  uint16_t cnt;
};
static void printbuf_flush(void);       /* private function */
/*---------------------------------------------------------------------------*/
const char* debug_print_lvl_to_string[NUM_OF_DEBUG_PRINT_LEVELS] = { \
  "CRIT: ", "ERROR:", "WARN: ", "INFO: ", "DBG:  " };
/* global buffer, required to compose the messages */
char debug_print_buffer[DEBUG_PRINT_CONF_MSG_LEN];
#if DEBUG_PRINT_CONF_USE_XMEM
static uint8_t buffer_full = 0;
static uint8_t n_buffered_msg = 0;
static uint32_t start_addr_msg = MEMBX_INVALID_ADDR;
static debug_print_t msg;
#else /* DEBUG_PRINT_CONF_USE_XMEM */
static struct  printbuf dbg_printbuf = { 0 };
static uint8_t dbg_printbuf_data[DEBUG_PRINT_CONF_BUFFER_SIZE];
#endif /* DEBUG_PRINT_CONF_USE_XMEM */
/*---------------------------------------------------------------------------*/
PROCESS(debug_print_process, "Debug Print");
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(debug_print_process, ev, data)
{
  PROCESS_BEGIN();

  uart_enable(1);       /* make sure UART is enabled */
  printf("Process '%s' started" DEBUG_PRINT_CONF_EOL, debug_print_process.name);

#if DEBUG_PRINT_CONF_USE_XMEM
  static uint32_t next_msg = MEMBX_INVALID_ADDR;
  n_buffered_msg = 0;
  start_addr_msg = MEMBX_INVALID_ADDR;     /* this line is necessary! */
  if (!xmem_init()) {          /* init if not already done */
    DEBUG_PRINT_FATAL("ERROR: fram init failed");
  }
  start_addr_msg = xmem_alloc(DEBUG_PRINT_CONF_NUM_MSG *sizeof(debug_print_t));
#else  /* DEBUG_PRINT_CONF_USE_XMEM */
  dbg_printbuf.data = dbg_printbuf_data;
  dbg_printbuf.size = DEBUG_PRINT_CONF_BUFFER_SIZE;
#endif /* DEBUG_PRINT_CONF_USE_XMEM */

#ifdef DEBUG_PRINT_CONF_TASK_ACT_PIN
  PIN_CFG_OUT(DEBUG_PRINT_CONF_TASK_ACT_PIN);
#endif

#if DEBUG_CONF_STACK_GUARD
  /* fîll all unused stack memory with a dummy value */
  uint16_t* addr;
  uint16_t* end = (uint16_t*)&addr - 1;   /* stop at the current stack height */
  for(addr = (uint16_t*)(DEBUG_CONF_STACK_GUARD); addr < end; addr++) {
    *addr = 0xaaaa;
  }
  printf("Debug buffer size %uB, max stack size %uB" DEBUG_PRINT_CONF_EOL,
         DEBUG_PRINT_CONF_NUM_MSG * DEBUG_PRINT_CONF_MSG_LEN + 
         DEBUG_PRINT_CONF_BUFFER_SIZE,
         (SRAM_END - DEBUG_CONF_STACK_GUARD - 7));
#else
  printf("Debug buffer size %uB" DEBUG_PRINT_CONF_EOL,
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
      printf("%u %7lu %s: %s" DEBUG_PRINT_CONF_EOL, node_id, msg.time,
             debug_print_lvl_to_string[msg.level], msg.content);
      DEBUG_PRINT_UART_DISABLE;
      next_msg += sizeof(debug_print_t);
      n_buffered_msg--;
      /* do not pause process between the print-outs (otherwise a circular
         buffer / list data structure will be necessary!) */
      /* only let this task run when no other work is to do */
    }
    if(buffer_full) {
      printf("WARN:  debug print buffer full" DEBUG_PRINT_CONF_EOL);
      buffer_full = 0;
    }
    xmem_sleep();

#else /* DEBUG_PRINT_CONF_USE_XMEM */

    DEBUG_PRINT_UART_ENABLE;
    printbuf_flush();
    DEBUG_PRINT_UART_DISABLE;
#endif /* DEBUG_PRINT_CONF_USE_XMEM */

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
#if DEBUG_PRINT_CONF_PRINT_FILENAME
debug_print_msg(uint32_t timestamp,
                debug_level_t level,
                char *data,
                char *filename)
#else /* DEBUG_PRINT_CONF_PRINT_FILENAME */
debug_print_msg(uint32_t timestamp,
                debug_level_t level,
                char *data)
#endif /* DEBUG_PRINT_CONF_PRINT_FILENAME */
{
#if DEBUG_PRINT_CONF_USE_XMEM
  if(n_buffered_msg < DEBUG_PRINT_CONF_NUM_MSG &&
     MEMBX_INVALID_ADDR != start_addr_msg) {
    /* compose the message struct */
    msg.time = timestamp;
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
  char tmp[32];
  debug_print_put("[");
  #if DEBUG_PRINT_CONF_PRINT_DBGLEVEL
  debug_print_put(debug_print_lvl_to_string[level]);
  #endif /* DEBUG_PRINT_CONF_PRINT_DBGLEVEL */
  #if DEBUG_PRINT_CONF_PRINT_NODEID
  snprintf(tmp, 32, DEBUG_PRINT_CONF_PRINT_DBGLEVEL ? " %u " : "%u ", node_id);
  debug_print_put(tmp);
  #endif /* DEBUG_PRINT_CONF_PRINT_NODEID */
  #if DEBUG_PRINT_CONF_PRINT_FILENAME
  snprintf(tmp, 32,
           (DEBUG_PRINT_CONF_PRINT_DBGLEVEL ||
            DEBUG_PRINT_CONF_PRINT_NODEID) ? " %-10s" : "%-10s",
           filename);
  debug_print_put(tmp);
  #endif /* DEBUG_PRINT_CONF_PRINT_FILENAME */
  #if DEBUG_PRINT_CONF_PRINT_TIMESTAMP
  snprintf(tmp, 32, 
           (DEBUG_PRINT_CONF_PRINT_DBGLEVEL ||
            DEBUG_PRINT_CONF_PRINT_NODEID   ||
            DEBUG_PRINT_CONF_PRINT_FILENAME) ?  " %4lu" : "%4lu",
           timestamp);
  debug_print_put(tmp);
  #endif /* DEBUG_PRINT_CONF_PRINT_TIMESTAMP */
  debug_print_put("] ");
  debug_print_put(data);
  debug_print_put(DEBUG_PRINT_CONF_EOL);
 #if DEBUG_PRINT_CONF_POLL
  process_poll(&debug_print_process);
 #endif /* DEBUG_PRINT_CONF_POLL */
#endif /* DEBUG_PRINT_CONF_USE_XMEM */
}
/*---------------------------------------------------------------------------*/
void
debug_print_msg_now(char *data)
{
  if(data) {
    DEBUG_PRINT_UART_ENABLE;
    printf(data);
    printf(DEBUG_PRINT_CONF_EOL);
    DEBUG_PRINT_UART_DISABLE;
  }
}
/*---------------------------------------------------------------------------*/
uint16_t
debug_print_get_max_stack_size(void)
{
#if DEBUG_CONF_STACK_GUARD
  uint16_t* addr = (uint16_t*)DEBUG_CONF_STACK_GUARD;
  uint16_t* end = (uint16_t*)(SRAM_START + SRAM_SIZE);
  for(; addr < end; addr++) {
    if(*addr != 0xaaaa) {
      return ((uint16_t)end - (uint16_t)addr);
    }
  }
#endif /* DEBUG_CONF_STACK_GUARD */
  return 0;
}
/*---------------------------------------------------------------------------*/
#if !DEBUG_PRINT_CONF_USE_XMEM
void
debug_print_put(const char* str)
{
  static const char line_cut[] = "~" DEBUG_PRINT_CONF_EOL;

  if(dbg_printbuf.cnt == dbg_printbuf.size)
    return;
  uint16_t lim = dbg_printbuf.size - strlen(line_cut);
  while(*str && dbg_printbuf.cnt < lim) {
    dbg_printbuf.data[dbg_printbuf.put_idx++] = *str++;
    if(dbg_printbuf.put_idx == dbg_printbuf.size) {
      dbg_printbuf.put_idx = 0;
    }
    dbg_printbuf.cnt++;
  }
  if(*str) {
    /* no more space -> add special character + newline */
    str = line_cut;
    while(*str) {
      dbg_printbuf.data[dbg_printbuf.put_idx++] = *str++;
      if(dbg_printbuf.put_idx == dbg_printbuf.size) {
        dbg_printbuf.put_idx = 0;
      }
    }
    dbg_printbuf.cnt += strlen(line_cut);
  }
}
/*---------------------------------------------------------------------------*/
static void
printbuf_flush(void)
{
  while(dbg_printbuf.cnt)
  {
    uint8_t c = dbg_printbuf.data[dbg_printbuf.get_idx++];
    if(dbg_printbuf.get_idx == dbg_printbuf.size) {
      dbg_printbuf.get_idx = 0;
    }
    putchar(c);
    dbg_printbuf.cnt--;
  }
}
#endif /* DEBUG_PRINT_CONF_USE_XMEM */
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
debug_print_msg(uint32_t timestamp, debug_level_t level, char *data)
{
}
/*---------------------------------------------------------------------------*/
void
debug_print_msg_now(char *data)
{
}
/*---------------------------------------------------------------------------*/
void
debug_print_get_max_stack_size(void)
{
}
/*---------------------------------------------------------------------------*/
void
debug_print_put(void)
{
}
/*---------------------------------------------------------------------------*/
#endif /* DEBUG_PRINT_CONF_ON */
/*---------------------------------------------------------------------------*/
uint16_t
sprint_hex16(uint16_t val, char* out_buffer)
{
  uint16_t mask = 0xf000;
  while(mask) {
    uint16_t b = val & mask;
    if(b < 10) {
      *out_buffer = b + '0';
    } else {
      *out_buffer = b + ('a' - 10);
    }
    out_buffer++;
    mask >>= 4;
  }
  return 4;
}
/*---------------------------------------------------------------------------*/
uint16_t
sprint_uint16(uint16_t val, char* out_buffer)
{
  uint16_t char_cnt = 1;
  uint16_t div = 1;
  if(val >= 10000) {
    div = 10000;
    char_cnt = 5;
  } else if(val >= 1000) {
    div = 1000;
    char_cnt = 4;
  } else if(val >= 100) {
    div = 100;
    char_cnt = 3;
  } else if (val >= 10) {
    div = 10;
    char_cnt = 2;
  }
  while(div > 1) {
    uint16_t d = val / div;
    *out_buffer++ = d + '0';
    val -= d * div;
    div /= 10;
  }
  *out_buffer++ = (uint8_t)(val + '0');
  return char_cnt;
}
/*---------------------------------------------------------------------------*/
uint16_t
sprint_uint32(uint32_t val, char* out_buffer, uint16_t min_chars)
{
  uint32_t div      = 1000000000;
  uint16_t char_cnt = 0;
  uint16_t digits   = 10;   /* start with the highest digit */
  while(digits > 1) {
    if(char_cnt == 0) {
      if(val < div) {
        if(digits <= min_chars) {
          *out_buffer++ = ' ';
        }
        div /= 10;
        digits--;
        continue;
      }
      char_cnt = digits;
    }
    uint16_t d = (val / div);
    *out_buffer++ = d + '0';
    digits--;
    val -= d * div;
    div /= 10;
  }
  *out_buffer++ = (uint8_t)(val + '0');
  if(min_chars > char_cnt) {
    char_cnt = min_chars;
  }
  return char_cnt;
}
/*---------------------------------------------------------------------------*/
