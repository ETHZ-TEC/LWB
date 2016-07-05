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

#include "contiki.h"
#include "platform.h"
#include "log.h"

/*---------------------------------------------------------------------------*/
#if LOG_CONF_ON
/*---------------------------------------------------------------------------*/
static log_t       log;
static log_entry_t log_entry;
static uint8_t     initialized = 0;
static uint32_t    last_entry = (uint32_t)LOG_CONF_START_ADDR + 
                                (uint32_t)LOG_META_LEN + 
                                ((uint32_t)LOG_CONF_NUM_ENTRIES - 1) *
                                (uint32_t)LOG_ENTRY_LEN;
/*---------------------------------------------------------------------------*/
uint8_t 
log_init(void)
{
  if(!initialized) {
    /* config check */
    if(last_entry > XMEM_SIZE) {
      DEBUG_PRINT_ERROR("log buffer does not fit into XMEM");
    }   
    
    /* initialize the external non-volatile memory */
    if(!xmem_init()) {
      DEBUG_PRINT_MSG_NOW("ERROR: log_init failed");
      return 0;
    }
    /* reset the meta data */
    memset(&log, 0, sizeof(log_t));
    
    /* read the meta data block from the memory */
    xmem_read(LOG_CONF_START_ADDR, sizeof(log_t), (uint8_t*)&log);
    uint16_t crc = crc16((uint8_t*)&log, sizeof(log) - 2, 0);
    if(crc != log.crc) {
      DEBUG_PRINT_MSG_NOW("WARNING: log corrupted, dropping all messages");
      log_clear();
    }  
    DEBUG_PRINT_MSG_NOW("Log initialized (%lu / %lu entries used)", 
                        log.n_entries,
                        (uint32_t)LOG_CONF_NUM_ENTRIES);
  
    initialized = 1;
    
    /* add a log entry */
    log_entry.event   = LOG_EVENT_LOG_INIT;
    log_entry.data[0] = 0;       /* no data */
    log_write(&log_entry);
  }
  return 1; 
}
/*---------------------------------------------------------------------------*/
void
log_clear(void)
{
  memset(&log, 0, sizeof(log_t));
  log.next_write = log.next_read = LOG_CONF_START_ADDR + sizeof(log_t);
  log.crc = crc16((uint8_t*)&log, sizeof(log) - 2, 0);
  xmem_write(LOG_CONF_START_ADDR, sizeof(log_t), (uint8_t*)&log);
}
/*---------------------------------------------------------------------------*/
uint8_t
log_write(const log_entry_t* entry)
{
#if LOG_CONF_ROLLOVER
  if(log.n_entries >= LOG_CONF_NUM_ENTRIES) {
    /* overwrite the oldest entry */
    log.n_entries--;
    if(log.next_read >= last_entry) {
      log.next_read = LOG_CONF_START_ADDR + sizeof(log_t);
    } else {
      log.next_read += sizeof(log_entry_t);
    }
  }
#endif /* LOG_CONF_ROLLOVER */
  if(initialized && (log.n_entries < LOG_CONF_NUM_ENTRIES)) {
    /* the process must not be interrupted! */
    uint16_t interrupt_enabled = __get_interrupt_state() & GIE;
    __dint(); __nop();
    if(!xmem_write(log.next_write, sizeof(log_entry_t), (uint8_t*)entry)) {
      return 0;
    }
    /* update the meta data */
    log.n_entries++;
    if(log.next_write >= last_entry) {
      log.next_write = LOG_CONF_START_ADDR + sizeof(log_t);
    } else {
      log.next_write += sizeof(log_entry_t);
    }
    log.crc = crc16((uint8_t*)&log, sizeof(log) - 2, 0);
    if(!xmem_write(LOG_CONF_START_ADDR, sizeof(log_t), (uint8_t*)&log)) {
      return 0;
    }
    if(interrupt_enabled) {
      __eint(); __nop();
    }
    return 1;
  }
  return 0;
}
/*---------------------------------------------------------------------------*/
void
log_print(uint16_t num_entries)
{
  if(initialized) {
    if(num_entries > log.n_entries || !num_entries) {
      num_entries = log.n_entries;
    }
    /* don't use the DEBUG_PRINT_... functions here, such that it also works
     * if debug print is disabled! */
    uart_enable(1);
    printf("--- START OF LOG ---\r\n");
    uint32_t read_ptr = log.next_read;
    char print_buffer[64];
    while(num_entries) {
      if(!xmem_read(read_ptr, sizeof(log_entry_t), (uint8_t*)&log_entry)) {
        break;
      }
      snprintf(print_buffer, 64, "@0x%04x: 0x%02x 0x%04x 0x%04x\r\n", 
                                 (uint16_t)read_ptr,
                                 log_entry.event,
                                 log_entry.data[0],
                                 log_entry.data[1]);
      uart_enable(1);
      printf(print_buffer);
      read_ptr += sizeof(log_entry_t);
      if(read_ptr > last_entry) {
        read_ptr = LOG_CONF_START_ADDR + sizeof(log_t);
      }
      num_entries--;
    }
    uart_enable(1);
    printf("--- END OF LOG ---\r\n");
    uart_enable(0);
  }
}
/*---------------------------------------------------------------------------*/
uint8_t 
log_write_simple(log_event_t event, uint16_t val1, uint16_t val2)
{
  log_entry.event   = event;
  log_entry.data[0] = val1;
  log_entry.data[1] = val2;
  return log_write(&log_entry);
}
/*---------------------------------------------------------------------------*/
#else /* LOG_CONF_ON */
/*---------------------------------------------------------------------------*/
uint8_t log_init(void) { return 0; }
void log_clear(void) { }
uint8_t log_write(const log_entry_t* entry) { return 0; }
void log_print(uint16_t num_entries) { }
uint8_t log_write_simple(log_event_t event, uint16_t val1, uint16_t val2) 
{ return 0; }    
/*---------------------------------------------------------------------------*/
#endif /* LOG_CONF_ON */
/*---------------------------------------------------------------------------*/
