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

#include "main.h"

#if LOG_CONF_ON

#if !LOG_CONF_LEVEL_FIXED
log_level_t   log_lvl = LOG_LEVEL_INFO;
#endif /* LOG_CONF_LEVEL_FIXED */
char log_buffer[LOG_CONF_STRING_BUFFER_SIZE];
/* mapping from log level to string */
#if LOG_CONF_FORMAT_OUTPUT
const char* log_lvl_to_string[NUM_LOG_LEVELS] = { "",
                                                  "ERROR",
                                                  "WARNING",
                                                  "INFO",
                                                  "VERBOSE" };
#endif /* LOG_CONF_FORMAT_OUTPUT */
/*---------------------------------------------------------------------------*/
/* helper function, generates the message_t struct and forwards it to the
 * preconfigured target */
void
log_msg(const log_event_t* evt)
{
  /* generate a message */
  message_t msg;
  msg.header.device_id       = node_id;
  msg.header.type            = MSG_TYPE_LOG;
  msg.header.payload_len     = LOG_EVENT_HDR_LEN;
  if(evt->type == LOG_EVENT_GENERIC) {
    msg.header.payload_len  += evt->value;
  }
  msg.header.target_id       = DEVICE_ID_SINK;
#if LOG_CONF_TARGET == LOG_TARGET_BOLT
  msg.header.seqnr           = seq_no_bolt++;
#elif LOG_CONF_TARGET == LOG_TARGET_LWB
  msg.header.seqnr           = seq_no_lwb++;
#endif /* LOG_CONF_TARGET */
  msg.header.generation_time = lwb_get_timestamp();

  /* copy the payload */
  memcpy(&msg.log_event, evt, msg.header.payload_len);

  /* calculate and add the crc checksum */
  uint16_t crc = crc16((uint8_t*)&msg, msg.header.payload_len + MSG_HDR_LEN,0);
  MSG_SET_CRC16(&msg, crc);

#if LOG_CONF_TARGET == LOG_TARGET_BOLT
  BOLT_WRITE((uint8_t*)&msg, MSG_LEN(msg));
#elif LOG_CONF_TARGET == LOG_TARGET_LWB
  lwb_send_pkt(DEVICE_ID_SINK, LOG_CONF_LWB_STREAM_ID,
               (uint8_t*)&msg, MSG_LEN(msg));
#endif /* LOG_CONF_TARGET */
}
/*---------------------------------------------------------------------------*/
void
log_event(log_level_t lvl, log_event_type_t type, uint16_t val)
{
  log_event_t evt;
  evt.component_id = COMPONENT_ID;
  evt.type         = type;
  evt.value        = val;
  log_msg(&evt);
}
/*---------------------------------------------------------------------------*/
#if LOG_CONF_TARGET == LOG_TARGET_UART
void
log_event_str(log_level_t lvl, const char* evt_str, uint16_t val)
{
#if LOG_CONF_DISABLE_UART
  uart_enable(1);
#endif /* LOG_CONF_DISABLE_UART */
#if LOG_CONF_FORMAT_OUTPUT
  uint32_t now_ms = (rtimer_now_lf() * 1000) / RTIMER_SECOND_LF;
  printf("%u %5lu %s: %s (value: 0x%x)\r\n", node_id, now_ms,
         log_lvl_to_string[lvl], evt_str, val);
#else /* LOG_CONF_FORMAT_OUTPUT */
  printf("%s (value: 0x%x)\r\n", evt_str, val);
#endif /* LOG_CONF_FORMAT_OUTPUT */
#if LOG_CONF_DISABLE_UART
  uart_enable(0);
#endif /* LOG_CONF_DISABLE_UART */
}
#endif /* LOG_CONF_TARGET */
/*---------------------------------------------------------------------------*/
void
log_generic(log_level_t lvl, const char* str)
{
#if LOG_CONF_TARGET == LOG_TARGET_UART
 #if LOG_CONF_DISABLE_UART
  uart_enable(1);
 #endif /* LOG_CONF_DISABLE_UART */
 #if LOG_CONF_FORMAT_OUTPUT
  uint32_t now_ms = (rtimer_now_lf() * 1000) / RTIMER_SECOND_LF;
  printf("%u %5lu %s: %s\r\n", node_id, now_ms,
         log_lvl_to_string[lvl], str);
 #else /* LOG_CONF_FORMAT_OUTPUT */
  printf("%s\r\n", str);
 #endif /* LOG_CONF_FORMAT_OUTPUT */
 #if LOG_CONF_DISABLE_UART
  uart_enable(0);
 #endif /* LOG_CONF_DISABLE_UART */
#else
  log_event_t evt;
  evt.type    = LOG_EVENT_GENERIC;
  evt.value   = strlen(str);
  if(evt.value > (MSG_PAYLOAD_LEN - LOG_EVENT_HDR_LEN)) {
    evt.value = MSG_PAYLOAD_LEN - LOG_EVENT_HDR_LEN;
  }
  memcpy(evt.extra_data, str, evt.value);
  log_msg(&evt);
#endif /* LOG_CONF_TARGET */
}
/*---------------------------------------------------------------------------*/

#endif /* LOG_CONF_ON */
