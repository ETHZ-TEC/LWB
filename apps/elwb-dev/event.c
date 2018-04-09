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

#include "main.h"

#if EVENT_CONF_ON

#if !EVENT_CONF_LEVEL_FIXED
event_level_t   event_lvl = EVENT_LEVEL_INFO;
#endif /* EVENT_CONF_LEVEL_FIXED */

/*---------------------------------------------------------------------------*/
void
event_write(event_level_t lvl, dpp_event_type_t type, uint16_t val)
{
#if EVENT_CONF_TARGET == EVENT_TARGET_UART
  char tmp[32];
  snprintf(tmp, EVENT_CONF_STRING_BUFFER_SIZE + 1, 
           "event: 0x%02x, code: 0x%02x", type, value);
  DEBUG_PRINT_MSG(0, (debug_level_t)lvl, tmp);

#else /* EVENT_CONF_TARGET == EVENT_TARGET_UART */
 
  /* generate a message */
  dpp_message_t msg;
  msg.header.device_id       = node_id;
  msg.header.type            = DPP_MSG_TYPE_EVENT;
  msg.header.payload_len     = DPP_EVENT_LEN;
  msg.header.target_id       = DPP_DEVICE_ID_SINK;
 #if EVENT_CONF_TARGET == EVENT_TARGET_BOLT
  msg.header.seqnr           = seq_no_bolt++;
 #elif EVENT_CONF_TARGET == EVENT_TARGET_LWB
  msg.header.seqnr           = seq_no_lwb++;
 #endif /* EVENT_CONF_TARGET */
  msg.header.generation_time = lwb_get_timestamp();

  /* create the payload */
  msg.event.type = type;
  msg.event.value = val;

  /* calculate and add the crc checksum */
  uint16_t crc = crc16((uint8_t*)&msg, 
                       msg.header.payload_len + DPP_MSG_HDR_LEN, 0);
  DPP_MSG_SET_CRC16(&msg, crc);

 #if EVENT_CONF_TARGET == EVENT_TARGET_BOLT
  bolt_write((uint8_t*)&msg, DPP_MSG_LEN(msg));
 #elif EVENT_CONF_TARGET == EVENT_TARGET_LWB
  lwb_send_pkt(DEVICE_ID_SINK, EVENT_CONF_LWB_STREAM_ID,
               (uint8_t*)&msg, DPP_MSG_LEN(msg));
 #endif /* EVENT_CONF_TARGET */
  
#endif /* EVENT_CONF_TARGET == EVENT_TARGET_UART */
}
/*---------------------------------------------------------------------------*/

#endif /* EVENT_CONF_ON */
