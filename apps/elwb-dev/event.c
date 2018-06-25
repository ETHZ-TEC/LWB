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
event_level_t   event_lvl = EVENT_CONF_LEVEL;
event_target_t  event_tgt = EVENT_CONF_TARGET;
#endif /* EVENT_CONF_LEVEL_FIXED */

/*---------------------------------------------------------------------------*/
void
event_write(event_level_t lvl, dpp_event_type_t type, uint32_t val)
{
  if(event_tgt == EVENT_TARGET_UART) {
    char tmp[33];
    snprintf(tmp, 33, "event: 0x%02x, code: 0x%02lx", type, val);
    DEBUG_PRINT_MSG(0, (debug_level_t)lvl, tmp);
    
  } else {
    dpp_event_t event;
    event.type = type;
    event.value = val;

    if(event_tgt == EVENT_TARGET_BOLT) {
      send_msg(DPP_DEVICE_ID_SINK, DPP_MSG_TYPE_EVENT, (uint8_t*)&event, 0, 1);
      DEBUG_PRINT_VERBOSE("event msg sent to BOLT");
    } else if (event_tgt == EVENT_TARGET_LWB) {
      send_msg(DPP_DEVICE_ID_SINK, DPP_MSG_TYPE_EVENT, (uint8_t*)&event, 0, 0);
      DEBUG_PRINT_VERBOSE("event msg sent to LWB");
    } else {
      DEBUG_PRINT_WARNING("invalid event target");
    }
  }
}
/*---------------------------------------------------------------------------*/

#endif /* EVENT_CONF_ON */
