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

/* log event types (application specific)
 *
 * each component (MCU) can define its own event types */

#ifndef __LOG_EVENT_H__
#define __LOG_EVENT_H__


typedef enum {
  LOG_EVENT_INVALID      = 0,
  LOG_EVENT_GENERIC      = 1,    /* value: length of event data */
  LOG_EVENT_NODE_RST     = 2,    /* value: reset cause */
  LOG_EVENT_CFG_CHANGED  = 3,    /* value: source or new value */
  LOG_EVENT_GLOSSY_ERROR = 4,    /* value: error count */

  LOG_EVENT_COMM_TIMESTAMP_SENT = 100,
} log_event_type_t;


/* mapping between message codes and human readable strings */

#define LOG_EVENT_NODE_RST_STR            "node reset"
#define LOG_EVENT_COMM_TIMESTAMP_SENT_STR "timestamp sent"
#define LOG_EVENT_CFG_CHANGED_STR         "config changed"
#define LOG_EVENT_GLOSSY_ERROR_STR        "Glossy error"

#endif /* __LOG_MSG_H__ */
