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

/* only to include project files files and define global/external variables
 * do not store any configuration in this file */

#ifndef __MAIN_H__
#define __MAIN_H__


#include "contiki.h"
#include "platform.h"
#include "elwb.h"
#include "messages/dpp_message.h"      /* packet structure and message types */
#include "event.h"
#include "message.h"


/* general return value type definition */
typedef enum
{
  FAILED = 0,
  SUCCESS = 1
} retval_t;

/* defined in fw.c */
retval_t fw_init(void);
retval_t fw_process_msg(dpp_message_t* msg);

/* global variables */
extern uint64_t utc_time;
extern uint64_t utc_time_rx;  /* reception time of the UTC in local ticks */
extern uint8_t  utc_time_updated;
extern rtimer_clock_t bolt_captured_trq;
extern uint16_t seq_no_lwb;   /* separate sequence number for each interface */
extern uint16_t seq_no_bolt;
extern uint32_t rst_flag;     /* defined in contiki-cc430-main.c */
extern dpp_message_t msg_tx;  /* needed for send_msg() */

#endif /* __MAIN_H__ */
