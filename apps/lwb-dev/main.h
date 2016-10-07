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

/* only to include project files files and define global/external variables
 * do not store any configuration in this file */

#ifndef __MAIN_H__
#define __MAIN_H__


#include "contiki.h"
#include "platform.h"
#include "nvcfg.h"
#include "log-events.h"
#include "packet.h"                    /* packet structure and message types */
#include "log.h"


/* TODO: use different streams for different message types */
#define STREAM_ID    1


/* non-volatile configuration */
typedef struct {
  uint8_t   rst_cnt;
  uint8_t   reserved[7];
} config_t;


void host_init(void);
void source_init(void);
void host_run(void);
void source_run(void);

/* defined in utils.c */
void get_node_health(comm_health_t* out_data);
void get_node_info(node_info_t* out_data);
void send_msg(uint16_t recipient,
              message_type_t type,
              const uint8_t* data,
              uint8_t len,
              uint8_t send_to_bolt);

/* the static scheduler implements the following function: */
void lwb_sched_set_period(uint16_t period);

/* global variables */
extern uint16_t seq_no_lwb;   /* separate sequence number for each interface */
extern uint16_t seq_no_bolt;
extern uint32_t rst_flag;     /* defined in contiki-cc430-main.c */
extern config_t cfg;          /* most important config parameters and stats */


#endif /* __MAIN_H__ */
