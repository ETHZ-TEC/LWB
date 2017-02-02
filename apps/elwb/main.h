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
 */
 
#ifndef __MAIN_H__
#define __MAIN_H__


#include "contiki.h"
#include "platform.h"


#define MSG_PKT_LEN     (LWB_CONF_MAX_DATA_PKT_LEN - LWB_CONF_HEADER_LEN - \
                         GLOSSY_MAX_HEADER_LEN)     /* 62 - 3 - 4 = 55 bytes */
#define MSG_HDR_LEN     4
#define MSG_PAYLOAD_LEN (MSG_PKT_LEN - MSG_HDR_LEN - 2)

/* actual message length including CRC */
#define MSG_LEN(m)      ((m).header.payload_len + MSG_HDR_LEN + 2)
#define MSG_LEN_PTR(m)  ((m)->header.payload_len + MSG_HDR_LEN + 2)


/* the message type (7 bits available, MSB is reserved) */
typedef enum {
  MSG_TYPE_INVALID = 0,
  MSG_TYPE_TIMESYNC = 1,
  MSG_TYPE_TIMECONV = 5,    /* request a pair of timestamps (global + local) */
  MSG_TYPE_COMM_CMD = 10,
  MSG_TYPE_AE_EVENT = 160,
  MSG_TYPE_AE_DATA = 161,
} message_type_t;

#define MSG_TYPE_MIN    0x80      // last bit of 'type' defines header length

typedef enum {
  COMM_CMD_LWB_RESUME = 0,
  COMM_CMD_LWB_PAUSE,
  COMM_CMD_LWB_SET_ROUND_PERIOD,
  COMM_CMD_LWB_SET_HEALTH_PERIOD,
  COMM_CMD_LWB_SET_TX_PWR,
  COMM_CMD_NODE_RESET = 50,
} comm_cmd_type_t;

typedef struct {
  comm_cmd_type_t type : 8;
  uint16_t        value;
} comm_cmd_t;

typedef uint64_t timestamp_t;

typedef struct {
  timestamp_t global_time;
  timestamp_t local_time;
} timeconv_t;


/* application layer packet format (a packet is called 'message') */

#pragma pack(1)

typedef struct {
  uint16_t    event_id;    /* sequence number */
  uint16_t    len;         /* length in number of samples */
  timestamp_t generation_time;
  /* more characteristics of the AE event */
  uint16_t    amplitude;   /* max. amplitude */
  uint16_t    rise_time;   /* rise time */
  uint16_t    avg_freq;    /* average frequency */
} ae_event_t;

#define AE_DATA_SAMPLES_PER_PKT   ((MSG_PAYLOAD_LEN - 4) / 2)
typedef struct {
  uint16_t event_id;    /* sequence number */
  uint16_t offset;      /* offset in samples since the start */
  uint16_t samples[AE_DATA_SAMPLES_PER_PKT];
} ae_data_t;

/* 
 * message structure for acoustic emission messages
 * - no target ID required (packets always go to the sink)
 * - no generation time required (will be included in the payload if necessary)
 * - crc is calculated over the whole structure and appended to the message
 */
typedef struct {
  /* header: MSG_HDR_LEN bytes */
  struct {
    uint16_t       device_id;   /* sender node ID */
    /* MSB of type must be set! */
    message_type_t type : 8;    /* force 1 byte */
    uint8_t        payload_len;
  } header;
  /* payload: MSG_PAYLOAD_LEN bytes */ 
  union {
    uint8_t        payload[MSG_PAYLOAD_LEN + 2];        /* raw bytes */
    uint16_t       payload16[MSG_PAYLOAD_LEN / 2];  /* rounded down! */
    ae_event_t     ae_evt;
    ae_data_t      ae_data;
    timestamp_t    timestamp;
    timeconv_t     timeinfo;
    comm_cmd_t     comm_cmd;
  };
} message_min_t;


#pragma pack()


#define MSG_SET_CRC16(msg, crc) \
  (msg)->payload[(msg)->header.payload_len] = crc & 0xff; \
  (msg)->payload[(msg)->header.payload_len + 1] = (crc >> 8) & 0xff;
#define MSG_GET_CRC16(msg)      \
  ((uint16_t)(msg)->payload[(msg)->header.payload_len] | \
   (uint16_t)(msg)->payload[(msg)->header.payload_len + 1] << 8)

/* error checks */

#if BOLT_CONF_MAX_MSG_LEN < MSG_PKT_LEN
#error "BOLT max msg length is too small"
#endif
    

#endif /* __MAIN_H__ */
