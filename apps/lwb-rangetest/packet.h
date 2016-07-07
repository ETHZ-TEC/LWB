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

/* packet structure and message type definitions */

#ifndef __PACKET_H__
#define __PACKET_H__


#define MSG_MAX_LEN             31
#define MSG_HDR_LEN             14          /* bytes */
#define MSG_PAYLOAD_LEN         (MSG_MAX_LEN - MSG_HDR_LEN)

/* extended message type */
#define MSG_EXT_HDR_LEN         6           /* without timestamp */
#define MSG_EXT_PAYLOAD_LEN     (MSG_MAX_LEN - MSG_EXT_HDR_LEN)

typedef enum {
  MSG_TYPE_INVALID = 0,
  MSG_TYPE_TIMESTAMP,
  MSG_TYPE_CC430_HEALTH,
  MSG_TYPE_ERROR,
  MSG_TYPE_WARNING,
  MSG_TYPE_INFO,
  MSG_TYPE_LWB_CMD,
  MSG_TYPE_FW_INFO,
  MSG_TYPE_FW_DATA,
  MSG_TYPE_FW_VALIDATE,
  MSG_TYPE_FW_READY,
  MSG_TYPE_FW_REQ_DATA,
  MSG_TYPE_FW_UPDATE,
  MSG_TYPE_FW_ROLLBACK,
} message_type_t;

typedef enum {
  LWB_CMD_RESUME = 0,
  LWB_CMD_PAUSE,
  LWB_CMD_SET_SCHED_PERIOD,
  LWB_CMD_SET_STATUS_PERIOD,
} lwb_cmd_t;

typedef enum {
  FW_STATUS_INIT,
  FW_STATUS_RECEIVING,
  FW_STATUS_VALIDATED,
  FW_STATUS_UPDATING,
  FW_STATUS_UPDATED,
} fw_status_type_t;
    
    

#pragma pack(1)         /* force alignment to 1 byte */

typedef struct {
  int16_t  temp;      /* temperature value in 100x °C */
  uint16_t vcc;       /* supply voltage (raw ADC value) */
  uint16_t cpu_dc;    /* cpu duty cycle in per thousands */
  uint16_t rf_dc;     /* radio duty cycle in per thousands */
  uint16_t rx_cnt;    /* reception counter (total # successfully rcvd pkts) */
  uint16_t n_rx_hops; /* RX count (CRC ok) + hop cnts of last Glossy flood */
  uint8_t  per;       /* total packet error rate in percentage */
  uint8_t  snr;       /* signal-to-noise ratio of the last reception */
  int8_t   rssi[3];   /* RSSI values of the last Glossy flood */
} health_msg_t;

/* application layer packet format (a packet is called 'message') */
typedef struct {
  struct {
    uint16_t       device_id;
    message_type_t type : 8;    /* force 1 byte */
    uint8_t        payload_len;
    uint16_t       seqnr;
    uint64_t       generation_time;
  } header;
  union {
    uint8_t        payload[MSG_PAYLOAD_LEN];
    uint16_t       payload16[MSG_PAYLOAD_LEN / 2];
    health_msg_t   health;
  };
} message_t;

/* container for any kind of data */
typedef struct {
  uint16_t         pktnr;       /* packet number (offset) */
  uint8_t          payload[MSG_EXT_PAYLOAD_LEN - 2];
} data_t;

/* alternate packet format */
typedef struct {
  struct {
    uint16_t       device_id;
    message_type_t type : 8;
    uint8_t        payload_len;
    uint16_t       seqnr;
  } header;
  data_t           data;
} message_ext_t;

#pragma pack()


#endif /* __PACKET_H__ */