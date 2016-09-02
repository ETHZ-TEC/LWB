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
 *          Tonio Gsell
 */

/* packet structure and message type definitions
 * 
 * revised packet format */

#ifndef __PACKET_H__
#define __PACKET_H__


#define MSG_PKT_LEN     (LWB_CONF_MAX_DATA_PKT_LEN - LWB_CONF_HEADER_LEN)
#define MSG_HDR_LEN     14

/* actual message length including CRC */
#define MSG_LEN(msg)    ((msg).header.payload_len + MSG_HDR_LEN + 2)

typedef enum {
  LWB_CMD_RESUME = 0,
  LWB_CMD_PAUSE,
  LWB_CMD_SET_SCHED_PERIOD,
  LWB_CMD_SET_STATUS_PERIOD,
} lwb_cmd_type_t;

/* there are 4 message classes (2 bits required to encode) */
typedef enum {
  MSG_CLASS_EVENT = 0,
  MSG_CLASS_STATUS,
  MSG_CLASS_DATA,
  MSG_CLASS_CMD,
} message_class_type_t;

/* the message type (6 bits available -> 64 different types for each class) */
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


typedef uint64_t timestamp_t;


#pragma pack(1)         /* force alignment to 1 byte */

// 34 bytes
typedef struct {
  uint32_t uptime;        /* in seconds */
  int16_t  temp;          /* temperature value in 100x °C */
  uint16_t vcc;           /* supply voltage (raw ADC value) */
  uint16_t cpu_dc;        /* cpu duty cycle in per thousands */
  uint16_t rf_dc;         /* radio duty cycle in per thousands */
  uint16_t lwb_rx_cnt;    /* RX counter (total # successfully rcvd pkts) */
  uint16_t lwb_n_rx_hops; /* RX count + hop cnts of last Glossy flood */
  uint16_t rf_per;        /* total packet error rate in percentage * 100 */
  uint8_t  rf_snr;        /* signal-to-noise ratio of the last reception */
  int8_t   lwb_rssi[3];   /* RSSI values of the last Glossy flood */
  uint16_t lwb_fsr;       /* LWB flood success rate in percentage * 100 */
  uint8_t  lwb_tx_buf;    /* number of packets in the transmit buffer */
  uint8_t  lwb_rx_buf;    /* number of packets in the receive buffer */
  uint8_t  lwb_tx_drop;   /* dropped tx packets since last health message */
  uint8_t  lwb_rx_drop;   /* dropped rx packets since last health message */
  uint8_t  lwb_bootstrap_cnt;
  uint8_t  lwb_sleep_cnt;
  uint32_t lfxt_ticks;    /* in 32kHz ticks, rollover of ~36h */
} cc430_health_t;


typedef struct {
  lwb_cmd_type_t  type : 8;
  uint16_t        target_id;
  uint16_t        value;
} lwb_cmd_t;


/* application layer packet format (a packet is called 'message') */

/* crc is calculated over the whole structure and appended to the message */
typedef struct {
  struct {
    uint16_t       device_id;   /* sender node ID */
    message_type_t type : 8;    /* force 1 byte */
    uint8_t        payload_len;
    uint16_t       seqnr;
    uint64_t       generation_time;
  } header;
  union {
    cc430_health_t cc430_health;
    lwb_cmd_t      lwb_cmd;
    uint8_t        payload[MSG_PKT_LEN - MSG_HDR_LEN];  /* raw bytes */
    uint16_t       payload16[(MSG_PKT_LEN - MSG_HDR_LEN) / 2];
  };
} message_t;

#define MSG_SET_CRC16(msg, crc) \
  (msg)->payload[(msg)->header.payload_len] = crc & 0xff; \
  (msg)->payload[(msg)->header.payload_len + 1] = (crc >> 8) & 0xff;
#define MSG_GET_CRC16(msg)      \
  ((uint16_t)(msg)->payload[(msg)->header.payload_len] | \
   (uint16_t)(msg)->payload[(msg)->header.payload_len + 1] << 8)


/* for FUTURE use: extended message type with smaller header (set last bit
 * of message type to indicate an extended message) */

#define MSG_EXT_PKT_LEN   (LWB_CONF_MAX_DATA_PKT_LEN - LWB_CONF_HEADER_LEN)
#define MSG_EXT_HDR_LEN   4
#define MSG_EXT_LEN(msg)  ((msg).header.payload_len + MSG_EXT_HDR_LEN + 2)

typedef struct {
  struct {
    uint16_t       device_id;   /* sender node ID */
    /* MSB of type must be set! */
    message_type_t type : 8;    /* force 1 byte */
    uint8_t        payload_len;
  } header;
  union {
    uint8_t        payload[MSG_EXT_PKT_LEN - MSG_EXT_HDR_LEN];  /* raw bytes */
  };
} message_ext_t;

#define MSG_EXT_SET_CRC16(msg, crc) \
  (msg)->payload[(msg)->header.payload_len] = crc & 0xff; \
  (msg)->payload[(msg)->header.payload_len + 1] = (crc >> 8) & 0xff;
#define MSG_EXT_GET_CRC16(msg)      \
  ((uint16_t)(msg)->payload[(msg)->header.payload_len] | \
   (uint16_t)(msg)->payload[(msg)->header.payload_len + 1] << 8)


#pragma pack()


#endif /* __PACKET_H__ */
