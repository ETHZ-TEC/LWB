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

/* packet structure and message type definitions */

#ifndef __PACKET_H__
#define __PACKET_H__


#define MSG_PKT_LEN     (LWB_CONF_MAX_DATA_PKT_LEN - LWB_CONF_HEADER_LEN - \
                         GLOSSY_MAX_HEADER_LEN)     /* 62 - 3 - 4 = 55 bytes */
#define MSG_HDR_LEN     16
#define MSG_PAYLOAD_LEN (MSG_PKT_LEN - MSG_HDR_LEN - 2)  /* 55 - 16 - 2 = 37 */

/* actual message length including CRC */
#define MSG_LEN(msg)    ((msg).header.payload_len + MSG_HDR_LEN + 2)

/* special (reserved) device IDs */
#define DEVICE_ID_SINK        0
#define DEVICE_ID_BROADCAST   0xffff


typedef enum {
  COMM_CMD_LWB_RESUME = 0,
  COMM_CMD_LWB_PAUSE,
  COMM_CMD_LWB_SET_ROUND_PERIOD,
  COMM_CMD_LWB_SET_HEALTH_PERIOD,
  COMM_CMD_LWB_SET_TX_PWR,
  COMM_CMD_NODE_RESET = 50,
} comm_cmd_type_t;

/* the message type (7 bits available, MSB is reserved) */
typedef enum {
  /* general message types */
  MSG_TYPE_INVALID = 0,
  MSG_TYPE_TIMESYNC = 1,
  MSG_TYPE_LOG = 2,
  MSG_TYPE_NODE_INFO = 3,

  /* message types concerning the communication processor */
  MSG_TYPE_COMM_CMD = 10,
  MSG_TYPE_COMM_HEALTH = 11,

  MSG_TYPE_APP_FW = 50, /* application processor FW update */
} message_type_t;


typedef uint64_t timestamp_t;


#pragma pack(1)         /* force alignment to 1 byte */


#define MSG_COMM_HEALTH_LEN   36    /* bytes */
typedef struct {
  uint32_t uptime;        /* in seconds */
  int16_t  temp;          /* temperature value in 100x °C */
  uint16_t vcc;           /* supply voltage (raw ADC value) */
  uint16_t cpu_dc;        /* cpu duty cycle (0..10000) */
  uint16_t rf_dc;         /* radio duty cycle (0..10000) */
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
  uint16_t lwb_t_to_rx;   /* time[us] to first rx (offset to glossy_start) */
  uint16_t lwb_t_flood;   /* flood duration [us] */
  uint8_t  lwb_n_rx_started;  /* # preambles+sync det. in last flood */
  uint8_t  test_byte;     /* packet delivery rate */
} comm_health_t;


typedef struct {
  comm_cmd_type_t type : 8;
  uint16_t        value;
} comm_cmd_t;


#define MSG_NODE_INFO_LEN   37    /* bytes */
typedef struct {
  uint8_t   component_id;
  uint8_t   rst_cnt;
  uint8_t   rst_flag;
  char      mcu_desc[12];     /* MCU description, i.g. 'CC430F5147' */
  char      compiler_desc[3]; /* description/name */
  uint16_t  compiler_ver;
  uint32_t  compile_date;
  char      fw_name[8];       /* name of the firmware/application */
  uint16_t  fw_ver;
  uint8_t   sw_rev_id[3];     /* repository revision number (GIT or SVN) */
} node_info_t;


#define LOG_EVENT_HDR_LEN   4
typedef struct {
  uint8_t           component_id;
  log_event_type_t  type : 8;
  uint16_t          value;
  /* depending on the event type, there is extra data */
  uint8_t           extra_data[MSG_PAYLOAD_LEN - 4];
} log_event_t;


/* application layer packet format (a packet is called 'message') */

/* crc is calculated over the whole structure and appended to the message */
typedef struct {
  /* header: MSG_HDR_LEN bytes */
  struct {
    uint16_t       device_id;   /* sender node ID */
    message_type_t type : 8;    /* force 1 byte */
    uint8_t        payload_len;
    uint16_t       target_id;
    uint16_t       seqnr;
    uint64_t       generation_time;
  } header;
  /* none of the union members may be larger than MSG_PAYLOAD_LEN bytes
   * except for 'payload' */
  union {
    comm_health_t  comm_health;
    comm_cmd_t     comm_cmd;
    node_info_t    node_info;
    log_event_t    log_event;
    /* add +2 to increase overall structure size to include the crc! */
    uint8_t        payload[MSG_PAYLOAD_LEN + 2];   /* raw bytes */
    uint16_t       payload16[MSG_PAYLOAD_LEN / 2]; /* rounded down! */
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

#define MSG_EXT_PKT_LEN      (LWB_CONF_MAX_DATA_PKT_LEN - LWB_CONF_HEADER_LEN)
#define MSG_EXT_HDR_LEN      4
#define MSG_EXT_PAYLOAD_LEN  (MSG_EXT_PKT_LEN - MSG_EXT_HDR_LEN - 2)
#define MSG_EXT_LEN(msg)     ((msg).header.payload_len + MSG_EXT_HDR_LEN + 2)

typedef struct {
  struct {
    uint16_t       device_id;   /* sender node ID */
    /* MSB of type must be set! */
    message_type_t type : 8;    /* force 1 byte */
    uint8_t        payload_len;
  } header;
  union {
    uint8_t        payload[MSG_EXT_PAYLOAD_LEN + 2];  /* raw bytes */
  };
} message_ext_t;

#define MSG_EXT_SET_CRC16(msg, crc) \
  (msg)->payload[(msg)->header.payload_len] = crc & 0xff; \
  (msg)->payload[(msg)->header.payload_len + 1] = (crc >> 8) & 0xff;
#define MSG_EXT_GET_CRC16(msg)      \
  ((uint16_t)(msg)->payload[(msg)->header.payload_len] | \
   (uint16_t)(msg)->payload[(msg)->header.payload_len + 1] << 8)


#pragma pack()


/* error check (max. message length) */
#if (MSG_PAYLOAD_LEN < MSG_COMM_HEALTH_LEN) || \
    (MSG_PAYLOAD_LEN < MSG_NODE_INFO_LEN)
#error "payload of message_t is too big"
#endif

#if BOLT_CONF_MAX_MSG_LEN < MSG_PKT_LEN
#error "BOLT max msg length is too small"
#endif


#endif /* __PACKET_H__ */
