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
 *          Tonio Gsell
 */

#ifndef __CONFIG_H__
#define __CONFIG_H__


/*
 * application specific config file to override default settings
 */

/* --- Node and component ID --- */

#define HOST_ID                         16 //6

#define COMPONENT_ID                    DPP_COMPONENT_ID_CC430
#define IS_HOST                         (NODE_ID == HOST_ID)


/* --- Radio config --- */

#define RF_CONF_TX_CH                   10
#define ELWB_CONF_RF_CH_PRIMARY         RF_CONF_TX_CH
#define ELWB_CONF_RF_CH_SECONDARY       RF_CONF_TX_CH
#define RF_CONF_TX_POWER                RF1A_TX_POWER_0_dBm
#define RF_CONF_MAX_PKT_LEN             128


/* --- Network parameters --- */

#define ELWB_CONF_N_HOPS                3


/* --- eLWB config --- */

#define ELWB_CONF_SCHED_PERIOD_IDLE     15            /* period T in seconds */
#define ELWB_CONF_SCHED_PERIOD_MIN      3        /* min. allowed period in s */
#define ELWB_CONF_SCHED_CRC             1          /* append CRC to schedule */
#define ELWB_CONF_SCHED_NODE_TIMEOUT    3600        /* remove node after x s */
#define ELWB_CONF_MAX_DATA_SLOTS        40    /* max. # data slots per round */
#define ELWB_CONF_MAX_N_NODES           40        /* max. # nodes in network */
#define ELWB_CONF_DATA_ACK              1  /* use data ACKs from src to host */
/* packet and buffer size */
#define ELWB_CONF_MAX_PKT_LEN           (128 - 2)     /* subtract Glossy hdr */
#define ELWB_CONF_USE_XMEM              FRAM_CONF_ON      /* if FRAM enabled */
#define ELWB_CONF_OUT_BUFFER_SIZE       6 /* = max #pkts a src node can send */
#define ELWB_CONF_IN_BUFFER_SIZE        4  /* = max #pkts a src node can rcv */
/* timings */
#define ELWB_CONF_T_CONT                (RTIMER_SECOND_HF / 200)      /* 5ms */
#define ELWB_CONF_T_SCHED               (RTIMER_SECOND_HF / 50)      /* 20ms */
#define ELWB_CONF_T_SILENT              (120 * RTIMER_SECOND_HF)     /* 2min */
#define ELWB_CONF_T_DEEPSLEEP_LF        (RTIMER_SECOND_LF * 1800)   /* 30min */
/* retransmissions */
#define ELWB_CONF_N_TX_SCHED            3
#define ELWB_CONF_N_TX_DATA             3
/* override default packet filter (only keep pkts on src that match node_id) */ 
#define ELWB_CONF_SRC_PKT_FILTER(data)  (0)      /* don't accept any packets */
#define LWB_VERSION                     0       /* exclude default LWB impl. */

/* Glossy */
#define GLOSSY_COMMON_HEADER            0xc0


/* --- BOLT config --- */

#define BOLT_CONF_ON                    0                  /* don't use BOLT */


/* --- MISC --- */

#define WATCHDOG_CONF_ON                1
#define RTIMER_CONF_LF_UPDATE_INT       1       /* enable LFXT OVF interrupt */


/* --- DEBUG config --- */

#define DEBUG_PRINT_CONF_ON             1
#define DEBUG_PRINT_CONF_LEVEL          DEBUG_PRINT_LVL_INFO
#define DEBUG_PRINT_CONF_USE_RINGBUFFER 1
#define DEBUG_PRINT_CONF_BUFFER_SIZE    512  /* debug print buffer size in b */
#define DEBUG_PRINT_CONF_MSG_LEN        90  /* max debug msg length per line */
#define DEBUG_PRINT_CONF_PRINT_NODEID   1
#define DEBUG_CONF_STACK_GUARD          (SRAM_START + 3588)
                                         /* -> .bss + .dec size */
#define DEBUG_CONF_ISR_IND_PIN          COM_GPIO3     /* pin 9 on DBG header */
#define DEBUG_PRINT_CONF_TASK_ACT_PIN   COM_GPIO2     /* pin 8 on DBG header */
#define APP_TASK_ACT_PIN                COM_GPIO2
#define ELWB_CONF_TASK_ACT_PIN          COM_GPIO3
#define GLOSSY_START_PIN                COM_GPIO3
#define RF_GDO2_PIN                     COM_GPIO1


/* --- Compile time parameter checks --- */

#if DPP_MSG_PKT_LEN > ELWB_CONF_MAX_PKT_LEN
#error "ELWB_CONF_MAX_PKT_LEN is too small"
#endif


/* non-volatile configuration and stats (length must be NVCFG_CONF_BLOCK_SIZE
 * bytes) */
typedef struct {
  uint16_t  node_id;
  uint8_t   rst_cnt;
  uint8_t   dbg_flags;     /* debug flags:
                              0x01    set to turn LEDs off
                              0x02    disable eLWB ACK mechanism on source */
  uint8_t   tx_pwr;        /* transmit power for radio */ 
  uint8_t   spare;
} config_t;

extern config_t cfg;           /* most important config parameters and stats */


#endif /* __CONFIG_H__ */
