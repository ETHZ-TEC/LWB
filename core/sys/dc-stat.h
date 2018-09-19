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

/*
 * Duty cycle statistics: a minimal implementation of ENERGEST for CPU and
 * RF duty cycle with reduced overhead and some limitations:
 * - Will only work if the active cycles are < 2s (= 1 timer period @ 32kHz)
 * - Rollover will occur after 2^32 ticks of activity (~ 1.5 days)
 * - Requires rtimer-ext.h
 * Optimized for 16-bit CPU architecture. Requires only 34B of heap memory.
 *
 * The duty cycle value is between 0 (no load) and 10000 (full load).
 */

#ifndef DC_STAT_H_
#define DC_STAT_H_


#ifndef DCSTAT_CONF_ON
#define DCSTAT_CONF_ON          0
#endif /* DCSTAT_CONF_ON */

#if DCSTAT_CONF_ON

/* rtimer value (64 bits) */
#ifndef DCSTAT_RTIMER_NOW
#define DCSTAT_RTIMER_NOW()     rtimer_now_lf()
#endif /* DCSTAT_RTIMER_NOW */

/* HW timer value (16 bits) */
#ifndef DCSTAT_RTIMER_NOW_HW
#define DCSTAT_RTIMER_NOW_HW()  rtimer_now_lf_hw()
#endif /* DCSTAT_RTIMER_NOW_HW */

/* define default (dummy) hooks / actions */
#ifdef DCSTAT_CPU_ON_PIN
#define DCSTAT_CPU_ON_ACT       PIN_SET(DCSTAT_CPU_ON_PIN)
#define DCSTAT_CPU_OFF_ACT      PIN_CLR(DCSTAT_CPU_ON_PIN)
#else /* DCSTAT_CPU_ON_PIN */
#define DCSTAT_CPU_ON_ACT
#define DCSTAT_CPU_OFF_ACT
#endif /* DCSTAT_CPU_ON_PIN */

#ifdef DCSTAT_RF_ON_PIN
#define DCSTAT_RF_ON_ACT        PIN_SET(DCSTAT_RF_ON_PIN)
#define DCSTAT_RF_OFF_ACT       PIN_CLR(DCSTAT_RF_ON_PIN)
#else /* DCSTAT_RF_ON_PIN */
#define DCSTAT_RF_ON_ACT
#define DCSTAT_RF_OFF_ACT
#endif /* DCSTAT_RF_ON_PIN */

#ifdef DCSTAT_RF_TX_ON_PIN
#define DCSTAT_RF_TX_ON_ACT     PIN_SET(DCSTAT_RF_TX_ON_PIN)
#define DCSTAT_RF_TX_OFF_ACT    PIN_CLR(DCSTAT_RF_TX_ON_PIN)
#else /* DCSTAT_RF_TX_ON_PIN */
#define DCSTAT_RF_TX_ON_ACT
#define DCSTAT_RF_TX_OFF_ACT
#endif /* DCSTAT_RF_TX_ON_PIN */

#ifdef DCSTAT_RF_RX_ON_PIN
#define DCSTAT_RF_RX_ON_ACT    PIN_SET(DCSTAT_RF_RX_ON_PIN)
#define DCSTAT_RF_RX_OFF_ACT   PIN_CLR(DCSTAT_RF_RX_ON_PIN)
#else /* DCSTAT_RF_RX_ON_PIN */
#define DCSTAT_RF_RX_ON_ACT
#define DCSTAT_RF_RX_OFF_ACT
#endif /* DCSTAT_RF_RX_ON_PIN */

#define DCSTAT_CPU_ON   { \
    if(dc_stat_isr == 0) { \
      dc_stat_starttime_cpu = (uint16_t)DCSTAT_RTIMER_NOW_HW(); \
      DCSTAT_CPU_ON_ACT; \
    } \
    dc_stat_isr++; \
  }

#define DCSTAT_CPU_OFF  { \
    if(dc_stat_isr == 1) { \
      dc_stat_sum_cpu += (uint16_t)(DCSTAT_RTIMER_NOW_HW() - dc_stat_starttime_cpu); \
      DCSTAT_CPU_OFF_ACT; \
    } \
    if(dc_stat_isr) { \
      dc_stat_isr--; \
    } \
  }
#define DCSTAT_CPU_DC   (uint16_t)(dc_stat_sum_cpu / ((DCSTAT_RTIMER_NOW() - dc_stat_resettime) / 10000))

#define DCSTAT_RF_ON    { \
    if(!dc_stat_starttime_rf) { \
      dc_stat_starttime_rf = (uint16_t)DCSTAT_RTIMER_NOW_HW(); \
    } \
    DCSTAT_RF_ON_ACT; \
  }

#define DCSTAT_RF_OFF   { \
    if(dc_stat_starttime_rf) { \
      dc_stat_sum_rf += (uint16_t)(DCSTAT_RTIMER_NOW_HW() - dc_stat_starttime_rf); \
      dc_stat_starttime_rf = 0; \
    } \
    DCSTAT_RF_OFF_ACT; \
  }

#define DCSTAT_RF_DC    (uint16_t)(dc_stat_sum_rf / ((DCSTAT_RTIMER_NOW() - dc_stat_resettime) / 10000))

#define DCSTAT_RFTX_ON  { \
    if(!dc_stat_starttime_rf_tx) { \
      dc_stat_starttime_rf_tx = (uint16_t)DCSTAT_RTIMER_NOW_HW(); \
    } \
    DCSTAT_RF_TX_ON_ACT; \
  }

#define DCSTAT_RFTX_OFF { \
    if(dc_stat_starttime_rf_tx) { \
      dc_stat_sum_rf_tx += (uint16_t)(DCSTAT_RTIMER_NOW_HW() - dc_stat_starttime_rf_tx); \
      dc_stat_starttime_rf_tx = 0; \
    } \
    DCSTAT_RF_TX_OFF_ACT; \
  }

#define DCSTAT_RFTX_DC  (uint16_t)(dc_stat_sum_rf_tx / ((DCSTAT_RTIMER_NOW() - dc_stat_resettime) / 10000))

#define DCSTAT_RFRX_ON  { \
    if(!dc_stat_starttime_rf_rx) { \
      dc_stat_starttime_rf_rx = (uint16_t)DCSTAT_RTIMER_NOW_HW(); \
    } \
    DCSTAT_RF_RX_ON_ACT; \
  }

#define DCSTAT_RFRX_OFF { \
    if(dc_stat_starttime_rf_rx) { \
      dc_stat_sum_rf_rx += (uint16_t)(DCSTAT_RTIMER_NOW_HW() - dc_stat_starttime_rf_rx); \
      dc_stat_starttime_rf_rx = 0; \
    } \
    DCSTAT_RF_RX_OFF_ACT; \
  }

#define DCSTAT_RFRX_DC  (uint16_t)(dc_stat_sum_rf_rx / ((DCSTAT_RTIMER_NOW() - dc_stat_resettime) / 10000))

#define DCSTAT_RESET    { \
    dc_stat_sum_cpu   = 0; \
    dc_stat_sum_rf    = 0; \
    dc_stat_sum_rf_tx = 0; \
    dc_stat_sum_rf_rx = 0; \
    dc_stat_resettime = DCSTAT_RTIMER_NOW(); \
    dc_stat_isr       = 0; \
  }

extern uint16_t dc_stat_starttime_cpu;
extern uint16_t dc_stat_starttime_rf;
extern uint16_t dc_stat_starttime_rf_tx;
extern uint16_t dc_stat_starttime_rf_rx;
extern uint32_t dc_stat_sum_cpu;
extern uint32_t dc_stat_sum_rf;
extern uint32_t dc_stat_sum_rf_tx;
extern uint32_t dc_stat_sum_rf_rx;
extern uint64_t dc_stat_resettime;
extern uint16_t dc_stat_isr;


#else /* DCSTAT_CONF_ON */

#define DCSTAT_CPU_ON
#define DCSTAT_CPU_OFF
#define DCSTAT_CPU_DC   0

#define DCSTAT_RF_ON
#define DCSTAT_RF_OFF
#define DCSTAT_RF_DC    0

#define DCSTAT_RFTX_ON
#define DCSTAT_RFTX_OFF
#define DCSTAT_RFTX_DC  0

#define DCSTAT_RFRX_ON
#define DCSTAT_RFRX_OFF
#define DCSTAT_RFRX_DC  0

#define DCSTAT_RESET

#endif /* DCSTAT_CONF_ON */


#endif /* DC_STAT_H_ */
