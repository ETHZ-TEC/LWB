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

#include <stdint.h>
#include "platform.h"

/*---------------------------------------------------------------------------*/
static uint32_t dc_stat_starttime_cpu   = 0,
                dc_stat_starttime_rf    = 0,
                dc_stat_starttime_rf_tx = 0,
                dc_stat_starttime_rf_rx = 0;
static uint64_t dc_stat_sum_cpu   = 0,
                dc_stat_sum_rf    = 0,
                dc_stat_sum_rf_tx = 0,
                dc_stat_sum_rf_rx = 0,
                dc_stat_resettime = 0;
static uint16_t dc_stat_isr = 0;         /* for CPU */
/*---------------------------------------------------------------------------*/
void dcstat_cpu_on(void)
{
  if(dc_stat_isr == 0) {
    dc_stat_starttime_cpu = (uint32_t)rtimer_now_lf();
  }
  dc_stat_isr++;
}
/*---------------------------------------------------------------------------*/
void dcstat_cpu_off(void)
{
  if(dc_stat_isr == 1) {
    uint32_t elapsed = (uint32_t)rtimer_now_lf() - dc_stat_starttime_cpu;
    dc_stat_sum_cpu += elapsed;
  }
  if(dc_stat_isr) {
    dc_stat_isr--;
  }
}
/*---------------------------------------------------------------------------*/
void dcstat_rf_on(void)
{
  dc_stat_starttime_rf = (uint32_t)rtimer_now_lf();
}
/*---------------------------------------------------------------------------*/
void dcstat_rf_off(void)
{
  if(dc_stat_starttime_rf) {
    uint32_t elapsed = (uint32_t)rtimer_now_lf() - dc_stat_starttime_rf;
    dc_stat_sum_rf += elapsed;
    dc_stat_starttime_rf = 0;
  }
}
/*---------------------------------------------------------------------------*/
void dcstat_rf_tx_on(void)
{
  dc_stat_starttime_rf_tx = (uint32_t)rtimer_now_lf();
}
/*---------------------------------------------------------------------------*/
void dcstat_rf_tx_off(void)
{
  if(dc_stat_starttime_rf_tx) {
    uint32_t elapsed = (uint32_t)rtimer_now_lf() - dc_stat_starttime_rf_tx;
    dc_stat_sum_rf_tx += elapsed;
    dc_stat_starttime_rf_tx = 0;
  }
}
/*---------------------------------------------------------------------------*/
void dcstat_rf_rx_on(void)
{
  dc_stat_starttime_rf_rx = (uint32_t)rtimer_now_lf();
}
/*---------------------------------------------------------------------------*/
void dcstat_rf_rx_off(void)
{
  if(dc_stat_starttime_rf_rx) {
    uint32_t elapsed = (uint32_t)rtimer_now_lf() - dc_stat_starttime_rf_rx;
    dc_stat_sum_rf_rx += elapsed;
    dc_stat_starttime_rf_rx = 0;
  }
}
/*---------------------------------------------------------------------------*/
uint16_t dcstat_get_cpu_dc(void)
{
  return (dc_stat_sum_cpu * 10000 / (rtimer_now_lf() - dc_stat_resettime));
}
/*---------------------------------------------------------------------------*/
uint16_t dcstat_get_rf_dc(void)
{
  return (dc_stat_sum_rf * 10000 / (rtimer_now_lf() - dc_stat_resettime));
}
/*---------------------------------------------------------------------------*/
uint16_t dcstat_get_rf_tx_dc(void)
{
  return (dc_stat_sum_rf_tx * 10000 / (rtimer_now_lf() - dc_stat_resettime));
}
/*---------------------------------------------------------------------------*/
uint16_t dcstat_get_rf_rx_dc(void)
{
  return (dc_stat_sum_rf_rx * 10000 / (rtimer_now_lf() - dc_stat_resettime));
}
/*---------------------------------------------------------------------------*/
void dcstat_reset(void)
{
  dc_stat_sum_cpu   = 0;
  dc_stat_sum_rf    = 0;
  dc_stat_sum_rf_tx = 0;
  dc_stat_sum_rf_rx = 0;
  dc_stat_isr       = 0;
  dc_stat_resettime = rtimer_now_lf();
}
/*---------------------------------------------------------------------------*/
