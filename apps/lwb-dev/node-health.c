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

/* generate node health packets */

#include "main.h"

/*---------------------------------------------------------------------------*/
uint8_t
get_node_health(comm_health_t* out_data)
{
  static int16_t          temp = 0;
  static rtimer_clock_t   last_energest_rst = 0;
  static uint16_t         last_rx_drop = 0,
                          last_tx_drop = 0;
  const lwb_statistics_t* lwb_stats = lwb_get_stats();

  while(REFCTL0 & REFGENBUSY);
  REFCTL0 |= REFON;
  while(REFCTL0 & REFGENBUSY);
  __delay_cycles(MCLK_SPEED / 25000);                /* let REF settle */

  temp = (temp + adc_get_temp()) / 2;      /* moving average (LP filter) */
  out_data->temp = temp;
  out_data->vcc  = adc_get_vcc();
  REFCTL0 &= ~REFON;             /* shut down REF module to save power */

  rtimer_clock_t now      = rtimer_now_lf();

  glossy_get_rssi(out_data->lwb_rssi);
  out_data->rf_snr        = glossy_get_snr();
  out_data->lwb_rx_cnt    = glossy_get_n_pkts_crcok();
  out_data->rf_per        = glossy_get_per();
  out_data->lwb_n_rx_hops = glossy_get_n_rx() |
                            (glossy_get_relay_cnt() << 4);
  out_data->lwb_fsr       = glossy_get_fsr();
  out_data->cpu_dc        = (uint16_t)
                            (energest_type_time(ENERGEST_TYPE_CPU) *
                            1000 / (now - last_energest_rst));
  out_data->rf_dc         = (uint16_t)
                            ((energest_type_time(ENERGEST_TYPE_TRANSMIT) +
                            energest_type_time(ENERGEST_TYPE_LISTEN)) *
                            1000 / (now - last_energest_rst));
  out_data->lwb_tx_buf    = lwb_tx_buffer_state();
  out_data->lwb_rx_buf    = lwb_rx_buffer_state();
  out_data->lwb_tx_drop   = lwb_stats->txbuf_drop - last_tx_drop;
  out_data->lwb_rx_drop   = lwb_stats->rxbuf_drop - last_rx_drop;
  out_data->lwb_sleep_cnt = lwb_stats->sleep_cnt;
  out_data->lwb_bootstrap_cnt = lwb_stats->bootstrap_cnt;
  //out_data->lfxt_ticks    = now;
  out_data->uptime        = now / XT1CLK_SPEED;
  out_data->lwb_n_rx_started = glossy_get_n_rx_started();
  out_data->lwb_t_flood   = (uint16_t)(glossy_get_flood_duration() * 100 /325);
  out_data->lwb_t_to_rx   = (uint16_t)(glossy_get_t_to_first_rx() * 100 / 325);

  /* reset values */
  last_energest_rst  = now;
  energest_type_set(ENERGEST_TYPE_CPU, 0);
  energest_type_set(ENERGEST_TYPE_TRANSMIT, 0);
  energest_type_set(ENERGEST_TYPE_LISTEN, 0);
  last_rx_drop = lwb_stats->rxbuf_drop;
  last_tx_drop = lwb_stats->txbuf_drop;

  return (sizeof(comm_health_t) + MSG_HDR_LEN);
}
/*---------------------------------------------------------------------------*/
