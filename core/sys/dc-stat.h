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

/* Duty cycle statistics: a minimal implementation of ENERGEST for CPU and
 * RF duty cycle with reduced overhead and accuracy.
 *
 * The duty cycle value is between 0 (no load) and 10000 (full load).
 */

#ifndef __DC_STAT_H__
#define __DC_STAT_H__


#ifndef DCSTAT_CONF_ON
#define DCSTAT_CONF_ON    0
#endif /* DCSTAT_CONF_ON */


#if DCSTAT_CONF_ON

#define DCSTAT_CPU_ON   dcstat_cpu_on()
#define DCSTAT_CPU_OFF  dcstat_cpu_off()
#define DCSTAT_CPU_DC   dcstat_get_cpu_dc()

#define DCSTAT_RF_ON    dcstat_rf_on()
#define DCSTAT_RF_OFF   dcstat_rf_off()
#define DCSTAT_RF_DC    dcstat_get_rf_dc()

#define DCSTAT_RESET    dcstat_reset()

void dcstat_cpu_on(void);
void dcstat_cpu_off(void);
void dcstat_rf_on(void);
void dcstat_rf_off(void);
uint16_t dcstat_get_cpu_dc(void);
uint16_t dcstat_get_rf_dc(void);
void dcstat_reset(void);

#else /* DCCALC_CONF_ON */

#define DCSTAT_CPU_ON
#define DCSTAT_CPU_OFF
#define DCSTAT_CPU_DC   0

#define DCSTAT_RF_ON
#define DCSTAT_RF_OFF
#define DCSTAT_RF_DC    0

#define DCSTAT_RESET

#endif /* DCCALC_CONF_ON */


#endif /* __DC_STAT_H__ */
