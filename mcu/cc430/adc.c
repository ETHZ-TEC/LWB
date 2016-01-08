/*
 * Copyright (c) 2015, Swiss Federal Institute of Technology (ETH Zurich).
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
 *
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

#include "platform.h"

#ifdef MCU_HAS_ADC12
/*---------------------------------------------------------------------------*/
#define ADC_TEMP_CAL1   0x1a1c   /* calibration data, located in info memory */
#define ADC_TEMP_CAL2   0x1a1a
/*---------------------------------------------------------------------------*/
static int32_t slope;    /* needed to transform the sampled values (ADC) */
/*---------------------------------------------------------------------------*/
void
adc_init(void)
{
  /* clear master bit to control the REF module with the ADC12CTL bits */
  REFCTL0 &= ~REFMSTR;
  /* enable ADC12 and REF module, set sample-and-hold time, set multiple sample
     conversion (only needed for seq-of-channels) */
  ADC12CTL0 = ADC12ON + ADC12REFON + ADC12SHT0_10 + ADC12MSC;
  /* start address is MEM0, sample-and-hold source is SC bit, sequence of
     channels (samples all channels until EOS bit is found) */
  ADC12CTL1 = ADC12SHP + ADC12CSTARTADD_0 + ADC12CONSEQ_1 + ADC12SHS_0;
  ADC12CTL2 |= ADC12RES_2;          /* 12-bit resolution */
  /* make sure the temperature sensor isn't turned off! */
  ADC12CTL2 &= ~ADC12TCOFF;
  ADC12IE = 0;                      /* disable all interrupts */
  ADC12IFG = 0;                     /* reset interrupt flags */
  /* ADC channel 10 (temperature sense), select reference Vref (1.5V) */
  ADC12MCTL0 = ADC12INCH_10 + ADC12SREF_1;
  /* ADC channel 11 (voltage sense), select reference Vref (1.5V), set EOS bit
     (this is the last channel to be sampled) */
  ADC12MCTL1 = ADC12INCH_11 + ADC12SREF_1 + ADC12EOS;
  /* ~40us delay to allow Ref to settle */
  __delay_cycles(MCLK_SPEED / 25000);

  /* use calibration data stored in info memory */
  slope = ((int32_t)(85 - 30) << 16) /
    (int32_t)(*((int16_t *)ADC_TEMP_CAL1) - *((int16_t *)ADC_TEMP_CAL2));
}
/*---------------------------------------------------------------------------*/
void
adc_get_data(uint8_t *out_data)
{
  /* enable and trigger the conversion (SC bit only important for
     sequence-of-channels mode) */
  ADC12CTL0 |= ADC12ENC + ADC12SC;
  while(ADC12CTL1 & ADC12BUSY);
  ADC12CTL0 &= ~(ADC12ENC + ADC12SC);       /* stop ADC */

  /* read out and convert the sampled value */
  out_data[0] = (uint8_t)((int32_t)(
                 ((int32_t)ADC12MEM0 - (int32_t)*((int16_t *)ADC_TEMP_CAL2)) *
                 slope) >> 16) + 30;
  out_data[1] = (uint8_t)(
                ((((uint32_t)ADC12MEM1 * 3000 >> 12) + 1) - 2000) >> 2);
  /* another way to encode the voltage as percentage:
     (uint8_t)LIMIT(((((uint32_t)ADC12MEM1 * 3000 >> 12) + 1) - 2200) / 8, 0,
     100) */
}
/*---------------------------------------------------------------------------*/
#endif /* MCU_HAS_ADC12 */