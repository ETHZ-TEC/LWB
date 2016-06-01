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
#include "contiki.h"

static int32_t slope;    /* needed to transform the sampled values (ADC) */

/* calibration data, located in info memory */
#define ADC_TEMP_1_5V_30   0x1a1a   /* value at 30°C for 1.5V ref */
#define ADC_TEMP_1_5V_85   0x1a1c   /* value at 85°C for 1.5V ref */
#define ADC_TEMP_2_5V_30   0x1a22   /* value at 30°C for 2.5V ref */
#define ADC_TEMP_2_5V_85   0x1a24   /* value at 85°C for 2.5V ref */
#define ADC_TEMP_2_0V_30   0x1a1e   /* value at 30°C for 2.0V ref */
#define ADC_TEMP_2_0V_85   0x1a20   /* value at 85°C for 2.0V ref */
/*---------------------------------------------------------------------------*/
#ifdef MCU_HAS_ADC12
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
  slope = (5500) / 
          (*(int16_t*)ADC_TEMP_1_5V_85 - *(int16_t*)ADC_TEMP_1_5V_30);
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
                 ((int32_t)ADC12MEM0 - (int32_t)*((int16_t *)ADC_TEMP_1_5V_30))
                 * slope) / 100) + 30;
  out_data[1] = (uint8_t)(
                ((((uint32_t)ADC12MEM1 * 3000 >> 12) + 1) - 2000) >> 2);
  /* another way to encode the voltage as percentage:
     (uint8_t)LIMIT(((((uint32_t)ADC12MEM1 * 3000 >> 12) + 1) - 2200) / 8, 0,
     100) */
}
/*---------------------------------------------------------------------------*/
int16_t
adc_get_temp(void)
{    
  if(ADC12CTL0 & ADC12ON) {
    ADC12CTL0 |= ADC12ENC + ADC12SC; 
    /* wait until sample / conversion operation has completed */
    while (ADC12CTL0 & ADC12BUSY);
    return (((int32_t)((int16_t)ADC12MEM0 - *(int16_t*)ADC_TEMP_1_5V_30) *
            slope)) + 3000;
  }
  return 0;
}
/*---------------------------------------------------------------------------*/
int16_t
adc_get_vcc(void)
{    
  if(ADC12CTL0 & ADC12ON) {
    ADC12CTL0 |= ADC12ENC + ADC12SC; 
    /* wait until sample / conversion operation has completed */
    while (ADC12CTL0 & ADC12BUSY);    
    return ADC12MEM1;
  }
  return 0;
}
/*---------------------------------------------------------------------------*/
#elif defined(MCU_HAS_ADC10)
/*---------------------------------------------------------------------------*/
void
adc_init(void)
{
  /* configure ADC10 - pulse sample mode */
  ADC10CTL0 = ADC10SHT_8 + ADC10ON;    /* 256 ADC10CLKs sample and hold time */
  /* for accurate readings, sampling time must be at least 30us (~150cycles) */
  /* ADC10SHS_0 = start trigger is SC bit */
  /* ADC10SSEL_1 = ACLK, ADC10SSEL_0 = MODCLK (~4.8MHz) */
  ADC10CTL1 = ADC10SHP + ADC10SHS_0 + ADC10DIV_0 + ADC10CONSEQ_2 + ADC10SSEL_0;
  ADC10CTL2 |= ADC10RES;                    /* 10-bit resolution */
  ADC10MCTL0 = ADC10SREF_1 + ADC10INCH_10;  // A10, internal Vref+
  
  /* configure internal reference */
  while(REFCTL0 & REFGENBUSY);              /* wait if ref generator busy */                                         
  REFCTL0 |= REFVSEL_1 + REFON;             /* select internal ref = 2.0V */
                                                
  /* values for the 2.0V reference, see datasheet p.105 */
  slope = ((int32_t)5500 * 32) / 
          (*(int16_t*)ADC_TEMP_2_0V_85 - *(int16_t*)ADC_TEMP_2_0V_30);
            
  //ADC10IE |= ADC10IE0; 
}
/*---------------------------------------------------------------------------*/
int16_t
adc_get_temp(void)
{  
  if(ADC10CTL0 & ADC10ON) {
    ADC10MCTL0 = ADC10SREF_1 + ADC10INCH_10;
    ADC10CTL0 |= ADC10ENC + ADC10SC; 
    /* wait until sample / conversion operation has completed */
    while (ADC10CTL0 & ADC10BUSY);
    ADC10CTL0 &= ~ADC10ENC;
    int16_t adcval = ADC10MEM0;
    return (((int32_t)(adcval - *(int16_t*)ADC_TEMP_2_0V_30) *
            slope) >> 5) + 3000;
  }
  return 0;  
}
/*---------------------------------------------------------------------------*/
int16_t
adc_get_vcc(void)
{  
  /* measured: (AVCC - AVSS) / 2 
   * conversion: Vcc = 2 * Vref * ADC_VAL / 1024 */
  if(ADC10CTL0 & ADC10ON) {
    ADC10MCTL0 = ADC10SREF_1 + ADC10INCH_11;
    ADC10CTL0 |= ADC10ENC + ADC10SC; 
    /* wait until sample / conversion operation has completed */
    while (ADC10CTL0 & ADC10BUSY);
    ADC10CTL0 &= ~ADC10ENC;
    return ADC10MEM0;
  }
  return 0;  
}
/*---------------------------------------------------------------------------*/
#endif /* MCU_HAS_ADCx */


