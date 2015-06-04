 
#include "contiki.h"

static int32_t slope;               // needed to transform the sampled values (ADC)


/**
 * @brief initialize the ADC12
 */
void adc_init(void) {

    REFCTL0   &= ~REFMSTR;    // clear master bit to control the REF module with the ADC12CTL bits
    ADC12CTL0  = ADC12ON + ADC12REFON + ADC12SHT0_10 + ADC12MSC;   // enable ADC12 and REF module, set sample-and-hold time, set multiple sample conversion (only needed for seq-of-channels)
    ADC12CTL1  = ADC12SHP + ADC12CSTARTADD_0 + ADC12CONSEQ_1 + ADC12SHS_0; // start address is MEM0, sample-and-hold source is SC bit, sequence of channels (samples all channels until EOS bit is found)
    ADC12CTL2 |= ADC12RES_2;        // 12-bit resolution
    ADC12CTL2 &= ~ADC12TCOFF;       // make sure the temperature sensor isn't turned off!
    ADC12IE    = 0;                 // disable all interrupts
    ADC12IFG   = 0;                 // reset interrupt flags
    ADC12MCTL0 = ADC12INCH_10 + ADC12SREF_1;            // ADC channel 10 (temperature sense), select reference Vref (1.5V)
    ADC12MCTL1 = ADC12INCH_11 + ADC12SREF_1 + ADC12EOS; // ADC channel 11 (voltage sense), select reference Vref (1.5V), set EOS bit (this is the last channel to be sampled)
    __delay_cycles(MCLK_SPEED / 25000);                 // ~40us delay to allow Ref to settle
    
    // use calibration data stored in info memory
    slope = ((int32_t)(85 - 30) << 16) / (int32_t)(*((int16_t*)0x1a1c) - *((int16_t*)0x1a1a));
}


/**
 * @brief get the battery voltage and temperature
 * @param[out] out_data the output buffer; the 1st byte will contain the temperature, the 2nd the encoded voltage Venc (Vcc = Venc x 4 + 2000) 
 */
uint8_t adc_get_data(uint8_t* out_data) {

    ADC12CTL0 |= ADC12ENC + ADC12SC;        // enable and trigger the conversion (SC bit only important for sequence-of-channels mode)
    while (ADC12CTL1 & ADC12BUSY);
    ADC12CTL0 &= ~(ADC12ENC + ADC12SC);     // stop ADC
    
    // read out and convert the sampled value
    out_data[0] = (uint8_t)((int32_t)(((int32_t)ADC12MEM0 - (int32_t)*((int16_t*)0x1a1a)) * slope) >> 16) + 30;
    out_data[1] = (uint8_t)(((((uint32_t)ADC12MEM1 * 3000 >> 12) + 1) - 2000) >> 2);
    // another way to encode the voltage as percentage: (uint8_t)LIMIT(((((uint32_t)ADC12MEM1 * 3000 >> 12) + 1) - 2200) / 8, 0, 100) 
    
    return 2;
}
