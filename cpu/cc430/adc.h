/**
 * @file
 * @brief configure the ADC to sample the battery voltage and temperature
 * @author rdaforno 
 */
 
#ifndef __ADC_H
#define __ADC_H


void adc_init(void);
uint8_t adc_get_data(uint8_t* out_data);


#endif
