#ifndef __ADC_H_INCL__
#define __ADC_H_INCL__

#include "board.h"

#define ADC_REFERENCE_VOLTS 2.048
#define ADC_SAMPLE_RANGE    4096.0
#define ADC_VOLTS_PER_BIT   (ADC_REFERENCE_VOLTS/ADC_SAMPLE_RANGE)

#define ADC_SAMPLE_TO_UPDATE_RATIO  (BOARD_ADC_SAMPLE_FREQ_HZ/METER_UPDATE_FREQ_HZ)


typedef enum
{
    adc_input_voltage,
    adc_input_current,
    adc_input_battery,
    adc_input_testpoint
}
adc_input_t;


void adc_set_input( adc_input_t input );
void adc_enable( bool enable  );

void adc_init( void );
void adc_sample_timer_isr(void);

int16_t * adc_get_samples( void );
void adc_release_samples(void);
float_t adc_transform( int16_t sample );

void adc_sample_timer_isr(void);

#endif
