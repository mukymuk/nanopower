#include "global.h"
#include "meter.h"
#include "adc.h"
#include "lcd.h"

// resistor values from schematic
#define R9      0.05
#define R10     113000.0
#define R11     1000000.0

typedef enum
{
    meter_mode_voltage,
    meter_mode_current,
    meter_mode_frequency,
    meter_mode_last
}
meter_mode_t;

static meter_mode_t s_mode;
static adc_input_t s_input;
static uint8_t s_font_width, s_font_height;

void meter_init( void )
{
    s_mode = meter_mode_voltage;
    s_input = adc_input_testpoint;
    lcd_font( &s_font_width, &s_font_height );
    adc_set_input( s_input );
    adc_enable(true);
}


static int32_t sum( const int16_t * p_samples, uint32_t count )
{
    uint32_t i;
    int32_t sum = 0;
    for(i=0;i<count;i++)
        sum += p_samples[i];
    return sum;
}


#define UROUND_MILLI  1000.0f
#define UROUND_MICRO  (UROUND_MILLI*UROUND_MILLI)

static void render_measurement( float_t adc_in_v )
{
    uint8_t c;
    const char * p_units;
    const char * p_mode;
    float_t m;
    lcd_clear();
    switch( s_mode )
    {
        case meter_mode_voltage:
        {
            if( s_input == adc_input_testpoint )
                m = adc_in_v;
            else if( s_input == adc_input_voltage )
                m = adc_in_v * (float_t)((R10+R11)/R10);
            m = roundf( m * UROUND_MICRO ) / UROUND_MICRO;
            if( m < 0 )
                c = 7;
            else
                c = 6;
            lcd_printf( (LCD_WIDTH-c*s_font_width)/2-4, (LCD_HEIGHT-s_font_height)/2, "%1.3fV", m );
            break;
        }
        case meter_mode_current:
        {
            m = roundf( adc_in_v * (float_t)(1000.0/R9) );
            if( m < 0 )
                c = 6;
            else
                c = 5;
            lcd_printf( (LCD_WIDTH-c*s_font_width)/2-4, (LCD_HEIGHT-s_font_height)/2, "%3.0fmA", m );
            break;
        }
        case meter_mode_frequency:
        {
            break;
        }
    }
//    UG_FontSelect( &FONT_12X16 );
    lcd_update();
}


void meter_update(void)
{
    int16_t * p_samples;
    if( board_button_event() )
    {
        adc_enable(false);
        s_mode++;
        if(s_mode == meter_mode_last)
            s_mode = 0;
        switch( s_mode )
        {
            case meter_mode_voltage:
            {
                adc_set_input(adc_input_testpoint);
                break;
            }
            case meter_mode_current:
            {
                adc_set_input(adc_input_current);
                break;
            }
            case meter_mode_frequency:
            {
                adc_enable(false);
                break;
            }
        }
        adc_enable(true);
    }
    if( lcd_ready() && (p_samples = adc_get_samples()) )
    {
        int32_t m = sum( p_samples, ADC_SAMPLE_TO_UPDATE_RATIO );
        float_t adc_in_v = (float_t)m * (float_t)(ADC_VOLTS_PER_BIT/ADC_SAMPLE_TO_UPDATE_RATIO);
     //   if( adc_in_v > 1.0f )
     //       while(1);
        adc_release_samples();
        render_measurement( adc_in_v );
    }
}



