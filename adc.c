#include "global.h"
#include "adc.h"
#include "board.h"
#include "max1161x_regs.h"
#include "meter.h"


#define I2C_ADDR    0b01100110
#define I2C_READ    1
#define I2C_WRITE   0

static i2c_req_t s_i2c_req;

static int16_t  s_sample_buffer[2][ADC_SAMPLE_TO_UPDATE_RATIO];
static uint8_t  s_sample_ndx;
static uint8_t  s_sample_buffer_ndx;
static int16_t * s_p_ready;

void adc_init( void )
{
}

void adc_enable( bool enable  )
{
    const max1161x_setup_t setup_on =
    {
        .reg = MAX116X_REG_SETUP,
        .sel2 = MAX1161X_SEL2_INTERNAL_REFERENCE,
        .sel1 = MAX1161X_SEL1_OUTPUT_REFERENCE,
        .sel0 = MAX1161X_SEL0_REFERENCE_ON,
        .clk = MAX1161X_CLK_INTERNAL,
        .bip_uni = MAX1161X_BIP_UNI_BIPOLAR,
        .rst = MAX1161X_RST_DISASSERT
    };
    const max1161x_setup_t setup_low_power =
    {
        .reg = MAX116X_REG_SETUP,
        .sel2 = MAX1161X_SEL2_INTERNAL_REFERENCE,
        .sel1 = MAX1161X_SEL1_OUTPUT_REFERENCE,
        .sel0 = MAX1161X_SEL1_REFERENCE_OFF,
        .clk = MAX1161X_CLK_INTERNAL,
        .bip_uni = MAX1161X_BIP_UNI_BIPOLAR,
        .rst = MAX1161X_RST_DISASSERT
    };
    if( enable )
    {
        board_i2c_write( I2C_ADDR, &setup_on, sizeof(setup_on), NULL );
        board_enable_sample_timer(enable);
    }
    else
    {
        board_enable_sample_timer(enable);
        board_i2c_write( I2C_ADDR, &setup_low_power, sizeof(setup_low_power), NULL );
        s_sample_buffer_ndx = 0;
        s_sample_ndx = 0;
    }
}

void adc_set_input( adc_input_t input )
{
    static const max1161x_config_t channel_config[4] =
    {
        {
            // voltage input
            .reg = MAX116X_REG_CONFIG,
            .scan = MAX1161X_SCAN_CS,
            .cs = MAX1161X_CS_AIN(0),
            .sgl_dif = MAX1161X_SGL_DIF_DIFFERENTIAL
        },
        {
            // current input
            .reg = MAX116X_REG_CONFIG,
            .scan = MAX1161X_SCAN_CS,
            .cs = MAX1161X_CS_AIN(2),
            .sgl_dif = MAX1161X_SGL_DIF_DIFFERENTIAL
        },
        {
            // battery input
            .reg = MAX116X_REG_CONFIG,
            .scan = MAX1161X_SCAN_CS,
            .cs = MAX1161X_CS_AIN(4),
            .sgl_dif = MAX1161X_SGL_DIF_DIFFERENTIAL
        },
        {
            // testpoint input
            .reg = MAX116X_REG_CONFIG,
            .scan = MAX1161X_SCAN_CS,
            .cs = MAX1161X_CS_AIN(6),
            .sgl_dif = MAX1161X_SGL_DIF_DIFFERENTIAL
        }
    };
    max1161x_config_t config = channel_config[input];
    board_adc_timer_isr_lock();
    board_i2c_write( I2C_ADDR, &config, sizeof(config), false );
    board_adc_timer_isr_unlock();
}

int16_t * adc_get_samples( void )
{
    if( s_p_ready )
    {
    }
    return s_p_ready;   // return the most recently collected sample set.
                        // consits of ADC_SAMPLE_TO_UPDATE_RATIO samples.
                        // remains NULL until a full buffer is available.
}

void adc_release_samples( void )
{
    board_i2c_isr_lock();
    s_p_ready = NULL;   // indicates the sample buffer is free for reuse by i2c_cb_isr()
    board_i2c_isr_unlock();
}

static void i2c_cb_isr( void )
{
    // i2c isr context.
 //   if( !s_sample_buffer[s_sample_buffer_ndx][s_sample_ndx] )
  //      while( 1 );
   s_sample_ndx++;
    if( s_sample_ndx >= ADC_SAMPLE_TO_UPDATE_RATIO )
    {
        s_sample_ndx = 0;
        if( !s_p_ready )
        {
            s_p_ready = &s_sample_buffer[s_sample_buffer_ndx][0];
            for(uint8_t i=0;i<ADC_SAMPLE_TO_UPDATE_RATIO;i++)
            {
                s_p_ready[i] = __REV16(s_p_ready[i]);  // handle endian conversion
                if( s_p_ready[i] & 0x0800 )
                    s_p_ready[i] |= 0xF000;
                else
                    s_p_ready[i] &= 0x0FFF;
            }

            s_sample_buffer_ndx ^= 1;
            memset( &s_sample_buffer[s_sample_buffer_ndx], 0, sizeof(s_sample_buffer[s_sample_buffer_ndx]));
        }
    }
    board_adc_timer_isr_unlock();   // debugging can cause the timer isr to get hit multiple times before i2c_cb_isr()
                                    // this prevents that.  no effect when normally running.
}

void adc_sample_timer_isr(void)
{
    // timer isr context.
    board_adc_timer_isr_lock(); // debugging can cause the timer isr to get hit multiple times before i2c_cb_isr()
                                // this prevents that.  no effect when normally running.
    board_i2c_read( I2C_ADDR, &s_sample_buffer[s_sample_buffer_ndx][s_sample_ndx], sizeof(int16_t), false, i2c_cb_isr );

}

float_t adc_transform( int16_t sample )
{
    return (float_t)sample * (float_t)(ADC_REFERENCE_VOLTS/ADC_SAMPLE_RANGE) ;
}
