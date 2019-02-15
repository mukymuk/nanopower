#ifndef __BOARD_H_INCL__
#define __BOARD_H_INCL__

#include "i2c.h"

void board_init( void );
void board_spi_write32( const void *pv_data, uint32_t length );
void board_lcd_disp( bool on );
void board_sleep(void);
void board_i2c_write( uint8_t i2c_addr, const void *pv_data, uint32_t length, bool restart );

typedef void (*board_i2c_cb_t)(void);

void board_i2c_read( uint8_t i2c_addr, void *pv_data, uint32_t length, bool restart, board_i2c_cb_t p_board_i2c_cb );

void board_enable_sample_timer( bool enable );

void board_adc_timer_isr_unlock(void);
void board_adc_timer_isr_lock(void);
void board_i2c_isr_unlock( void );
void board_i2c_isr_lock( void );

bool board_button_event( void );
// These three defines setup the sampling rate used by the ADC.
// The sample timer prescaler and count are both used to produce the output frequency
#define BOARD_SAMPLE_TIMER_PRESCALE_REG       TMR_PRES_4
#define BOARD_SAMPLE_TIMER_PRESCALE           4      // corrosponds to BOARD_SAMPLE_TIMER_PRESCALE_REG
#define BOARD_ADC_SAMPLE_FREQ_HZ              64    // choose a 2^N value to simplify some math
                                                    // non-2^N values will require reviewing the adc and meter modules.

#endif
