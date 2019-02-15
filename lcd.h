#ifndef __LCD_H_INCL__
#define __LCD_H_INCL__

#include "ugui.h"

#define LCD_WIDTH			128
#define LCD_HEIGHT			128

void lcd_init( void );
void lcd_clear();
void lcd_update(void);
void lcd_update_timer_isr(void);
void lcd_spi_write_complete_isr(void);
void lcd_printf( uint8_t x, uint8_t y, const char * p_format, ... );
bool lcd_ready( void );
void lcd_font( uint8_t *p_width, uint8_t *p_height );

#endif
