#include "global.h"
#include "lcd.h"
#include "board.h"
#include "mxc_device.h" // for rbit

#include <stdarg.h>
#include <stdio.h>

#define LCD_BYTES_PER_LINE  (LCD_WIDTH/8)

#define FONT    FONT_16X26

#pragma pack(1)

typedef struct
{
    uint8_t mode;
    uint8_t gate_line_address;
    uint8_t data[LCD_BYTES_PER_LINE];
}
lcd_line_t;

typedef struct
{
    lcd_line_t line[LCD_HEIGHT];
    uint16_t final_data_transfer_period;
}
lcd_frame_t;

#pragma pack()

static lcd_frame_t s_frame;
static UG_GUI s_ug_gui;
static bool s_ready;

static void write_pixel( UG_S16 x, UG_S16 y, UG_COLOR c )
{
    uint8_t byte_offset = x / 8;
    uint8_t bit_offset = x - byte_offset * 8;
    uint8_t pixel = 0x80 >> bit_offset;
    s_frame.line[y].data[byte_offset] &= ~pixel;
}

static void frame_init( void )
{
    uint32_t i;
    for( i = 0; i < LCD_HEIGHT; i++ ) s_frame.line[i].gate_line_address = __RBIT( i + 1 ) >> 24;
}

void lcd_font( uint8_t *p_width, uint8_t *p_height )
{
    *p_width = s_ug_gui.font.char_width;
    *p_height = s_ug_gui.font.char_height;
}

void lcd_init( void )
{
    UG_Init( &s_ug_gui, write_pixel, LCD_WIDTH, LCD_HEIGHT );
    UG_SelectGUI( &s_ug_gui );
    UG_FontSelect( &FONT );
    lcd_clear();
    lcd_update();
}

void lcd_update( void )
{
    s_ready = false;
    s_frame.line[0].mode = 0x80;
    board_spi_write32( &s_frame, sizeof(s_frame) ); // must complete before this function is called again
}

void lcd_clear()
{
    uint32_t * p = (uint32_t*)&s_frame;
    for( uint32_t i = 0; i < sizeof(s_frame) >> 2; i++ ) p[i] = 0xFFFFFFFF;
    frame_init();
}

void lcd_spi_write_complete_isr( void )
{
    s_ready = true;
}

bool lcd_ready( void )
{
    return s_ready;
}

void lcd_printf( uint8_t x, uint8_t y, const char * p_format, ... )
{
    static uint8_t buf[LCD_WIDTH * LCD_HEIGHT];
    va_list args;
    va_start( args, p_format );
    vsnprintf( buf, sizeof(buf), p_format, args );
    UG_PutString( x, y, buf );
    va_end( args );
}

