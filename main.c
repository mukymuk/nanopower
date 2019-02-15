#include "global.h"

#include "board.h"
#include "lcd.h"
#include "adc.h"
#include "meter.h"

int main(void)
{
    board_init();

    lcd_init();
    adc_init();
    meter_init();

    board_lcd_disp(true);
    while(1)
    {
        meter_update();
    }
}
