#include <stdio.h>
#include "lcd.h"

void app_main(void)
{
    uint16_t data = 0x00FF;
    lcd_init();
    lcd_write_rect(0, 0, 1, 1, &data);
//  lcd_exit();
}
