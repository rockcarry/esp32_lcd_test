#ifndef __LCD_H__
#define __LCD_H__

#include <stdint.h>

void lcd_init (void);
void lcd_exit (void);
void lcd_onoff(int onoff);
void lcd_write_rect(int x, int y, int w, int h, uint16_t *data);
void lcd_write_done(void);

#endif
