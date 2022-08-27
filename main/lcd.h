#ifndef __LCD_H__
#define __LCD_H__

#include <stdint.h>

#define LCD_LANDSCAPE       1
#if LCD_LANDSCAPE
#define SCREEN_WIDTH        320
#define SCREEN_HEIGHT       240
#define MAX_TRANSFER_LINES  48
#else
#define SCREEN_WIDTH        240
#define SCREEN_HEIGHT       320
#define MAX_TRANSFER_LINES  64
#endif

void lcd_init (void);
void lcd_exit (void);
void lcd_onoff(int onoff);
void lcd_write_rect(int x, int y, int w, int h, uint16_t *data);
void lcd_write_done(void);
int  tp_get_xy(int *x, int *y);

#endif
