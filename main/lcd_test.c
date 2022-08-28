#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lcd.h"

#define RGB565(r, g, b) ((((g) & 0x07) << 13) | (((b) & 0xF8) << 5) | ((r) & 0xF8) | ((g) >> 5)) // (((r) & 0xF8) << 8) | (((g) & 0xFC) << 3) | (((b) & 0xF1) >> 3)

static uint16_t screen_buf[SCREEN_WIDTH * SCREEN_HEIGHT] = {0};

static void putpixel(int x, int y, uint16_t c)
{
    screen_buf[y * SCREEN_WIDTH + x] = c;
}

static void vpost(void)
{
    int  i;
    for (i = 0; i < SCREEN_HEIGHT; i += MAX_TRANSFER_LINES) {
        lcd_write_rect(0, i, SCREEN_WIDTH, MAX_TRANSFER_LINES, screen_buf + i * SCREEN_WIDTH);
        lcd_write_done();
    }
}

void line(int x1, int y1, int x2, int y2, uint16_t c) {
    int dx, dy, d, e;

    dx = abs(x1 - x2);
    dy = abs(y1 - y2);

    if ((dx >= dy && x1 > x2) || (dx < dy && y1 > y2)) {
        d = x1; x1 = x2; x2 = d;
        d = y1; y1 = y2; y2 = d;
    }

    if (dx >= dy) {
        d = y2 - y1 > 0 ? 1 : -1;
        for (e = dx/2; x1 < x2; x1++, e += dy) {
            if (e >= dx) e -= dx, y1 += d;
            putpixel(x1, y1, c);
        }
    } else {
        d = x2 - x1 > 0 ? 1 : -1;
        for (e = dy/2; y1 < y2; y1++, e += dx) {
            if (e >= dy) e -= dy, x1 += d;
            putpixel(x1, y1, c);
        }
    }
    putpixel(x2, y2, c);
}

void app_main(void)
{
    int cx = 0, cy = 0, lx = 0, ly = 0, pressed = 0, released = 0;

    printf("hello lcd test !\n");
    lcd_init();
    vpost();

    while (1) {
        pressed = tp_get_xy(&cx, &cy);
        if (pressed) {
 //         printf("pressed: %d, x: %3d, y: %3d\n", pressed, cx, cy);
            if (released == 5) {
                released = 0;
                memset(screen_buf, 0, sizeof(screen_buf));
            } else {
                line(lx, ly, cx, cy, RGB565(0, 0, 255));
                putpixel(lx, ly, RGB565(255, 0, 0));
                putpixel(cx, cy, RGB565(255, 0, 0));
            }
            lx = cx, ly = cy;
            vpost();
        } else {
            if (released < 5) released++;
        }
        vTaskDelay(15 / portTICK_RATE_MS);
    }

    lcd_exit();
}
