#include <stdlib.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lcd.h"

#define RGB565(r, g, b) ((((g) & 0x07) << 13) | (((b) & 0xF8) << 5) | ((r) & 0xF8) | ((g) >> 5)) // (((r) & 0xF8) << 8) | (((g) & 0xFC) << 3) | (((b) & 0xF1) >> 3)

static uint16_t screen_buf[SCREEN_WIDTH * SCREEN_HEIGHT];

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

void app_main(void)
{
    uint16_t color[3] = { RGB565(255, 0, 0), RGB565(0, 255, 0), RGB565(0, 0, 255) };
    int      i, j;

    printf("hello lcd test !\n");
    lcd_init();

    for (i = 0; i < SCREEN_HEIGHT; i++) {
        for (j = 0; j < SCREEN_WIDTH; j++) {
            putpixel(j, i, RGB565(j, i, j));
        }
    }
    vpost();
    vTaskDelay(1000 / portTICK_RATE_MS);

    for (i = 0; ; i++) {
        for (j = 0; j < sizeof(screen_buf) / sizeof(uint16_t); j++) screen_buf[j] = color[i % 3];
        vpost();
        vTaskDelay(1000 / portTICK_RATE_MS);
    }

    lcd_exit();
}
