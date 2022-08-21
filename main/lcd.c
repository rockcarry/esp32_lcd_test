#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lcd.h"

#define GPIO_DISP_LCD_RST   -1
#define GPIO_DISP_LCD_BKL   -1
#define GPIO_DISP_DAT_CMD   9
#define GPIO_DISP_SPI_CS    10
#define GPIO_DISP_SPI_MOSI  11
#define GPIO_DISP_SPI_CLK   12
#define GPIO_DISP_SPI_MISO  13
#define LCD_SPI_HOST        SPI2_HOST

static uint8_t s_ili9341_init_data[] = {
    0xCF, 3,  0x00, 0x83, 0X30,
    0xED, 4,  0x64, 0x03, 0X12, 0X81,
    0xE8, 3,  0x85, 0x01, 0x79,
    0xCB, 5,  0x39, 0x2C, 0x00, 0x34, 0x02,
//  0xF6, 3,  0x00, 0x00, 0x20, // little endian is not support for spi mode
    0xF7, 1,  0x20,
    0xEA, 2,  0x00, 0x00,
    0xC0, 1,  0x26,          /* power control */
    0xC1, 1,  0x11,          /* power control */
    0xC5, 2,  0x35, 0x3E,    /* vcom control */
    0xC7, 1,  0xBE,          /* vcin control */
#if LCD_LANDSCAPE
    0x36, 1,  0x00,          /* memory access control */
#else
    0x36, 1,  0x60,          /* memory access control */
#endif
    0x3A, 1,  0x55,          /* pixel format set */
    0xB1, 2,  0x00, 0x1B,
    0xF2, 1,  0x08,
    0x26, 1,  0x01,
    0xE0, 15, 0x1F, 0x1A, 0x18, 0x0A, 0x0F, 0x06, 0x45, 0X87, 0x32, 0x0A, 0x07, 0x02, 0x07, 0x05, 0x00,
    0XE1, 15, 0x00, 0x25, 0x27, 0x05, 0x10, 0x09, 0x3A, 0x78, 0x4D, 0x05, 0x18, 0x0D, 0x38, 0x3A, 0x1F,
    0x2A, 4,  0x00, 0x00, 0x00, 0xEF,
    0x2B, 4,  0x00, 0x00, 0x01, 0x3f,
    0x2C, 0,
    0xB7, 1,  0x07,
    0xB6, 4,  0x0A, 0x82, 0x27, 0x00,
    0x11, 0xFF,
    0x29, 0xFF,
};

static spi_device_handle_t s_spi_dev  = NULL;
static spi_transaction_t   s_trans[6] = {};

static void lcd_cmd(spi_device_handle_t spidev, uint8_t cmd)
{
    int ret;
    spi_transaction_t t = {};
    t.length    = 8;
    t.tx_buffer = &cmd;
    t.user      = (void*)0;
    ret = spi_device_polling_transmit(spidev, &t);
    assert(ret == ESP_OK);
}

static void lcd_data(spi_device_handle_t spidev, uint8_t *data, int len)
{
    int ret;
    spi_transaction_t t = {};
    t.length    = 8 * len;
    t.tx_buffer = data;
    t.user      = (void*)1;
    ret = spi_device_polling_transmit(spidev, &t);
    assert(ret == ESP_OK);
}

static void pretransfer_callback(spi_transaction_t *t)
{
    int dc = (int)t->user;
    gpio_set_level(GPIO_DISP_DAT_CMD, dc);
}

static uint32_t lcd_get_id(void)
{
    int ret;
    spi_transaction_t t = {};
    lcd_cmd(s_spi_dev, 0x04);
    t.length = 8 * 3;
    t.flags  = SPI_TRANS_USE_RXDATA;
    t.user   = (void*)1;
    ret = spi_device_polling_transmit(s_spi_dev, &t);
    assert(ret == ESP_OK);
    return *(uint32_t*)t.rx_data;
}

void lcd_init(void)
{
    int ret, id, i;

    spi_bus_config_t buscfg = {
        .miso_io_num    = GPIO_DISP_SPI_MISO,
        .mosi_io_num    = GPIO_DISP_SPI_MOSI,
        .sclk_io_num    = GPIO_DISP_SPI_CLK,
        .quadwp_io_num  = -1,
        .quadhd_io_num  = -1,
        .max_transfer_sz= SCREEN_WIDTH * MAX_TRANSFER_LINES * 2,
    };

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 80 * 1000 * 1000,
        .mode           = 0,
        .spics_io_num   = GPIO_DISP_SPI_CS,
        .queue_size     = 6,
        .pre_cb         = pretransfer_callback,
    };

    if (GPIO_DISP_DAT_CMD > 0) {
        gpio_set_direction(GPIO_DISP_DAT_CMD, GPIO_MODE_OUTPUT);
        gpio_set_level(GPIO_DISP_DAT_CMD, 0);
    }
    if (GPIO_DISP_LCD_RST > 0) {
        gpio_set_direction(GPIO_DISP_LCD_RST, GPIO_MODE_OUTPUT);
        gpio_set_level(GPIO_DISP_LCD_RST, 0);
        vTaskDelay(100 / portTICK_RATE_MS);
        gpio_set_level(GPIO_DISP_LCD_RST, 1);
        vTaskDelay(100 / portTICK_RATE_MS);
    }

    ret = spi_bus_initialize(LCD_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK(ret);
    ret = spi_bus_add_device(LCD_SPI_HOST, &devcfg, &s_spi_dev);
    ESP_ERROR_CHECK(ret);

    id = lcd_get_id();
    printf("lcd id: %08X\n", id);

    for (i = 0; i + 1 < sizeof(s_ili9341_init_data); ) {
        lcd_cmd (s_spi_dev, s_ili9341_init_data[i]);
        if (s_ili9341_init_data[i + 1] == 0xFF) {
            vTaskDelay(100 / portTICK_RATE_MS); // delay 100ms
            i += 2;
        } else {
            if (i + 2 + s_ili9341_init_data[i + 1] < sizeof(s_ili9341_init_data) && s_ili9341_init_data[i + 1]) {
                lcd_data(s_spi_dev, s_ili9341_init_data + i + 2, s_ili9341_init_data[i + 1]);
            }
            i += 2 + s_ili9341_init_data[i + 1];
        }
    }

    if (GPIO_DISP_LCD_BKL > 0) {
        gpio_set_direction(GPIO_DISP_LCD_BKL, GPIO_MODE_OUTPUT);
        gpio_set_level(GPIO_DISP_LCD_BKL, 1);
    }
}

void lcd_exit(void)
{
    spi_bus_remove_device(s_spi_dev);
    spi_bus_free(LCD_SPI_HOST);
}

void lcd_onoff(int onoff)
{
    uint8_t data = 0x08;
    lcd_cmd(s_spi_dev, onoff ? 0x11 : 0x10);
    lcd_data(s_spi_dev, &data, 1);
    if (GPIO_DISP_LCD_BKL > 0) gpio_set_level(GPIO_DISP_LCD_BKL, onoff);
}

void lcd_write_rect(int x, int y, int w, int h, uint16_t *data)
{
    int ret, i;

    memset(&s_trans, 0, sizeof(s_trans));
    for (i = 0; i < 6; i++) {
        s_trans[i].length = (i & 1) ? 8 * 4 : 8 * 1;
        s_trans[i].user   = (void*)!!(i & 1);
        s_trans[i].flags  = SPI_TRANS_USE_TXDATA;
    }
    s_trans[0].tx_data[0] = 0x2A;             // column address set
    s_trans[1].tx_data[0] = x >> 8;           // start col high
    s_trans[1].tx_data[1] = x >> 0;           // start col low
    s_trans[1].tx_data[2] = (x + w - 1) >> 8; // end col high
    s_trans[1].tx_data[3] = (x + w - 1) >> 0; // end col low

    s_trans[2].tx_data[0] = 0x2B;             // page address set
    s_trans[3].tx_data[0] = y >> 8;           // start page high
    s_trans[3].tx_data[1] = y >> 0;           // start page low
    s_trans[3].tx_data[2] = (y + h - 1) >> 8; // end page high
    s_trans[3].tx_data[3] = (y + h - 1) >> 0; // end page low

    s_trans[4].tx_data[0] = 0x2C;             // memory write
    s_trans[5].tx_buffer  = data;             // finally send the line data
    s_trans[5].length     = w * h * 2 * 8;    // data length in bits
    s_trans[5].flags      = 0;                // undo SPI_TRANS_USE_TXDATA flag

    for (i = 0; i < 6; i++) {
        ret = spi_device_queue_trans(s_spi_dev, s_trans + i, portMAX_DELAY);
        assert(ret == ESP_OK);
    }
}

void lcd_write_done(void)
{
    spi_transaction_t *trans;
    int ret, i;
    for (i = 0; i < 6; i++) {
        ret = spi_device_get_trans_result(s_spi_dev, &trans, portMAX_DELAY);
        assert(ret == ESP_OK);
    }
}
