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
#define GPIO_TOUCH_SPI_CS   46
#define GPIO_TOUCH_PENIRQ   2
#define TOUCH_PRESS_THRES   800
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

static spi_device_handle_t s_spi_dev_lcd = NULL;
static spi_device_handle_t s_spi_dev_tp  = NULL;
static spi_transaction_t   s_trans[6]    = {};

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
    lcd_cmd(s_spi_dev_lcd, 0x04);
    t.length = 8 * 3;
    t.flags  = SPI_TRANS_USE_RXDATA;
    t.user   = (void*)1;
    ret = spi_device_polling_transmit(s_spi_dev_lcd, &t);
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

    spi_device_interface_config_t devcfg_lcd = {
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
    ret = spi_bus_add_device(LCD_SPI_HOST, &devcfg_lcd, &s_spi_dev_lcd);
    ESP_ERROR_CHECK(ret);

    id = lcd_get_id();
    printf("lcd id: %08X\n", id);

    for (i = 0; i + 1 < sizeof(s_ili9341_init_data); ) {
        lcd_cmd(s_spi_dev_lcd, s_ili9341_init_data[i]);
        if (s_ili9341_init_data[i + 1] == 0xFF) {
            vTaskDelay(100 / portTICK_RATE_MS); // delay 100ms
            i += 2;
        } else {
            if (i + 2 + s_ili9341_init_data[i + 1] < sizeof(s_ili9341_init_data) && s_ili9341_init_data[i + 1]) {
                lcd_data(s_spi_dev_lcd, s_ili9341_init_data + i + 2, s_ili9341_init_data[i + 1]);
            }
            i += 2 + s_ili9341_init_data[i + 1];
        }
    }

    if (GPIO_DISP_LCD_BKL > 0) {
        gpio_set_direction(GPIO_DISP_LCD_BKL, GPIO_MODE_OUTPUT);
        gpio_set_level(GPIO_DISP_LCD_BKL, 1);
    }

    spi_device_interface_config_t devcfg_tp = {
        .clock_speed_hz = 2 * 1000 * 1000,
        .mode           = 0,
        .spics_io_num   = GPIO_TOUCH_SPI_CS,
        .queue_size     = 1,
        .command_bits   = 8,
        .address_bits   = 0,
        .flags = SPI_DEVICE_HALFDUPLEX | SPI_DEVICE_NO_DUMMY,
    };
    ret = spi_bus_add_device(LCD_SPI_HOST, &devcfg_tp , &s_spi_dev_tp );
    ESP_ERROR_CHECK(ret);

    gpio_config_t irq_config = {
        .pin_bit_mask = BIT64(GPIO_TOUCH_PENIRQ),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ret = gpio_config(&irq_config);
    ESP_ERROR_CHECK(ret);
}

void lcd_exit(void)
{
    spi_bus_remove_device(s_spi_dev_lcd);
    spi_bus_remove_device(s_spi_dev_tp );
    spi_bus_free(LCD_SPI_HOST);
}

void lcd_onoff(int onoff)
{
    uint8_t data = 0x08;
    lcd_cmd(s_spi_dev_lcd, onoff ? 0x11 : 0x10);
    lcd_data(s_spi_dev_lcd, &data, 1);
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
        ret = spi_device_queue_trans(s_spi_dev_lcd, s_trans + i, portMAX_DELAY);
        assert(ret == ESP_OK);
    }
}

void lcd_write_done(void)
{
    spi_transaction_t *trans;
    int ret, i;
    for (i = 0; i < 6; i++) {
        ret = spi_device_get_trans_result(s_spi_dev_lcd, &trans, portMAX_DELAY);
        assert(ret == ESP_OK);
    }
}

#define CMD_X_READ  0b11010000
#define CMD_Y_READ  0b10010000
#define CMD_Z1_READ 0b10110000
#define CMD_Z2_READ 0b11000000
static int tp_read_reg(uint8_t reg, uint16_t *data)
{
    uint8_t buf[2];
    spi_transaction_t trans = {
        .length    = (sizeof(buf) + sizeof(reg)) * 8,
        .rxlength  =  sizeof(buf) * 8,
        .cmd       = reg,
        .rx_buffer = buf,
        .flags     = 0
    };

    // read - send first byte as command
    esp_err_t ret = spi_device_transmit(s_spi_dev_tp, &trans);
    *data = (buf[0] << 8) | (buf[1] << 0);
    return ret == ESP_OK ? 0 : -1;
}

static int tp_is_pressed(void)
{
    int16_t z1, z2, z;
    int     ret;
    if (gpio_get_level(GPIO_TOUCH_PENIRQ)) return 0;
    ret = tp_read_reg(CMD_Z1_READ, (uint16_t*)&z1);
    ret|= tp_read_reg(CMD_Z2_READ, (uint16_t*)&z2);
    if (ret != 0) return 0;
    z1 >>= 3;
    z2 >>= 3;
    z = z1 + 4096 - z2;
    return z > TOUCH_PRESS_THRES;
}

static void tp_filter(int *x, int *y)
{
    #define TP_FILTER_SIZE  4
    static int s_tp_filter_weight[TP_FILTER_SIZE] = { 1, 3, 3, 5 };
    static int s_tp_filter_buffer[TP_FILTER_SIZE][2];
    static int s_tp_filter_head = 0;
    static int s_tp_filter_tail = 0;
    static int s_tp_filter_num  = 0;
    int sum_x, sum_y, sum_div, i;

    if (!x) {
        s_tp_filter_head = s_tp_filter_tail = s_tp_filter_num = 0;
        return;
    }

    s_tp_filter_buffer[s_tp_filter_tail][0] = *x;
    s_tp_filter_buffer[s_tp_filter_tail][1] = *y;
    s_tp_filter_tail = (s_tp_filter_tail + 1) % TP_FILTER_SIZE;
    if (s_tp_filter_num < TP_FILTER_SIZE) s_tp_filter_num++;
    else s_tp_filter_head = s_tp_filter_tail;
    sum_x = sum_y = sum_div = 0;
    for (i = 0; i < s_tp_filter_num; i++) {
        sum_x   += s_tp_filter_buffer[(s_tp_filter_head + i) % TP_FILTER_SIZE][0] * s_tp_filter_weight[i];
        sum_y   += s_tp_filter_buffer[(s_tp_filter_head + i) % TP_FILTER_SIZE][1] * s_tp_filter_weight[i];
        sum_div += s_tp_filter_weight[i];
    }
    *x = sum_x / sum_div;
    *y = sum_y / sum_div;
}

int tp_get_xy(int *x, int *y)
{
    int16_t x16, y16;
    int     ret;

    if (!tp_is_pressed()) {
        tp_filter(NULL, NULL);
        return 0;
    }

    ret = tp_read_reg(CMD_X_READ, (uint16_t*)&x16);
    ret|= tp_read_reg(CMD_Y_READ, (uint16_t*)&y16);
    if (ret != 0) return 0;

#if LCD_LANDSCAPE
    *x = y16 >> 4;
    *y = x16 >> 4;
    *x = (*x - 92) * SCREEN_WIDTH  / (1870 - 92);
    *y = (*y - 85) * SCREEN_HEIGHT / (1920 - 85);
    *y = SCREEN_HEIGHT - *y;
#else
    *x = x16 >> 4;
    *y = y16 >> 4;
    *x = (*x - 85) * SCREEN_WIDTH  / (1920 - 85);
    *y = (*y - 92) * SCREEN_HEIGHT / (1870 - 92);
#endif

    if (*x < 0) *x = 0;
    if (*y < 0) *y = 0;
    if (*x >= SCREEN_WIDTH ) *x = SCREEN_WIDTH  - 1;
    if (*y >= SCREEN_HEIGHT) *y = SCREEN_HEIGHT - 1;
    tp_filter(x, y);
    return  1;
}
