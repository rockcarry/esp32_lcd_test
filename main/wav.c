#include <stdlib.h>
#include <stdio.h>
#include "driver/i2s.h"
#include "wav.h"

#define IIS_SCLK    16
#define IIS_LCLK    7
#define IIS_DSIN    6
#define IIS_DOUT    15
#define IIS_CH      1
#define SAMPLE_RATE 16000

void wav_start(int out)
{
    i2s_config_t i2s_config = {
        .mode                 = (i2s_mode_t)(I2S_MODE_MASTER | (out ? I2S_MODE_TX : I2S_MODE_RX)),
        .sample_rate          = SAMPLE_RATE,
        .bits_per_sample      = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format       = I2S_CHANNEL_FMT_RIGHT_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags     = ESP_INTR_FLAG_LOWMED,
        .dma_buf_count        = 5,
        .dma_buf_len          = 320,
        .bits_per_chan        = I2S_BITS_PER_SAMPLE_16BIT,
    };
    i2s_pin_config_t pin_config = {
        .mck_io_num   = -1,
        .bck_io_num   = IIS_SCLK,
        .ws_io_num    = IIS_LCLK,
        .data_out_num = IIS_DSIN,
        .data_in_num  = IIS_DOUT,
    };
    i2s_driver_install(IIS_CH, &i2s_config, 0, NULL);
    i2s_set_pin(IIS_CH, &pin_config);
    i2s_zero_dma_buffer(IIS_CH);
}

void wav_stop(void)
{
    i2s_driver_uninstall(IIS_CH);
}

int wav_write(void *pcm, int size)
{
    size_t n = 0;
    i2s_write(IIS_CH, pcm, size, &n, portMAX_DELAY);
    return n;
}

int wav_read(void *pcm, int size)
{
    size_t n = 0;
    i2s_read(IIS_CH, pcm, size, &n, portMAX_DELAY);
    return n;
}
