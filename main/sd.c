#include <stdlib.h>
#include <stdio.h>
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "sd.h"

#define PIN_NUM_MISO    0
#define PIN_NUM_MOSI    4
#define PIN_NUM_CLK     3
#define PIN_NUM_CS      8
#define MOUNT_POINT     "/sdcard"

static sdmmc_host_t  s_sdmmc_host = SDSPI_HOST_DEFAULT();
static sdmmc_card_t *s_sdmmc_card = NULL;

void sd_init(void)
{
    esp_err_t ret;
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files              = 5,
        .allocation_unit_size   = 16 * 1024
    };
    spi_bus_config_t bus_cfg = {
        .mosi_io_num     = PIN_NUM_MOSI,
        .miso_io_num     = PIN_NUM_MISO,
        .sclk_io_num     = PIN_NUM_CLK,
        .quadwp_io_num   = -1,
        .quadhd_io_num   = -1,
        .max_transfer_sz = 4000,
    };
    s_sdmmc_host.slot = SPI3_HOST;
    ret = spi_bus_initialize(s_sdmmc_host.slot, &bus_cfg, SDSPI_DEFAULT_DMA);
    if (ret != ESP_OK) {
        printf("failed to initialize bus.\n");
        return;
    }

    // this initializes the slot without card detect (CD) and write protect (WP) signals.
    // modify slot_config.gpio_cd and slot_config.gpio_wp if your board has these signals.
    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = PIN_NUM_CS;
    slot_config.host_id = s_sdmmc_host.slot;

    printf("mounting filesystem ...\n");
    ret = esp_vfs_fat_sdspi_mount(MOUNT_POINT, &s_sdmmc_host, &slot_config, &mount_config, &s_sdmmc_card);
    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            printf("failed to mount filesystem.\n"
                   "if you want the card to be formatted, set the CONFIG_EXAMPLE_FORMAT_IF_MOUNT_FAILED menuconfig option.\n");
        } else {
            printf("failed to initialize the card (%s).\n"
                   "make sure SD card lines have pull-up resistors in place.\n", esp_err_to_name(ret));
        }
        return;
    }
    printf("filesystem mounted !\n");

    // card has been initialized, print its properties
    sdmmc_card_print_info(stdout, s_sdmmc_card);
}

void sd_exit(void)
{
    // all done, unmount partition and disable SPI peripheral
    if (s_sdmmc_card) {
        esp_vfs_fat_sdcard_unmount(MOUNT_POINT, s_sdmmc_card);
        printf("card unmounted !\n");
    }

    // deinitialize the bus after all devices are removed
    spi_bus_free(s_sdmmc_host.slot);
}
