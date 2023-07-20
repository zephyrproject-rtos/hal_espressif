/*
 * SPDX-FileCopyrightText: 2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include <stdint.h>
#include <bootutil/bootutil_log.h>
#include "sdkconfig.h"
#include "soc/spi_periph.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "bootloader_init.h"
#include "bootloader_common.h"

#include "bootloader_flash_config.h"
#include "bootloader_flash.h"
#include "bootloader_flash_priv.h"

esp_image_header_t WORD_ALIGNED_ATTR bootloader_image_hdr;

esp_err_t bootloader_read_bootloader_header(void)
{
    /* load bootloader image header */
    if (bootloader_flash_read(ESP_BOOTLOADER_OFFSET, &bootloader_image_hdr,
                              sizeof(esp_image_header_t), true) != ESP_OK) {
        BOOT_LOG_ERR("Failed to load bootloader image header!");
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t bootloader_common_check_chip_validity(
                               const esp_image_header_t* img_hdr,
                               esp_image_type type)
{
    esp_err_t err = ESP_OK;
    esp_chip_id_t chip_id = CONFIG_IDF_FIRMWARE_CHIP_ID;
    if (chip_id != img_hdr->chip_id) {
        BOOT_LOG_ERR("Mismatch chip ID, expected %d, found %d",
                               chip_id, img_hdr->chip_id);
        err = ESP_FAIL;
    }

#ifndef CONFIG_SOC_ESP32C3
    uint8_t revision = bootloader_common_get_chip_revision();
    if (revision < img_hdr->min_chip_rev) {
        /* To fix this error, please update mininum supported chip revision from configuration,
         * located in TARGET (e.g. ESP32) specific options under "Component config" menu */
        BOOT_LOG_ERR("Found chip rev. %d but minimum required revision %d. Can't run.",
                               revision, img_hdr->min_chip_rev);
        err = ESP_FAIL;
    } else if (revision != img_hdr->min_chip_rev) {
#ifdef BOOTLOADER_BUILD
        BOOT_LOG_INF("Chip revision: %d, min. %s chip revision: %d",
                               revision, type == ESP_IMAGE_BOOTLOADER ? "bootloader" : "app.",
                               img_hdr->min_chip_rev);
#endif
    }
#endif /* CONFIG_SOC_ESP32C3 */

    return err;
}

esp_err_t bootloader_check_bootloader_validity(void)
{
    /* read chip revision from efuse */
    uint8_t revision = bootloader_common_get_chip_revision();
    BOOT_LOG_INF("Chip revision: %d", revision);
    /* compare with the one set in bootloader image header */
    if (bootloader_common_check_chip_validity(&bootloader_image_hdr, ESP_IMAGE_BOOTLOADER) != ESP_OK) {
        return ESP_FAIL;
    }
    return ESP_OK;
}

void bootloader_print_flash_info(const esp_image_header_t *bootloader_hdr)
{
    BOOT_LOG_DBG("magic %02x", bootloader_hdr->magic);
    BOOT_LOG_DBG("segments %02x", bootloader_hdr->segment_count);
    BOOT_LOG_DBG("spi_mode %02x", bootloader_hdr->spi_mode);
    BOOT_LOG_DBG("spi_speed %02x", bootloader_hdr->spi_speed);
    BOOT_LOG_DBG("spi_size %02x", bootloader_hdr->spi_size);

    const char *speed_str;
    switch (bootloader_hdr->spi_speed) {
    case ESP_IMAGE_SPI_SPEED_40M:
        speed_str = "40MHz";
        break;
    case ESP_IMAGE_SPI_SPEED_26M:
        speed_str = "26.7MHz";
        break;
    case ESP_IMAGE_SPI_SPEED_20M:
        speed_str = "20MHz";
        break;
    case ESP_IMAGE_SPI_SPEED_80M:
        speed_str = "80MHz";
        break;
    default:
        speed_str = "20MHz";
        break;
    }

#ifdef CONFIG_SOC_ESP32
#undef SPI_MEM_CTRL_REG
#define SPI_MEM_CTRL_REG(x) SPI_CTRL_REG(x)
#undef SPI_MEM_FREAD_QIO
#define SPI_MEM_FREAD_QIO SPI_FREAD_QIO
#undef SPI_MEM_FREAD_QUAD
#define SPI_MEM_FREAD_QUAD SPI_FREAD_QUAD
#undef SPI_MEM_FREAD_DIO
#define SPI_MEM_FREAD_DIO SPI_FREAD_DIO
#undef SPI_MEM_FREAD_DUAL
#define SPI_MEM_FREAD_DUAL SPI_FREAD_DUAL
#undef SPI_MEM_FASTRD_MODE
#define SPI_MEM_FASTRD_MODE SPI_FASTRD_MODE
#endif /* CONFIG_SOC_ESP32 */

    /* SPI mode could have been set to QIO during boot already,
     * so test the SPI registers not the flash header
     */
    const char *mode_str;
    uint32_t spi_ctrl = REG_READ(SPI_MEM_CTRL_REG(0));
    if (spi_ctrl & SPI_MEM_FREAD_QIO) {
        mode_str = "QIO";
    } else if (spi_ctrl & SPI_MEM_FREAD_QUAD) {
        mode_str = "QOUT";
    } else if (spi_ctrl & SPI_MEM_FREAD_DIO) {
        mode_str = "DIO";
    } else if (spi_ctrl & SPI_MEM_FREAD_DUAL) {
        mode_str = "DOUT";
    } else if (spi_ctrl & SPI_MEM_FASTRD_MODE) {
        mode_str = "FAST READ";
    } else {
        mode_str = "SLOW READ";
    }

    const char *size_str;
    switch (bootloader_hdr->spi_size) {
    case ESP_IMAGE_FLASH_SIZE_1MB:
        size_str = "1MB";
        break;
    case ESP_IMAGE_FLASH_SIZE_2MB:
        size_str = "2MB";
        break;
    case ESP_IMAGE_FLASH_SIZE_4MB:
        size_str = "4MB";
        break;
    case ESP_IMAGE_FLASH_SIZE_8MB:
        size_str = "8MB";
        break;
    case ESP_IMAGE_FLASH_SIZE_16MB:
        size_str = "16MB";
        break;
    default:
        size_str = "2MB";
        break;
    }
    BOOT_LOG_INF("SPI Flash speed: %s, mode: %s, size: %s", speed_str, mode_str, size_str);
}

void bootloader_print_banner(void)
{
#ifdef CONFIG_MCUBOOT
    BOOT_LOG_INF("MCUboot 2nd stage bootloader");
#endif
    BOOT_LOG_INF("compiled at " __DATE__ " " __TIME__);
}
