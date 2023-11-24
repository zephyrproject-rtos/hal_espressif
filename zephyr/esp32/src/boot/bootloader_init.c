/*
 * SPDX-FileCopyrightText: 2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>
#include <string.h>

#include <bootutil/bootutil_log.h>

#include "sdkconfig.h"
#include "esp_attr.h"
#include "esp_image_format.h"

#include "bootloader_init_common.h"
#include "bootloader_init.h"
#include "bootloader_mem.h"
#include "bootloader_console.h"
#include "bootloader_clock.h"
#include "bootloader_flash_config.h"
#include "bootloader_flash.h"
#include "bootloader_flash_priv.h"

#include "soc/dport_reg.h"
#include "soc/efuse_reg.h"
#include "soc/rtc.h"

#include "bootloader_wdt.h"
#include "hal/wdt_hal.h"

#include "esp32/rom/cache.h"
#include "esp32/rom/spi_flash.h"
#include "esp32/rom/uart.h"

#include <esp_rom_uart.h>
#include <esp_rom_gpio.h>
#include <esp_rom_sys.h>
#include <soc/uart_periph.h>
#include <soc/gpio_struct.h>
#include <hal/gpio_types.h>
#include <hal/gpio_ll.h>
#include <hal/uart_ll.h>

extern esp_image_header_t WORD_ALIGNED_ATTR bootloader_image_hdr;

#if CONFIG_ESP_CONSOLE_UART_CUSTOM
static uart_dev_t *alt_console_uart_dev =
               (CONFIG_ESP_CONSOLE_UART_NUM == 0) ? &UART0 :
               (CONFIG_ESP_CONSOLE_UART_NUM == 1) ? &UART1 : &UART2;
#endif /* CONFIG_ESP_CONSOLE_UART_CUSTOM */

static void bootloader_common_vddsdio_configure(void)
{
    rtc_vddsdio_config_t cfg = rtc_vddsdio_get_config();
    if (cfg.enable == 1 && cfg.tieh == RTC_VDDSDIO_TIEH_1_8V) {
        /* VDDSDIO regulator is enabled @ 1.8V */
        cfg.drefh = 3;
        cfg.drefm = 3;
        cfg.drefl = 3;
        cfg.force = 1;
        rtc_vddsdio_set_config(cfg);
        /* wait for regulator to become stable */
        esp_rom_delay_us(10);
    }
}

static void bootloader_reset_mmu(void)
{
    /* completely reset MMU in case serial bootloader was running */
    Cache_Read_Disable(0);
#if !CONFIG_FREERTOS_UNICORE
    Cache_Read_Disable(1);
#endif
    Cache_Flush(0);
#if !CONFIG_FREERTOS_UNICORE
    Cache_Flush(1);
#endif
    mmu_init(0);
#if !CONFIG_FREERTOS_UNICORE
    /* The lines which manipulate DPORT_APP_CACHE_MMU_IA_CLR bit are
     * necessary to work around a hardware bug.
     */
    DPORT_REG_SET_BIT(DPORT_APP_CACHE_CTRL1_REG, DPORT_APP_CACHE_MMU_IA_CLR);
    mmu_init(1);
    DPORT_REG_CLR_BIT(DPORT_APP_CACHE_CTRL1_REG, DPORT_APP_CACHE_MMU_IA_CLR);
#endif

    /* normal ROM boot exits with DROM0 cache unmasked,
     * but serial bootloader exits with it masked.
     */
    DPORT_REG_CLR_BIT(DPORT_PRO_CACHE_CTRL1_REG, DPORT_PRO_CACHE_MASK_DROM0);
#if !CONFIG_FREERTOS_UNICORE
    DPORT_REG_CLR_BIT(DPORT_APP_CACHE_CTRL1_REG, DPORT_APP_CACHE_MASK_DROM0);
#endif
}

static esp_err_t bootloader_check_rated_cpu_clock(void)
{
    int rated_freq = bootloader_clock_get_rated_freq_mhz();
    if (rated_freq < 80) {
        BOOT_LOG_ERR("Chip CPU frequency misconfigured (%dMHz)", rated_freq);
        return ESP_FAIL;
    }
    return ESP_OK;
}

static void update_flash_config(const esp_image_header_t *bootloader_hdr)
{
    uint32_t size;
    switch (bootloader_hdr->spi_size) {
    case ESP_IMAGE_FLASH_SIZE_1MB:
        size = 1;
        break;
    case ESP_IMAGE_FLASH_SIZE_2MB:
        size = 2;
        break;
    case ESP_IMAGE_FLASH_SIZE_4MB:
        size = 4;
        break;
    case ESP_IMAGE_FLASH_SIZE_8MB:
        size = 8;
        break;
    case ESP_IMAGE_FLASH_SIZE_16MB:
        size = 16;
        break;
    default:
        size = 2;
    }
    Cache_Read_Disable(0);
    /* Set flash chip size */
    esp_rom_spiflash_config_param(g_rom_flashchip.device_id,
        size * 0x100000, 0x10000, 0x1000, 0x100, 0xffff);

    Cache_Flush(0);
    Cache_Read_Enable(0);
}

static void IRAM_ATTR bootloader_init_flash_configure(void)
{
    bootloader_flash_clock_config(&bootloader_image_hdr);
    bootloader_flash_gpio_config(&bootloader_image_hdr);
    bootloader_flash_dummy_config(&bootloader_image_hdr);
    bootloader_flash_cs_timing_config();
}

static esp_err_t bootloader_init_spi_flash(void)
{
    bootloader_init_flash_configure();
    esp_rom_spiflash_unlock();
    update_flash_config(&bootloader_image_hdr);
    bootloader_print_flash_info(&bootloader_image_hdr);

    return ESP_OK;
}

#if CONFIG_ESP_CONSOLE_UART_CUSTOM
void IRAM_ATTR esp_rom_uart_putc(char c)
{
    while (uart_ll_get_txfifo_len(alt_console_uart_dev) == 0);
    uart_ll_write_txfifo(alt_console_uart_dev, (const uint8_t *) &c, 1);
}
#endif /* CONFIG_ESP_CONSOLE_UART_CUSTOM */

esp_err_t bootloader_init(void)
{
    esp_err_t ret = ESP_OK;

    bootloader_init_mem();

    /* check that static RAM is after the stack */
#ifdef NDEBUG
    {
        assert(&_bss_start <= &_bss_end);
        assert(&_data_start <= &_data_end);
        assert(sp < &_bss_start);
        assert(sp < &_data_start);
    }
#endif
    /* bss section cleared in __esp_platform_start
     */
    /* boosts up vddsdio */
    bootloader_common_vddsdio_configure();
    /* reset MMU */
    bootloader_reset_mmu();
    /* check rated CPU clock */
    if ((ret = bootloader_check_rated_cpu_clock()) != ESP_OK) {
        goto err;
    }
    /* config clock */
    bootloader_clock_configure();
    /* initialize uart console, from now on, we can use ets_printf */
    bootloader_console_init();
    /* print mcuboot banner */
    bootloader_print_banner();
    /* read flash ID */
    bootloader_flash_update_id();
    /* check for XMC flash */
    if ((ret = bootloader_flash_xmc_startup()) != ESP_OK) {
        BOOT_LOG_ERR("Failed when running XMC startup flow, reboot!");
        goto err;
    }
    /* read bootloader header */
    if ((ret = bootloader_read_bootloader_header()) != ESP_OK) {
        goto err;
    }
    /* read chip revision and check if it's compatible to bootloader */
    if ((ret = bootloader_check_bootloader_validity()) != ESP_OK) {
        goto err;
    }
    /* initialize spi flash */
    if ((ret = bootloader_init_spi_flash()) != ESP_OK) {
        goto err;
    }
    /* config WDT */
    bootloader_config_wdt();

err:
    esp_rom_uart_tx_wait_idle(0);
    return ret;
}
