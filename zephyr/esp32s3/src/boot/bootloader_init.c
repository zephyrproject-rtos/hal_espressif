/*
 * SPDX-FileCopyrightText: 2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>
#include <string.h>
#include "sdkconfig.h"
#include "esp_attr.h"
#include "esp_image_format.h"
#include "flash_qio_mode.h"
#include "esp_rom_efuse.h"
#include "esp_rom_gpio.h"
#include "esp_rom_sys.h"
#include "esp_rom_uart.h"
#include "esp_efuse.h"

#include "bootloader_init_common.h"
#include "bootloader_init.h"
#include "bootloader_common.h"
#include "bootloader_console.h"
#include "bootloader_mem.h"
#include "bootloader_clock.h"
#include "bootloader_flash_config.h"
#include "bootloader_flash.h"
#include "bootloader_flash_priv.h"
#include "bootloader_soc.h"
#include "bootloader_random.h"

#include "soc/cpu.h"
#include "soc/dport_reg.h"
#include "soc/efuse_reg.h"
#include "soc/rtc.h"
#include "soc/rtc_cntl_reg.h"
#include "soc/extmem_reg.h"
#include "soc/io_mux_reg.h"
#include "soc/assist_debug_reg.h"

#include "bootloader_wdt.h"
#include "hal/wdt_hal.h"

#include "esp32s3/rom/cache.h"
#include "esp32s3/rom/ets_sys.h"
#include "esp32s3/rom/spi_flash.h"
#include "esp32s3/rom/uart.h"

#include <bootutil/bootutil_log.h>

static const char *TAG = "boot.esp32s3";

extern esp_image_header_t WORD_ALIGNED_ATTR bootloader_image_hdr;

static void bootloader_reset_mmu(void)
{
    Cache_Suspend_DCache();
    Cache_Invalidate_DCache_All();
    Cache_MMU_Init();

    REG_CLR_BIT(EXTMEM_ICACHE_CTRL1_REG, EXTMEM_ICACHE_SHUT_CORE0_BUS);
    REG_CLR_BIT(EXTMEM_ICACHE_CTRL1_REG, EXTMEM_ICACHE_SHUT_CORE1_BUS);
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
    uint32_t autoload = Cache_Suspend_DCache();
    /* Set flash chip size */
    esp_rom_spiflash_config_param(g_rom_flashchip.device_id, size * 0x100000, 0x10000, 0x1000, 0x100, 0xffff);
    Cache_Resume_DCache(autoload);
}

void IRAM_ATTR bootloader_configure_spi_pins(int drv)
{
    const uint32_t spiconfig = esp_rom_efuse_get_flash_gpio_info();
    uint8_t wp_pin = esp_rom_efuse_get_flash_wp_gpio();
    uint8_t clk_gpio_num = SPI_CLK_GPIO_NUM;
    uint8_t q_gpio_num   = SPI_Q_GPIO_NUM;
    uint8_t d_gpio_num   = SPI_D_GPIO_NUM;
    uint8_t cs0_gpio_num = SPI_CS0_GPIO_NUM;
    uint8_t hd_gpio_num  = SPI_HD_GPIO_NUM;
    uint8_t wp_gpio_num  = SPI_WP_GPIO_NUM;
    if (spiconfig == 0) {

    } else {
        clk_gpio_num = spiconfig         & 0x3f;
        q_gpio_num = (spiconfig >> 6)    & 0x3f;
        d_gpio_num = (spiconfig >> 12)   & 0x3f;
        cs0_gpio_num = (spiconfig >> 18) & 0x3f;
        hd_gpio_num = (spiconfig >> 24)  & 0x3f;
        wp_gpio_num = wp_pin;
    }
    esp_rom_gpio_pad_set_drv(clk_gpio_num, drv);
    esp_rom_gpio_pad_set_drv(q_gpio_num,   drv);
    esp_rom_gpio_pad_set_drv(d_gpio_num,   drv);
    esp_rom_gpio_pad_set_drv(cs0_gpio_num, drv);
    if (hd_gpio_num <= MAX_PAD_GPIO_NUM) {
        esp_rom_gpio_pad_set_drv(hd_gpio_num, drv);
    }
    if (wp_gpio_num <= MAX_PAD_GPIO_NUM) {
        esp_rom_gpio_pad_set_drv(wp_gpio_num, drv);
    }
}

static void IRAM_ATTR bootloader_init_flash_configure(void)
{
    bootloader_flash_dummy_config(&bootloader_image_hdr);
    bootloader_flash_cs_timing_config();
}

static esp_err_t bootloader_init_spi_flash(void)
{
    bootloader_init_flash_configure();
#ifndef CONFIG_SPI_FLASH_ROM_DRIVER_PATCH
    const uint32_t spiconfig = esp_rom_efuse_get_flash_gpio_info();
    if (spiconfig != ESP_ROM_EFUSE_FLASH_DEFAULT_SPI &&
        spiconfig != ESP_ROM_EFUSE_FLASH_DEFAULT_HSPI) {
        ESP_LOGE(TAG, "SPI flash pins are overridden. Enable CONFIG_SPI_FLASH_ROM_DRIVER_PATCH in menuconfig");
        return ESP_FAIL;
    }
#endif

    bootloader_flash_unlock();
    update_flash_config(&bootloader_image_hdr);
    /* ensure the flash is write-protected */
    bootloader_enable_wp();

    bootloader_print_flash_info(&bootloader_image_hdr);

    return ESP_OK;
}

static void wdt_reset_cpu0_info_enable(void)
{
    REG_SET_BIT(SYSTEM_CPU_PERI_CLK_EN_REG, SYSTEM_CLK_EN_ASSIST_DEBUG);
    REG_CLR_BIT(SYSTEM_CPU_PERI_RST_EN_REG, SYSTEM_RST_EN_ASSIST_DEBUG);
    REG_WRITE(ASSIST_DEBUG_CORE_0_RCD_PDEBUGENABLE_REG, 1);
    REG_WRITE(ASSIST_DEBUG_CORE_0_RCD_RECORDING_REG, 1);
}

#if MCUBOOT_LOG_LEVEL >= MCUBOOT_LOG_LEVEL_DEBUG
static void wdt_reset_info_dump(int cpu)
{
    uint32_t inst = 0, pid = 0, stat = 0, data = 0, pc = 0,
             lsstat = 0, lsaddr = 0, lsdata = 0, dstat = 0;
    const char *cpu_name = cpu ? "APP" : "PRO";

    stat = 0xdeadbeef;
    pid = 0;
    if (cpu == 0) {
        inst    = REG_READ(ASSIST_DEBUG_CORE_0_RCD_PDEBUGINST_REG);
        dstat   = REG_READ(ASSIST_DEBUG_CORE_0_RCD_PDEBUGSTATUS_REG);
        data    = REG_READ(ASSIST_DEBUG_CORE_0_RCD_PDEBUGDATA_REG);
        pc      = REG_READ(ASSIST_DEBUG_CORE_0_RCD_PDEBUGPC_REG);
        lsstat  = REG_READ(ASSIST_DEBUG_CORE_0_RCD_PDEBUGLS0STAT_REG);
        lsaddr  = REG_READ(ASSIST_DEBUG_CORE_0_RCD_PDEBUGLS0ADDR_REG);
        lsdata  = REG_READ(ASSIST_DEBUG_CORE_0_RCD_PDEBUGLS0DATA_REG);
    } else {
        inst    = REG_READ(ASSIST_DEBUG_CORE_1_RCD_PDEBUGINST_REG);
        dstat   = REG_READ(ASSIST_DEBUG_CORE_1_RCD_PDEBUGSTATUS_REG);
        data    = REG_READ(ASSIST_DEBUG_CORE_1_RCD_PDEBUGDATA_REG);
        pc      = REG_READ(ASSIST_DEBUG_CORE_1_RCD_PDEBUGPC_REG);
        lsstat  = REG_READ(ASSIST_DEBUG_CORE_1_RCD_PDEBUGLS0STAT_REG);
        lsaddr  = REG_READ(ASSIST_DEBUG_CORE_1_RCD_PDEBUGLS0ADDR_REG);
        lsdata  = REG_READ(ASSIST_DEBUG_CORE_1_RCD_PDEBUGLS0DATA_REG);
    }

    ESP_LOGD(TAG, "WDT reset info: %s CPU STATUS        0x%08x", cpu_name, stat);
    ESP_LOGD(TAG, "WDT reset info: %s CPU PID           0x%08x", cpu_name, pid);
    ESP_LOGD(TAG, "WDT reset info: %s CPU PDEBUGINST    0x%08x", cpu_name, inst);
    ESP_LOGD(TAG, "WDT reset info: %s CPU PDEBUGSTATUS  0x%08x", cpu_name, dstat);
    ESP_LOGD(TAG, "WDT reset info: %s CPU PDEBUGDATA    0x%08x", cpu_name, data);
    ESP_LOGD(TAG, "WDT reset info: %s CPU PDEBUGPC      0x%08x", cpu_name, pc);
    ESP_LOGD(TAG, "WDT reset info: %s CPU PDEBUGLS0STAT 0x%08x", cpu_name, lsstat);
    ESP_LOGD(TAG, "WDT reset info: %s CPU PDEBUGLS0ADDR 0x%08x", cpu_name, lsaddr);
    ESP_LOGD(TAG, "WDT reset info: %s CPU PDEBUGLS0DATA 0x%08x", cpu_name, lsdata);
}
#endif

static void bootloader_check_wdt_reset(void)
{
    int wdt_rst = 0;
    soc_reset_reason_t rst_reas[2];

    rst_reas[0] = esp_rom_get_reset_reason(0);
    rst_reas[1] = esp_rom_get_reset_reason(1);
    if (rst_reas[0] == RESET_REASON_CORE_RTC_WDT || rst_reas[0] == RESET_REASON_CORE_MWDT0 || rst_reas[0] == RESET_REASON_CORE_MWDT1 ||
        rst_reas[0] == RESET_REASON_CPU0_MWDT0 || rst_reas[0] == RESET_REASON_CPU0_RTC_WDT) {
        ESP_LOGW(TAG, "PRO CPU has been reset by WDT.");
        wdt_rst = 1;
    }
    if (rst_reas[1] == RESET_REASON_CORE_RTC_WDT || rst_reas[1] == RESET_REASON_CORE_MWDT0 || rst_reas[1] == RESET_REASON_CORE_MWDT1 ||
        rst_reas[1] == RESET_REASON_CPU1_MWDT1 || rst_reas[1] == RESET_REASON_CPU1_RTC_WDT) {
        ESP_LOGW(TAG, "APP CPU has been reset by WDT.");
        wdt_rst = 1;
    }
    if (wdt_rst) {
#if MCUBOOT_LOG_LEVEL >= MCUBOOT_LOG_LEVEL_DEBUG
        /* if reset by WDT dump info from trace port */
        wdt_reset_info_dump(0);
        wdt_reset_info_dump(1);
#endif
    }
    wdt_reset_cpu0_info_enable();
}

static void bootloader_super_wdt_auto_feed(void)
{
    REG_WRITE(RTC_CNTL_SWD_WPROTECT_REG, RTC_CNTL_SWD_WKEY_VALUE);
    REG_SET_BIT(RTC_CNTL_SWD_CONF_REG, RTC_CNTL_SWD_AUTO_FEED_EN);
    REG_WRITE(RTC_CNTL_SWD_WPROTECT_REG, 0);}

static inline void bootloader_ana_reset_config(void)
{
    /* Enable WDT, BOR, and GLITCH reset */
    bootloader_ana_super_wdt_reset_config(true);
    bootloader_ana_bod_reset_config(true);
    bootloader_ana_clock_glitch_reset_config(true);
}

esp_err_t bootloader_init(void)
{
    esp_err_t ret = ESP_OK;
    bootloader_ana_reset_config();
    bootloader_super_wdt_auto_feed();
    /* protect memory region */
    bootloader_init_mem();
    /* check that static RAM is after the stack */
#ifndef NDEBUG
    {
        assert(&_bss_start <= &_bss_end);
        assert(&_data_start <= &_data_end);
    }
#endif
    /* bss section cleared in __esp_platform_start */
    /* reset MMU */
    bootloader_reset_mmu();
    /* config clock */
    bootloader_clock_configure();
    /* initialize uart console, from now on, we can use ets_printf */
    bootloader_console_init();
    /* print mcuboot banner */
    bootloader_print_banner();
    /* update flash ID */
    bootloader_flash_update_id();
    /* Check and run XMC startup flow */
    if ((ret = bootloader_flash_xmc_startup()) != ESP_OK) {
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
    /* check whether a WDT reset happend */
    bootloader_check_wdt_reset();
    /* config WDT */
    bootloader_config_wdt();
    /* enable RNG early entropy source */
    bootloader_random_enable();

err:
    return ret;
}
