/*
 * SPDX-FileCopyrightText: 2015-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/__assert.h>
#include "esp_attr.h"
#include "esp_private/periph_ctrl.h"
#include "esp_private/critical_section.h"
#include "soc/soc_caps.h"
#ifdef __PERIPH_CTRL_ALLOW_LEGACY_API
#include "hal/clk_gate_ll.h"
#endif
#include "esp_log.h"

#if defined(CONFIG_SOC_SERIES_ESP32)
#include <zephyr/dt-bindings/clock/esp32_clock.h>
#elif defined(CONFIG_SOC_SERIES_ESP32S2)
#include <zephyr/dt-bindings/clock/esp32s2_clock.h>
#elif defined(CONFIG_SOC_SERIES_ESP32S3)
#include <zephyr/dt-bindings/clock/esp32s3_clock.h>
#elif defined(CONFIG_SOC_SERIES_ESP32C2)
#include <zephyr/dt-bindings/clock/esp32c2_clock.h>
#elif defined(CONFIG_SOC_SERIES_ESP32C3)
#include <zephyr/dt-bindings/clock/esp32c3_clock.h>
#elif defined(CONFIG_SOC_SERIES_ESP32C5)
#include <zephyr/dt-bindings/clock/esp32c5_clock.h>
#elif defined(CONFIG_SOC_SERIES_ESP32C6)
#include <zephyr/dt-bindings/clock/esp32c6_clock.h>
#elif defined(CONFIG_SOC_SERIES_ESP32H2)
#include <zephyr/dt-bindings/clock/esp32h2_clock.h>
#endif

#include <hal/uart_ll.h>
#include <hal/i2c_ll.h>
#include <hal/spi_ll.h>
#include <hal/ledc_ll.h>
#include <hal/sha_ll.h>
#if SOC_AES_SUPPORTED
#include <hal/aes_ll.h>
#endif
#if SOC_ADC_SUPPORTED
#include <hal/adc_ll.h>
#endif
#include <hal/timg_ll.h>
#if SOC_TWAI_SUPPORTED
#include <hal/twai_ll.h>
#endif
#if SOC_GDMA_SUPPORTED
#include <hal/gdma_ll.h>
#endif
#if SOC_PCNT_SUPPORTED
#include <hal/pcnt_ll.h>
#endif
#if SOC_MCPWM_SUPPORTED
#include <hal/mcpwm_ll.h>
#endif
#if SOC_I2S_SUPPORTED
#include <hal/i2s_ll.h>
#endif
#if SOC_USB_SERIAL_JTAG_SUPPORTED
#include <hal/usb_serial_jtag_ll.h>
#endif
#if SOC_SDMMC_HOST_SUPPORTED
#include <hal/sdmmc_ll.h>
#endif
#if SOC_EMAC_SUPPORTED
#include <hal/emac_ll.h>
#endif
#if SOC_UHCI_SUPPORTED
#include <hal/uhci_ll.h>
#endif
#if SOC_SDIO_SLAVE_SUPPORTED
#include <hal/sdio_slave_ll.h>
#endif
#if SOC_ECC_SUPPORTED
#include <hal/ecc_ll.h>
#endif
#ifdef ESP32_HMAC_MODULE
#include <hal/hmac_ll.h>
#endif
#ifdef ESP32_DS_MODULE
#include <hal/ds_ll.h>
#endif
#if SOC_ETM_SUPPORTED
#include <hal/etm_ll.h>
#endif
#if SOC_TEMP_SENSOR_SUPPORTED
#include <hal/temperature_sensor_ll.h>
#endif
#if SOC_ECDSA_SUPPORTED
#include <hal/ecdsa_ll.h>
#endif
#ifdef ESP32_DEDIC_GPIO_MODULE
#include <hal/dedic_gpio_ll.h>
#endif
#if SOC_PAU_SUPPORTED
#include <hal/pau_ll.h>
#endif
#if SOC_ASSIST_DEBUG_SUPPORTED
#include <hal/assist_debug_ll.h>
#endif

#if SOC_MODEM_CLOCK_IS_INDEPENDENT && SOC_MODEM_CLOCK_SUPPORTED
#include "esp_private/esp_modem_clock.h"
#endif

/* For simplicity and backward compatible, we are using the same spin lock for both bus clock on/off and reset */
/* We may want to split them into two spin locks in the future */
static unsigned int __attribute__((unused)) periph_spinlock;
static atomic_t rcc_lock_counter;

static uint8_t ref_counts[PERIPH_MODULE_MAX] = {0};

IRAM_ATTR void periph_rcc_enter(void)
{
    if (atomic_inc(&rcc_lock_counter) == 0) {
        periph_spinlock = irq_lock();
    }
}

IRAM_ATTR void periph_rcc_exit(void)
{
    __ASSERT_NO_MSG(atomic_get(&rcc_lock_counter) > 0);
    if (atomic_dec(&rcc_lock_counter) == 1) {
        irq_unlock(periph_spinlock);
    }
}

uint8_t periph_rcc_acquire_enter(shared_periph_module_t periph)
{
    periph_rcc_enter();
    return ref_counts[periph];
}

void periph_rcc_acquire_exit(shared_periph_module_t periph, uint8_t ref_count)
{
    ref_counts[periph] = ++ref_count;
    periph_rcc_exit();
}

uint8_t periph_rcc_release_enter(shared_periph_module_t periph)
{
    periph_rcc_enter();
    if (ref_counts[periph] > 0) {
        return ref_counts[periph] - 1;
    }
    return 0;
}

void periph_rcc_release_exit(shared_periph_module_t periph, uint8_t ref_count)
{
    ref_counts[periph] = ref_count;
    periph_rcc_exit();
}

void periph_module_enable(shared_periph_module_t periph)
{
#ifdef __PERIPH_CTRL_ALLOW_LEGACY_API
    assert(periph < PERIPH_MODULE_MAX);
    periph_rcc_enter();
    if (ref_counts[periph] == 0) {
        periph_ll_enable_clk_clear_rst(periph);
    }
    ref_counts[periph]++;
    periph_rcc_exit();
#endif
}

void periph_module_disable(shared_periph_module_t periph)
{
#ifdef __PERIPH_CTRL_ALLOW_LEGACY_API
    assert(periph < PERIPH_MODULE_MAX);
    periph_rcc_enter();
    ref_counts[periph]--;
    if (ref_counts[periph] == 0) {
        periph_ll_disable_clk_set_rst(periph);
    }
    periph_rcc_exit();
#endif
}

void periph_module_reset(shared_periph_module_t periph)
{
#ifdef __PERIPH_CTRL_ALLOW_LEGACY_API
    assert(periph < PERIPH_MODULE_MAX);
    periph_rcc_enter();
    periph_ll_reset(periph);
    periph_rcc_exit();
#endif
}

void non_shared_periph_module_enable(int periph)
{
    periph_rcc_enter();
    switch (periph) {
    case ESP32_LEDC_MODULE:
        ledc_ll_enable_bus_clock(0, true);
        ledc_ll_reset_register(0);
        break;
    case ESP32_UART0_MODULE:
        uart_ll_enable_bus_clock(0, true);
        uart_ll_reset_register(0);
        break;
#if defined(ESP32_UART1_MODULE) && ESP32_UART1_MODULE >= 100
    case ESP32_UART1_MODULE:
        uart_ll_enable_bus_clock(1, true);
        uart_ll_reset_register(1);
        break;
#endif
    case ESP32_I2C0_MODULE:
        i2c_ll_enable_bus_clock(0, true);
        i2c_ll_reset_register(0);
        break;
#ifdef ESP32_I2C1_MODULE
    case ESP32_I2C1_MODULE:
        i2c_ll_enable_bus_clock(1, true);
        i2c_ll_reset_register(1);
        break;
#endif
#ifdef ESP32_SPI_MODULE
    case ESP32_SPI_MODULE:
        spi_ll_enable_bus_clock(SPI1_HOST, true);
        spi_ll_reset_register(SPI1_HOST);
        break;
#endif
#ifdef ESP32_SPI2_MODULE
    case ESP32_SPI2_MODULE:
        spi_ll_enable_bus_clock(SPI2_HOST, true);
        spi_ll_reset_register(SPI2_HOST);
        break;
#endif
#ifdef ESP32_SPI3_MODULE
    case ESP32_SPI3_MODULE:
        spi_ll_enable_bus_clock(SPI3_HOST, true);
        spi_ll_reset_register(SPI3_HOST);
        break;
#endif
#ifdef ESP32_FSPI_MODULE
    case ESP32_FSPI_MODULE:
        spi_ll_enable_bus_clock(SPI2_HOST, true);
        spi_ll_reset_register(SPI2_HOST);
        break;
#endif
#ifdef ESP32_HSPI_MODULE
#if ESP32_HSPI_MODULE >= 100
    case ESP32_HSPI_MODULE:
        spi_ll_enable_bus_clock(SPI3_HOST, true);
        spi_ll_reset_register(SPI3_HOST);
        break;
#endif
#endif
#if SOC_TWAI_SUPPORTED
#ifdef ESP32_TWAI_MODULE
    case ESP32_TWAI_MODULE:
        twai_ll_enable_bus_clock(0, true);
        twai_ll_reset_register(0);
        twai_ll_set_clock_source(0, TWAI_CLK_SRC_DEFAULT);
        twai_ll_enable_clock(0, true);
        break;
#endif
#ifdef ESP32_TWAI0_MODULE
    case ESP32_TWAI0_MODULE:
        twai_ll_enable_bus_clock(0, true);
        twai_ll_reset_register(0);
        twai_ll_set_clock_source(0, TWAI_CLK_SRC_DEFAULT);
        twai_ll_enable_clock(0, true);
        break;
#endif
#ifdef ESP32_TWAI1_MODULE
    case ESP32_TWAI1_MODULE:
        twai_ll_enable_bus_clock(1, true);
        twai_ll_reset_register(1);
        twai_ll_set_clock_source(1, TWAI_CLK_SRC_DEFAULT);
        twai_ll_enable_clock(1, true);
        break;
#endif
#endif /* SOC_TWAI_SUPPORTED */
#ifdef ESP32_SPI_DMA_MODULE
    case ESP32_SPI_DMA_MODULE:
        spi_dma_ll_enable_bus_clock(SPI1_HOST, true);
        spi_dma_ll_reset_register(SPI1_HOST);
        break;
#endif
#if SOC_GDMA_SUPPORTED
    case ESP32_GDMA_MODULE:
        gdma_ll_enable_bus_clock(0, true);
        gdma_ll_reset_register(0);
        break;
#endif
#if SOC_PCNT_SUPPORTED
    case ESP32_PCNT_MODULE:
        pcnt_ll_enable_bus_clock(0, true);
        pcnt_ll_reset_register(0);
        break;
#endif
#ifdef ESP32_MCPWM0_MODULE
    case ESP32_MCPWM0_MODULE:
        mcpwm_ll_enable_bus_clock(0, true);
        mcpwm_ll_reset_register(0);
        break;
#endif
#ifdef ESP32_PWM0_MODULE
    case ESP32_PWM0_MODULE:
        mcpwm_ll_enable_bus_clock(0, true);
        mcpwm_ll_reset_register(0);
        break;
#endif
#ifdef ESP32_PWM1_MODULE
    case ESP32_PWM1_MODULE:
        mcpwm_ll_enable_bus_clock(1, true);
        mcpwm_ll_reset_register(1);
        break;
#endif
#ifdef ESP32_PWM2_MODULE
    case ESP32_PWM2_MODULE:
        mcpwm_ll_enable_bus_clock(2, true);
        mcpwm_ll_reset_register(2);
        break;
#endif
#ifdef ESP32_PWM3_MODULE
    case ESP32_PWM3_MODULE:
        mcpwm_ll_enable_bus_clock(3, true);
        mcpwm_ll_reset_register(3);
        break;
#endif
#if SOC_I2S_SUPPORTED
#ifdef ESP32_I2S0_MODULE
    case ESP32_I2S0_MODULE:
        i2s_ll_enable_bus_clock(0, true);
        i2s_ll_reset_register(0);
        break;
#endif
#ifdef ESP32_I2S1_MODULE
    case ESP32_I2S1_MODULE:
        i2s_ll_enable_bus_clock(1, true);
        i2s_ll_reset_register(1);
        break;
#endif
#endif /* SOC_I2S_SUPPORTED */
#if SOC_AES_SUPPORTED
#ifdef ESP32_AES_MODULE
#if ESP32_AES_MODULE >= 100
    case ESP32_AES_MODULE:
        aes_ll_enable_bus_clock(true);
        aes_ll_reset_register();
        break;
#endif
#endif
#endif /* SOC_AES_SUPPORTED */
#ifdef ESP32_SHA_MODULE
#if ESP32_SHA_MODULE >= 100
    case ESP32_SHA_MODULE:
        sha_ll_enable_bus_clock(true);
        sha_ll_reset_register();
        break;
#endif
#endif
#if SOC_USB_SERIAL_JTAG_SUPPORTED
#ifdef ESP32_USB_MODULE
    case ESP32_USB_MODULE:
        usb_serial_jtag_ll_enable_bus_clock(true);
        break;
#endif
#ifdef ESP32_USB_DEVICE_MODULE
    case ESP32_USB_DEVICE_MODULE:
        usb_serial_jtag_ll_enable_bus_clock(true);
        break;
#endif
#endif
#if SOC_SDMMC_HOST_SUPPORTED
#ifdef ESP32_SDMMC_MODULE
    case ESP32_SDMMC_MODULE:
        sdmmc_ll_enable_bus_clock(0, true);
        sdmmc_ll_reset_register(0);
        break;
#endif
#endif
#if SOC_EMAC_SUPPORTED
#ifdef ESP32_EMAC_MODULE
    case ESP32_EMAC_MODULE:
        emac_ll_enable_bus_clock(0, true);
        emac_ll_reset_register(0);
        break;
#endif
#endif
#ifdef ESP32_LP_UART0_MODULE
    case ESP32_LP_UART0_MODULE:
        lp_uart_ll_enable_bus_clock(0, true);
        lp_uart_ll_reset_register(0);
        break;
#endif
#ifdef ESP32_LP_I2C0_MODULE
    case ESP32_LP_I2C0_MODULE:
        lp_i2c_ll_enable_bus_clock(0, true);
        lp_i2c_ll_reset_register(0);
        break;
#endif
#if SOC_ADC_SUPPORTED
#ifdef ESP32_SARADC_MODULE
#if ESP32_SARADC_MODULE >= 100
    case ESP32_SARADC_MODULE:
        adc_ll_enable_bus_clock(true);
        adc_ll_reset_register();
        break;
#endif
#endif
#endif
#if SOC_UHCI_SUPPORTED
#ifdef ESP32_UHCI1_MODULE
    case ESP32_UHCI1_MODULE:
        uhci_ll_enable_bus_clock(1, true);
        uhci_ll_reset_register(1);
        break;
#endif
#endif
#if SOC_SDIO_SLAVE_SUPPORTED
#ifdef ESP32_SDIO_SLAVE_MODULE
#if ESP32_SDIO_SLAVE_MODULE >= 100
    case ESP32_SDIO_SLAVE_MODULE:
        sdio_slave_ll_enable_bus_clock(true);
        sdio_slave_ll_reset_register();
        break;
#endif
#endif
#endif
#if SOC_ECC_SUPPORTED
#ifdef ESP32_ECC_MODULE
    case ESP32_ECC_MODULE:
        ecc_ll_enable_bus_clock(true);
        ecc_ll_reset_register();
        break;
#endif
#endif
#if SOC_HMAC_SUPPORTED
#ifdef ESP32_HMAC_MODULE
    case ESP32_HMAC_MODULE:
        hmac_ll_enable_bus_clock(true);
        hmac_ll_reset_register();
        break;
#endif
#endif
#if SOC_DIG_SIGN_SUPPORTED
#ifdef ESP32_DS_MODULE
    case ESP32_DS_MODULE:
        ds_ll_enable_bus_clock(true);
        ds_ll_reset_register();
        break;
#endif
#endif
#if SOC_ETM_SUPPORTED
#ifdef ESP32_ETM_MODULE
    case ESP32_ETM_MODULE:
        etm_ll_enable_bus_clock(0, true);
        etm_ll_reset_register(0);
        break;
#endif
#endif
#if SOC_TEMP_SENSOR_SUPPORTED
#ifdef ESP32_TEMPSENSOR_MODULE
    case ESP32_TEMPSENSOR_MODULE:
        temperature_sensor_ll_bus_clk_enable(true);
        temperature_sensor_ll_reset_module();
        break;
#endif
#endif
#if SOC_ECDSA_SUPPORTED
#ifdef ESP32_ECDSA_MODULE
    case ESP32_ECDSA_MODULE:
        ecdsa_ll_enable_bus_clock(true);
        ecdsa_ll_reset_register();
        break;
#endif
#endif
#ifdef ESP32_DEDIC_GPIO_MODULE
    case ESP32_DEDIC_GPIO_MODULE:
        dedic_gpio_ll_enable_bus_clock(true);
        dedic_gpio_ll_reset_register();
        break;
#endif
#if SOC_PAU_SUPPORTED
#ifdef ESP32_REGDMA_MODULE
    case ESP32_REGDMA_MODULE:
        pau_ll_enable_bus_clock(true);
        break;
#endif
#endif
#if SOC_ASSIST_DEBUG_SUPPORTED
#ifdef ESP32_ASSIST_DEBUG_MODULE
    case ESP32_ASSIST_DEBUG_MODULE:
        assist_debug_ll_enable_bus_clock(0, true);
        assist_debug_ll_reset_register(0);
        break;
#endif
#endif
    default:
        break;
    }
    periph_rcc_exit();
}

void non_shared_periph_module_disable(int periph)
{
    periph_rcc_enter();
    switch (periph) {
    case ESP32_LEDC_MODULE:
        ledc_ll_enable_bus_clock(0, false);
        break;
    case ESP32_UART0_MODULE:
        uart_ll_enable_bus_clock(0, false);
        break;
#if defined(ESP32_UART1_MODULE) && ESP32_UART1_MODULE >= 100
    case ESP32_UART1_MODULE:
        uart_ll_enable_bus_clock(1, false);
        break;
#endif
    case ESP32_I2C0_MODULE:
        i2c_ll_enable_bus_clock(0, false);
        break;
#ifdef ESP32_I2C1_MODULE
    case ESP32_I2C1_MODULE:
        i2c_ll_enable_bus_clock(1, false);
        break;
#endif
#ifdef ESP32_SPI_MODULE
    case ESP32_SPI_MODULE:
        spi_ll_enable_bus_clock(SPI1_HOST, false);
        break;
#endif
#ifdef ESP32_SPI2_MODULE
    case ESP32_SPI2_MODULE:
        spi_ll_enable_bus_clock(SPI2_HOST, false);
        break;
#endif
#ifdef ESP32_SPI3_MODULE
    case ESP32_SPI3_MODULE:
        spi_ll_enable_bus_clock(SPI3_HOST, false);
        break;
#endif
#ifdef ESP32_FSPI_MODULE
    case ESP32_FSPI_MODULE:
        spi_ll_enable_bus_clock(SPI2_HOST, false);
        break;
#endif
#ifdef ESP32_HSPI_MODULE
#if ESP32_HSPI_MODULE >= 100
    case ESP32_HSPI_MODULE:
        spi_ll_enable_bus_clock(SPI3_HOST, false);
        break;
#endif
#endif
#if SOC_TWAI_SUPPORTED
#ifdef ESP32_TWAI_MODULE
    case ESP32_TWAI_MODULE:
        twai_ll_enable_bus_clock(0, false);
        break;
#endif
#ifdef ESP32_TWAI0_MODULE
    case ESP32_TWAI0_MODULE:
        twai_ll_enable_bus_clock(0, false);
        break;
#endif
#ifdef ESP32_TWAI1_MODULE
    case ESP32_TWAI1_MODULE:
        twai_ll_enable_bus_clock(1, false);
        break;
#endif
#endif /* SOC_TWAI_SUPPORTED */
#ifdef ESP32_SPI_DMA_MODULE
    case ESP32_SPI_DMA_MODULE:
        spi_dma_ll_enable_bus_clock(SPI1_HOST, false);
        break;
#endif
#if SOC_GDMA_SUPPORTED
    case ESP32_GDMA_MODULE:
        gdma_ll_enable_bus_clock(0, false);
        break;
#endif
#if SOC_PCNT_SUPPORTED
    case ESP32_PCNT_MODULE:
        pcnt_ll_enable_bus_clock(0, false);
        break;
#endif
#ifdef ESP32_MCPWM0_MODULE
    case ESP32_MCPWM0_MODULE:
        mcpwm_ll_enable_bus_clock(0, false);
        break;
#endif
#ifdef ESP32_PWM0_MODULE
    case ESP32_PWM0_MODULE:
        mcpwm_ll_enable_bus_clock(0, false);
        break;
#endif
#ifdef ESP32_PWM1_MODULE
    case ESP32_PWM1_MODULE:
        mcpwm_ll_enable_bus_clock(1, false);
        break;
#endif
#ifdef ESP32_PWM2_MODULE
    case ESP32_PWM2_MODULE:
        mcpwm_ll_enable_bus_clock(2, false);
        break;
#endif
#ifdef ESP32_PWM3_MODULE
    case ESP32_PWM3_MODULE:
        mcpwm_ll_enable_bus_clock(3, false);
        break;
#endif
#if SOC_I2S_SUPPORTED
#ifdef ESP32_I2S0_MODULE
    case ESP32_I2S0_MODULE:
        i2s_ll_enable_bus_clock(0, false);
        break;
#endif
#ifdef ESP32_I2S1_MODULE
    case ESP32_I2S1_MODULE:
        i2s_ll_enable_bus_clock(1, false);
        break;
#endif
#endif /* SOC_I2S_SUPPORTED */
#if SOC_AES_SUPPORTED
#ifdef ESP32_AES_MODULE
#if ESP32_AES_MODULE >= 100
    case ESP32_AES_MODULE:
        aes_ll_enable_bus_clock(false);
        break;
#endif
#endif
#endif /* SOC_AES_SUPPORTED */
#ifdef ESP32_SHA_MODULE
#if ESP32_SHA_MODULE >= 100
    case ESP32_SHA_MODULE:
        sha_ll_enable_bus_clock(false);
        break;
#endif
#endif
#if SOC_USB_SERIAL_JTAG_SUPPORTED
#ifdef ESP32_USB_MODULE
    case ESP32_USB_MODULE:
        usb_serial_jtag_ll_enable_bus_clock(false);
        break;
#endif
#ifdef ESP32_USB_DEVICE_MODULE
    case ESP32_USB_DEVICE_MODULE:
        usb_serial_jtag_ll_enable_bus_clock(false);
        break;
#endif
#endif
#if SOC_SDMMC_HOST_SUPPORTED
#ifdef ESP32_SDMMC_MODULE
    case ESP32_SDMMC_MODULE:
        sdmmc_ll_enable_bus_clock(0, false);
        break;
#endif
#endif
#if SOC_EMAC_SUPPORTED
#ifdef ESP32_EMAC_MODULE
    case ESP32_EMAC_MODULE:
        emac_ll_enable_bus_clock(0, false);
        break;
#endif
#endif
#ifdef ESP32_LP_UART0_MODULE
    case ESP32_LP_UART0_MODULE:
        lp_uart_ll_enable_bus_clock(0, false);
        break;
#endif
#ifdef ESP32_LP_I2C0_MODULE
    case ESP32_LP_I2C0_MODULE:
        lp_i2c_ll_enable_bus_clock(0, false);
        break;
#endif
#if SOC_ADC_SUPPORTED
#ifdef ESP32_SARADC_MODULE
#if ESP32_SARADC_MODULE >= 100
    case ESP32_SARADC_MODULE:
        adc_ll_enable_bus_clock(false);
        break;
#endif
#endif
#endif
#if SOC_UHCI_SUPPORTED
#ifdef ESP32_UHCI1_MODULE
    case ESP32_UHCI1_MODULE:
        uhci_ll_enable_bus_clock(1, false);
        break;
#endif
#endif
#if SOC_SDIO_SLAVE_SUPPORTED
#ifdef ESP32_SDIO_SLAVE_MODULE
#if ESP32_SDIO_SLAVE_MODULE >= 100
    case ESP32_SDIO_SLAVE_MODULE:
        sdio_slave_ll_enable_bus_clock(false);
        break;
#endif
#endif
#endif
#if SOC_ECC_SUPPORTED
#ifdef ESP32_ECC_MODULE
    case ESP32_ECC_MODULE:
        ecc_ll_enable_bus_clock(false);
        break;
#endif
#endif
#if SOC_HMAC_SUPPORTED
#ifdef ESP32_HMAC_MODULE
    case ESP32_HMAC_MODULE:
        hmac_ll_enable_bus_clock(false);
        break;
#endif
#endif
#if SOC_DIG_SIGN_SUPPORTED
#ifdef ESP32_DS_MODULE
    case ESP32_DS_MODULE:
        ds_ll_enable_bus_clock(false);
        break;
#endif
#endif
#if SOC_ETM_SUPPORTED
#ifdef ESP32_ETM_MODULE
    case ESP32_ETM_MODULE:
        etm_ll_enable_bus_clock(0, false);
        break;
#endif
#endif
#if SOC_TEMP_SENSOR_SUPPORTED
#ifdef ESP32_TEMPSENSOR_MODULE
    case ESP32_TEMPSENSOR_MODULE:
        temperature_sensor_ll_bus_clk_enable(false);
        break;
#endif
#endif
#if SOC_ECDSA_SUPPORTED
#ifdef ESP32_ECDSA_MODULE
    case ESP32_ECDSA_MODULE:
        ecdsa_ll_enable_bus_clock(false);
        break;
#endif
#endif
#ifdef ESP32_DEDIC_GPIO_MODULE
    case ESP32_DEDIC_GPIO_MODULE:
        dedic_gpio_ll_enable_bus_clock(false);
        break;
#endif
#if SOC_PAU_SUPPORTED
#ifdef ESP32_REGDMA_MODULE
    case ESP32_REGDMA_MODULE:
        pau_ll_enable_bus_clock(false);
        break;
#endif
#endif
#if SOC_ASSIST_DEBUG_SUPPORTED
#ifdef ESP32_ASSIST_DEBUG_MODULE
    case ESP32_ASSIST_DEBUG_MODULE:
        assist_debug_ll_enable_bus_clock(0, false);
        break;
#endif
#endif
    default:
        break;
    }
    periph_rcc_exit();
}

#if !SOC_IEEE802154_BLE_ONLY
#if SOC_BT_SUPPORTED || SOC_WIFI_SUPPORTED || SOC_IEEE802154_SUPPORTED
IRAM_ATTR void wifi_bt_common_module_enable(void)
{
#if SOC_MODEM_CLOCK_IS_INDEPENDENT
    modem_clock_module_enable(PERIPH_PHY_MODULE);
#else
    periph_rcc_enter();
    if (ref_counts[PERIPH_WIFI_BT_COMMON_MODULE] == 0) {
        periph_ll_wifi_bt_module_enable_clk();
    }
    ref_counts[PERIPH_WIFI_BT_COMMON_MODULE]++;
#if CONFIG_IDF_TARGET_ESP32C2
    if (ref_counts[PERIPH_COEX_MODULE] == 0) {
        periph_ll_coex_module_enable_clk_clear_rst();
    }
    ref_counts[PERIPH_COEX_MODULE]++;
#endif
    periph_rcc_exit();
#endif
}

IRAM_ATTR void wifi_bt_common_module_disable(void)
{
#if SOC_MODEM_CLOCK_IS_INDEPENDENT
    modem_clock_module_disable(PERIPH_PHY_MODULE);
#else
    periph_rcc_enter();
    ref_counts[PERIPH_WIFI_BT_COMMON_MODULE]--;
    if (ref_counts[PERIPH_WIFI_BT_COMMON_MODULE] == 0) {
        periph_ll_wifi_bt_module_disable_clk();
    }
#if CONFIG_IDF_TARGET_ESP32C2
    ref_counts[PERIPH_COEX_MODULE]--;
    if (ref_counts[PERIPH_COEX_MODULE] == 0) {
        periph_ll_coex_module_disable_clk_set_rst();
    }
#endif
    periph_rcc_exit();
#endif
}
#endif  //#if SOC_BT_SUPPORTED || SOC_WIFI_SUPPORTED
#endif  //#if !SOC_IEEE802154_BLE_ONLY

#if CONFIG_ESP_WIFI_ENABLED
void wifi_module_enable(void)
{
#if SOC_MODEM_CLOCK_IS_INDEPENDENT
    modem_clock_module_enable(PERIPH_WIFI_MODULE);
#else
    periph_rcc_enter();
    if (ref_counts[PERIPH_WIFI_MODULE] == 0) {
        periph_ll_wifi_module_enable_clk_clear_rst();
    }
    ref_counts[PERIPH_WIFI_MODULE]++;
    periph_rcc_exit();
#endif
}

void wifi_module_disable(void)
{
#if SOC_MODEM_CLOCK_IS_INDEPENDENT
    modem_clock_module_disable(PERIPH_WIFI_MODULE);
#else
    periph_rcc_enter();
    ref_counts[PERIPH_WIFI_MODULE]--;
    if (ref_counts[PERIPH_WIFI_MODULE] == 0) {
        periph_ll_wifi_module_disable_clk_set_rst();
    }
    periph_rcc_exit();
#endif
}
#endif /* CONFIG_ESP_WIFI_ENABLED */

#if SOC_BT_SUPPORTED || SOC_WIFI_SUPPORTED || SOC_IEEE802154_SUPPORTED
/* PERIPH_WIFI_BT_COMMON_MODULE is enabled outside */
IRAM_ATTR void phy_module_enable(void)
{
#if SOC_MODEM_CLOCK_IS_INDEPENDENT
    modem_clock_module_enable(PERIPH_PHY_CALIBRATION_MODULE);
#else
    periph_rcc_enter();
#if SOC_WIFI_SUPPORTED || SOC_BT_SUPPORTED
    periph_ll_phy_calibration_module_enable_clk_clear_rst();
    if (ref_counts[PERIPH_RNG_MODULE] == 0) {
        periph_ll_enable_clk_clear_rst(PERIPH_RNG_MODULE);
    }
    ref_counts[PERIPH_RNG_MODULE]++;
#endif
#if SOC_WIFI_SUPPORTED
    if (ref_counts[PERIPH_WIFI_MODULE] == 0) {
        periph_ll_wifi_module_enable_clk_clear_rst();
    }
    ref_counts[PERIPH_WIFI_MODULE]++;
#endif
#if SOC_BT_SUPPORTED
    if (ref_counts[PERIPH_BT_MODULE] == 0) {
        periph_ll_enable_clk_clear_rst(PERIPH_BT_MODULE);
    }
    ref_counts[PERIPH_BT_MODULE]++;
#endif
    periph_rcc_exit();
#endif
}

/* PERIPH_WIFI_BT_COMMON_MODULE is disabled outside */
IRAM_ATTR void phy_module_disable(void)
{
#if SOC_MODEM_CLOCK_IS_INDEPENDENT
    modem_clock_module_disable(PERIPH_PHY_CALIBRATION_MODULE);
#else
    periph_rcc_enter();
#if SOC_BT_SUPPORTED
    ref_counts[PERIPH_BT_MODULE]--;
    if (ref_counts[PERIPH_BT_MODULE] == 0) {
        periph_ll_disable_clk_set_rst(PERIPH_BT_MODULE);
    }
#endif
#if SOC_WIFI_SUPPORTED
    ref_counts[PERIPH_WIFI_MODULE]--;
    if (ref_counts[PERIPH_WIFI_MODULE] == 0) {
        periph_ll_wifi_module_disable_clk_set_rst();
    }
#endif
#if SOC_WIFI_SUPPORTED || SOC_BT_SUPPORTED
    /* Do not disable PHY clock and RNG clock */
    ref_counts[PERIPH_RNG_MODULE]--;
#endif
    periph_rcc_exit();
#endif
}

IRAM_ATTR bool phy_module_has_clock_bits(uint32_t mask)
{
    uint32_t val = 0;
#if SOC_MODEM_CLOCK_IS_INDEPENDENT
    val = modem_clock_module_bits_get(PERIPH_PHY_MODULE);
#else
#if SOC_WIFI_SUPPORTED || SOC_BT_SUPPORTED
    val = DPORT_REG_READ(periph_ll_get_clk_en_reg(PERIPH_WIFI_BT_COMMON_MODULE));
#else
    return true;
#endif
#endif
    if ((val & mask) != mask) {
        ESP_LOGW("periph_ctrl", "phy module clock bits 0x%x, required 0x%x", val, mask);
        return false;
    }
    return true;
}

IRAM_ATTR void coex_module_enable(void)
{
#if SOC_MODEM_CLOCK_IS_INDEPENDENT
    modem_clock_module_enable(PERIPH_COEX_MODULE);
#else
    periph_rcc_enter();
#if CONFIG_IDF_TARGET_ESP32C2
    if (ref_counts[PERIPH_COEX_MODULE] == 0) {
        periph_ll_coex_module_enable_clk_clear_rst();
    }
    ref_counts[PERIPH_COEX_MODULE]++;
#endif
    periph_rcc_exit();
#endif
}

IRAM_ATTR void coex_module_disable(void)
{
#if SOC_MODEM_CLOCK_IS_INDEPENDENT
    modem_clock_module_disable(PERIPH_COEX_MODULE);
#else
    periph_rcc_enter();
#if CONFIG_IDF_TARGET_ESP32C2
    ref_counts[PERIPH_COEX_MODULE]--;
    if (ref_counts[PERIPH_COEX_MODULE] == 0) {
        periph_ll_coex_module_disable_clk_set_rst();
    }
#endif
    periph_rcc_exit();
#endif
}
#endif  //#if SOC_BT_SUPPORTED || SOC_WIFI_SUPPORTED || SOC_IEEE802154_SUPPORTED
