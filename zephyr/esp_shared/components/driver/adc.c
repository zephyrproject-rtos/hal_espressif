/*
 * SPDX-FileCopyrightText: 2016-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <esp_types.h>
#include "esp_err.h"
#include <hal/adc_hal.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/ring_buffer.h>

/* FIXME: */
typedef uint32_t TickType_t;

#include "driver/periph_ctrl.h"
#include "esp_heap_caps.h"
#include "hal/gpio_hal.h"
#include "driver/adc.h"
#include "hal/adc_hal.h"
#include "hal/adc_hal_digi.h"

//For calibration
#if CONFIG_IDF_TARGET_ESP32S2
#include "esp_efuse_rtc_table.h"
#elif SOC_ADC_CALIBRATION_V1_SUPPORTED
#include "esp_efuse_rtc_calib.h"
#endif

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(adc_hal, CONFIG_ADC_LOG_LEVEL);

struct adc_context {
    unsigned int spinlock;
};

#define ADC_GET_IO_NUM(periph, channel) (adc_channel_io_map[periph][channel])

#define ADC_ENTER_CRITICAL() do { adc_context.spinlock = irq_lock(); } while(0)
#define ADC_EXIT_CRITICAL() do { irq_unlock(adc_context.spinlock); } while(0)

extern struct adc_context adc_context;
/* this mutex lock is to protect the SARADC1 module. */
K_MUTEX_DEFINE(adc1_lock);
/* this mutex lock is to protect the SARADC2 module. */
K_MUTEX_DEFINE(adc2_lock);
/* this spin lock is to protect the shared registers used by ADC1 / ADC2 single read mode. */
K_MUTEX_DEFINE(reg_lock);

#define SAR_ADC1_LOCK_ACQUIRE() do { k_mutex_lock(&adc1_lock, K_FOREVER); } while(0)
#define SAR_ADC1_LOCK_RELEASE() do { k_mutex_unlock(&adc1_lock); } while(0)
#define SAR_ADC2_LOCK_ACQUIRE() do { k_mutex_lock(&adc2_lock, K_FOREVER); } while(0)
#define SAR_ADC2_LOCK_RELEASE() do { k_mutex_unlock(&adc2_lock); } while (0)
#define ADC_REG_LOCK_ENTER() do { k_mutex_lock(&reg_lock, K_FOREVER); } while(0)
#define ADC_REG_LOCK_EXIT() do { k_mutex_unlock(&reg_lock); } while(0)

#if SOC_ADC_CALIBRATION_V1_SUPPORTED
static uint32_t adc_get_calibration_offset(adc_ll_num_t adc_n, adc_channel_t chan, adc_atten_t atten);
#endif

static int8_t adc_digi_get_io_num(uint8_t adc_unit, uint8_t adc_channel)
{
    return adc_channel_io_map[adc_unit][adc_channel];
}

static esp_err_t adc_digi_gpio_init(adc_unit_t adc_unit, uint16_t channel_mask)
{
    esp_err_t ret = ESP_OK;
    uint64_t gpio_mask = 0;
    uint32_t n = 0;
    int8_t io = 0;
    struct gpio_dt_spec gpio;

    gpio.port = DEVICE_DT_GET(DT_NODELABEL(gpio0));
    gpio.dt_flags = 0;
    if (!device_is_ready(gpio.port)) {
        LOG_ERR("gpio0 port not ready");
        return ESP_FAIL;
    }

    while (channel_mask) {
        if (channel_mask & 0x1) {
            io = adc_digi_get_io_num(adc_unit, n);
            if (io < 0) {
                return ESP_ERR_INVALID_ARG;
            }
            gpio_mask |= BIT64(io);
            gpio.pin = io;
            gpio_pin_configure_dt(&gpio, GPIO_DISCONNECTED);
        }
        channel_mask = channel_mask >> 1;
        n++;
    }

    return ret;
}

esp_err_t adc_digi_initialize(const adc_digi_init_config_t *init_config)
{
    esp_err_t ret = ESP_OK;

	/* TODO: ready for continuous mode operation */
    return ret;
}

#if CONFIG_IDF_TARGET_ESP32C3
/*---------------------------------------------------------------
                    ADC Single Read Mode
---------------------------------------------------------------*/
static adc_atten_t s_atten1_single[ADC1_CHANNEL_MAX];
static adc_atten_t s_atten2_single[ADC2_CHANNEL_MAX];

esp_err_t adc_vref_to_gpio(adc_unit_t adc_unit, gpio_num_t gpio)
{
    esp_err_t ret;
    uint32_t channel = ADC2_CHANNEL_MAX;
    if (adc_unit == ADC_UNIT_2) {
        for (int i = 0; i < ADC2_CHANNEL_MAX; i++) {
            if (gpio == ADC_GET_IO_NUM(ADC_NUM_2, i)) {
                channel = i;
                break;
            }
        }
        if (channel == ADC2_CHANNEL_MAX) {
            return ESP_ERR_INVALID_ARG;
        }
    }

    adc_power_acquire();
    if (adc_unit & ADC_UNIT_1) {
        ADC_ENTER_CRITICAL();
        adc_hal_vref_output(ADC_NUM_1, channel, true);
        ADC_EXIT_CRITICAL();
    } else if (adc_unit & ADC_UNIT_2) {
        ADC_ENTER_CRITICAL();
        adc_hal_vref_output(ADC_NUM_2, channel, true);
        ADC_EXIT_CRITICAL();
    }

    ret = adc_digi_gpio_init(ADC_NUM_2, BIT(channel));

    return ret;
}

esp_err_t adc1_config_width(adc_bits_width_t width_bit)
{
    //On ESP32C3, the data width is always 12-bits.
    if (width_bit != ADC_WIDTH_BIT_12) {
        return ESP_ERR_INVALID_ARG;
    }

    return ESP_OK;
}

esp_err_t adc1_config_channel_atten(adc1_channel_t channel, adc_atten_t atten)
{
    if (channel >= SOC_ADC_CHANNEL_NUM(ADC_NUM_1)) {
        return ESP_ERR_INVALID_ARG;
	}
    if (atten >= ADC_ATTEN_MAX) {
        return ESP_ERR_INVALID_ARG;
	}

    esp_err_t ret = ESP_OK;
    s_atten1_single[channel] = atten;
    ret = adc_digi_gpio_init(ADC_NUM_1, BIT(channel));

    adc_hal_calibration_init(ADC_NUM_1);

    return ret;
}

int adc1_get_raw(adc1_channel_t channel)
{
    int raw_out = 0;

    periph_module_enable(PERIPH_SARADC_MODULE);
    adc_power_acquire();

    SAR_ADC1_LOCK_ACQUIRE();

    adc_atten_t atten = s_atten1_single[channel];
    uint32_t cal_val = adc_get_calibration_offset(ADC_NUM_1, channel, atten);
    adc_hal_set_calibration_param(ADC_NUM_1, cal_val);

    ADC_REG_LOCK_ENTER();
    adc_hal_set_atten(ADC_NUM_1, channel, atten);
    adc_hal_convert(ADC_NUM_1, channel, &raw_out);
    ADC_REG_LOCK_EXIT();

    SAR_ADC1_LOCK_RELEASE();

    adc_power_release();
    periph_module_disable(PERIPH_SARADC_MODULE);

    return raw_out;
}

esp_err_t adc2_config_channel_atten(adc2_channel_t channel, adc_atten_t atten)
{
    if (channel >= SOC_ADC_CHANNEL_NUM(ADC_NUM_2)) {
        return ESP_ERR_INVALID_ARG;
	}
    if (atten >= ADC_ATTEN_MAX) {
        return ESP_ERR_INVALID_ARG;
	}

    esp_err_t ret = ESP_OK;
    s_atten2_single[channel] = atten;
    ret = adc_digi_gpio_init(ADC_NUM_2, BIT(channel));

    adc_hal_calibration_init(ADC_NUM_2);

    return ret;
}

esp_err_t adc2_get_raw(adc2_channel_t channel, adc_bits_width_t width_bit, int *raw_out)
{
    //On ESP32C3, the data width is always 12-bits.
    if (width_bit != ADC_WIDTH_BIT_12) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = ESP_OK;

    periph_module_enable(PERIPH_SARADC_MODULE);
    adc_power_acquire();

    SAR_ADC2_LOCK_ACQUIRE();

    adc_arbiter_t config = ADC_ARBITER_CONFIG_DEFAULT();
    adc_hal_arbiter_config(&config);

    adc_atten_t atten = s_atten2_single[channel];
    uint32_t cal_val = adc_get_calibration_offset(ADC_NUM_2, channel, atten);
    adc_hal_set_calibration_param(ADC_NUM_2, cal_val);

    ADC_REG_LOCK_ENTER();
    adc_hal_set_atten(ADC_NUM_2, channel, atten);
    ret = adc_hal_convert(ADC_NUM_2, channel, raw_out);
    ADC_REG_LOCK_EXIT();

    SAR_ADC2_LOCK_RELEASE();

    adc_power_release();
    periph_module_disable(PERIPH_SARADC_MODULE);

    return ret;
}

#endif  //#if CONFIG_IDF_TARGET_ESP32C3


#if SOC_ADC_CALIBRATION_V1_SUPPORTED
/*---------------------------------------------------------------
                    Hardware Calibration Setting
---------------------------------------------------------------*/
#if CONFIG_IDF_TARGET_ESP32S2
#define esp_efuse_rtc_calib_get_ver()                               esp_efuse_rtc_table_read_calib_version()

static inline uint32_t esp_efuse_rtc_calib_get_init_code(int version, uint32_t adc_unit, int atten)
{
    int tag = esp_efuse_rtc_table_get_tag(version, adc_unit + 1, atten, RTCCALIB_V2_PARAM_VINIT);
    return esp_efuse_rtc_table_get_parsed_efuse_value(tag, false);
}
#endif	/* CONFIG_IDF_TARGET_ESP32S2 */

static uint16_t s_adc_cali_param[SOC_ADC_PERIPH_NUM][ADC_ATTEN_MAX] = {};

//NOTE: according to calibration version, different types of lock may be taken during the process:
//  1. Semaphore when reading efuse
//  2. Lock (Spinlock, or Mutex) if we actually do ADC calibration in the future
//This function shoudn't be called inside critical section or ISR
static uint32_t adc_get_calibration_offset(adc_ll_num_t adc_n, adc_channel_t channel, adc_atten_t atten)
{
    if (s_adc_cali_param[adc_n][atten]) {
        //LOG_DBG("Use calibrated val ADC%d atten=%d: %04X", adc_n, atten, s_adc_cali_param[adc_n][atten]);
        return (uint32_t)s_adc_cali_param[adc_n][atten];
    }

    // check if we can fetch the values from eFuse.
    int version = esp_efuse_rtc_calib_get_ver();

    uint32_t init_code = 0;

    if (version == ESP_EFUSE_ADC_CALIB_VER) {
        init_code = esp_efuse_rtc_calib_get_init_code(version, adc_n, atten);

    } else {
        //LOG_DBG("Calibration eFuse is not configured, use self-calibration for ICode");
        adc_power_acquire();
        ADC_ENTER_CRITICAL();
        const bool internal_gnd = true;
        init_code = adc_hal_self_calibration(adc_n, channel, atten, internal_gnd);
        ADC_EXIT_CRITICAL();
        adc_power_release();
    }


    s_adc_cali_param[adc_n][atten] = init_code;
    //LOG_DBG("Calib(V%d) ADC%d atten=%d: %04X", version, adc_n, atten, init_code);

    return init_code;
}

// Internal function to calibrate PWDET for WiFi
esp_err_t adc_cal_offset(adc_ll_num_t adc_n, adc_channel_t channel, adc_atten_t atten)
{
    adc_hal_calibration_init(adc_n);
    uint32_t cal_val = adc_get_calibration_offset(adc_n, channel, atten);
    ADC_ENTER_CRITICAL();
    adc_hal_set_calibration_param(adc_n, cal_val);
    ADC_EXIT_CRITICAL();
    return ESP_OK;
}
#endif //#if SOC_ADC_CALIBRATION_V1_SUPPORTED
