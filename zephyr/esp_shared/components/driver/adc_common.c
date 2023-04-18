/*
 * SPDX-FileCopyrightText: 2019-2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <esp_types.h>
#include <stdlib.h>
#include "esp_err.h"
#include "driver/rtc_io.h"
#include "driver/adc.h"
#include "hal/adc_types.h"
#include "hal/adc_hal.h"
#include "hal/adc_hal_conf.h"
#include <zephyr/kernel.h>
#include "esp_log.h"
#include "soc/periph_defs.h"

#if SOC_DAC_SUPPORTED
#include "driver/dac.h"
#include "hal/dac_hal.h"
#endif

#if CONFIG_IDF_TARGET_ESP32S3
#include "esp_efuse_rtc_calib.h"
#endif

#if CONFIG_IDF_TARGET_ESP32C3
#include "esp_efuse_rtc_calib.h"
#endif

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(adc_common, CONFIG_ADC_LOG_LEVEL);

#define ADC_CHECK_RET(fun_ret) ({                           \
    if (fun_ret != ESP_OK) {                                \
        LOG_ERR("%s:%d\n",__FUNCTION__,__LINE__);  \
        return ESP_FAIL;                                    \
    }                                                       \
})

static const char *ADC_TAG = "ADC";

#define ADC_CHECK(a, str, ret_val) ({                                               \
    if (!(a)) {                                                                     \
        LOG_ERR("%s(%d): %s", __FUNCTION__, __LINE__, str);                \
        return (ret_val);                                                           \
    }                                                                               \
})

#define ADC_GET_IO_NUM(periph, channel) (adc_channel_io_map[periph][channel])

#define ADC_CHANNEL_CHECK(periph, channel) ADC_CHECK(channel < SOC_ADC_CHANNEL_NUM(periph),	\
		"ADC"#periph" channel error", ESP_ERR_INVALID_ARG)

//////////////////////// Locks ///////////////////////////////////////////
struct adc_context {
    unsigned int spinlock;
};
struct adc_context adc_context = {
	.spinlock = 0,
};

#define RTC_ENTER_CRITICAL() do { adc_context.spinlock = irq_lock(); } while(0)
#define RTC_EXIT_CRITICAL() do { irq_unlock(adc_context.spinlock); } while(0)
#define DIGI_ENTER_CRITICAL()
#define DIGI_EXIT_CRITICAL()

#define ADC_ENTER_CRITICAL()	RTC_ENTER_CRITICAL()
#define ADC_EXIT_CRITICAL()	RTC_EXIT_CRITICAL()

#define ADC_POWER_ENTER()       RTC_ENTER_CRITICAL()
#define ADC_POWER_EXIT()        RTC_EXIT_CRITICAL()

#define DIGI_CONTROLLER_ENTER() DIGI_ENTER_CRITICAL()
#define DIGI_CONTROLLER_EXIT()  DIGI_EXIT_CRITICAL()

#define SARADC1_ENTER()         RTC_ENTER_CRITICAL()
#define SARADC1_EXIT()          RTC_EXIT_CRITICAL()

#define SARADC2_ENTER()         RTC_ENTER_CRITICAL()
#define SARADC2_EXIT()          RTC_EXIT_CRITICAL()

//n stands for ADC unit: 1 for ADC1 and 2 for ADC2. Currently both unit touches the same registers
#define VREF_ENTER(n)           RTC_ENTER_CRITICAL()
#define VREF_EXIT(n)            RTC_EXIT_CRITICAL()

#define FSM_ENTER()             RTC_ENTER_CRITICAL()
#define FSM_EXIT()              RTC_EXIT_CRITICAL()

//TODO: IDF-3610
//#if 1 //CONFIG_IDF_TARGET_ESP32 || CONFIG_IDF_TARGET_ESP32S2 || CONFIG_IDF_TARGET_ESP32S3
//prevent ADC1 being used by I2S dma and other tasks at the same time.
K_MUTEX_DEFINE(adc1_dma_lock);
#define SARADC1_ACQUIRE() do { k_mutex_lock(&adc1_dma_lock, K_FOREVER); } while(0)
#define SARADC1_RELEASE() do { k_mutex_unlock(&adc1_dma_lock); } while(0)
//#endif


/*
In ADC2, there're two locks used for different cases:
1. lock shared with app and Wi-Fi:
   ESP32:
        When Wi-Fi using the ADC2, we assume it will never stop, so app checks the lock and returns immediately if failed.
   ESP32S2:
        The controller's control over the ADC is determined by the arbiter. There is no need to control by lock.

2. lock shared between tasks:
   when several tasks sharing the ADC2, we want to guarantee
   all the requests will be handled.
   Since conversions are short (about 31us), app returns the lock very soon,
   we use a spinlock to stand there waiting to do conversions one by one.

adc2_spinlock should be acquired first, then adc2_wifi_lock or rtc_spinlock.
*/

#ifdef CONFIG_IDF_TARGET_ESP32
/* prevent ADC2 being used by wifi and other tasks at the same time. */
K_MUTEX_DEFINE(adc2_wifi_lock);
/** For ESP32S2 the ADC2 The right to use ADC2 is controlled by the arbiter, and there is no need to set a lock. */
#define SARADC2_ACQUIRE()  do { k_mutex_lock(&adc2_wifi_lock, K_FOREVER); } while(0)
#define SARADC2_RELEASE()  do { k_mutex_unlock(&adc2_wifi_lock); } while(0)
#define SARADC2_TRY_ACQUIRE() k_mutex_lock(&adc2_wifi_lock, K_NO_WAIT)
#define SARADC2_LOCK_CHECK()    (adc2_wifi_lock.lock_count != 0)

#elif CONFIG_IDF_TARGET_ESP32S2 || CONFIG_IDF_TARGET_ESP32S3
#define SARADC2_ACQUIRE()
#define SARADC2_RELEASE()
#define SARADC2_TRY_ACQUIRE()   (0)     //WIFI controller and rtc controller have independent parameter configuration.
#define SARADC2_LOCK_CHECK()    (true)

#endif // CONFIG_IDF_TARGET_*

#if CONFIG_IDF_TARGET_ESP32S2
#ifdef CONFIG_PM_ENABLE
static esp_pm_lock_handle_t s_adc2_arbiter_lock;
#endif  //CONFIG_PM_ENABLE
#endif  // !CONFIG_IDF_TARGET_ESP32

/*---------------------------------------------------------------
                    ADC Common
---------------------------------------------------------------*/
// ADC Power

// This gets incremented when adc_power_acquire() is called, and decremented when
// adc_power_release() is called. ADC is powered down when the value reaches zero.
// Should be modified within critical section (ADC_ENTER/EXIT_CRITICAL).
static int s_adc_power_on_cnt;

static void adc_power_on_internal(void)
{
    /* Set the power always on to increase precision. */
    adc_hal_set_power_manage(ADC_POWER_SW_ON);
}

void adc_power_acquire(void)
{
    ADC_POWER_ENTER();
    s_adc_power_on_cnt++;
    if (s_adc_power_on_cnt == 1) {
        adc_power_on_internal();
    }
    ADC_POWER_EXIT();
}

void adc_power_on(void)
{
    ADC_POWER_ENTER();
    adc_power_on_internal();
    ADC_POWER_EXIT();
}

static void adc_power_off_internal(void)
{
#if CONFIG_IDF_TARGET_ESP32
    adc_hal_set_power_manage(ADC_POWER_SW_OFF);
#else
    adc_hal_set_power_manage(ADC_POWER_BY_FSM);
#endif
}

void adc_power_release(void)
{
    ADC_POWER_ENTER();
    s_adc_power_on_cnt--;
    /* Sanity check */
    if (s_adc_power_on_cnt < 0) {
        ADC_POWER_EXIT();
        ESP_LOGE(ADC_TAG, "%s called, but s_adc_power_on_cnt == 0", __func__);
        abort();
    } else if (s_adc_power_on_cnt == 0) {
        adc_power_off_internal();
    }
    ADC_POWER_EXIT();
}

void adc_power_off(void)
{
    ADC_POWER_ENTER();
    adc_power_off_internal();
    ADC_POWER_EXIT();
}

esp_err_t adc1_pad_get_io_num(adc1_channel_t channel, gpio_num_t *gpio_num)
{
    ADC_CHANNEL_CHECK(ADC_NUM_1, channel);

    int io = ADC_GET_IO_NUM(ADC_NUM_1, channel);
    if (io < 0) {
        return ESP_ERR_INVALID_ARG;
    } else {
        *gpio_num = (gpio_num_t)io;
    }

    return ESP_OK;
}

esp_err_t adc2_pad_get_io_num(adc2_channel_t channel, gpio_num_t *gpio_num)
{
    ADC_CHANNEL_CHECK(ADC_NUM_2, channel);

    int io = ADC_GET_IO_NUM(ADC_NUM_2, channel);
    if (io < 0) {
        return ESP_ERR_INVALID_ARG;
    } else {
        *gpio_num = (gpio_num_t)io;
    }

    return ESP_OK;
}

//------------------------------------------------------------RTC Single Read----------------------------------------------//
#if SOC_ADC_RTC_CTRL_SUPPORTED
#if SOC_ADC_CALIBRATION_V1_SUPPORTED
static uint32_t get_calibration_offset(adc_ll_num_t adc_n, adc_channel_t chan)
{
    adc_atten_t atten = adc_ll_get_atten(adc_n, chan);
    extern uint32_t adc_get_calibration_offset(adc_ll_num_t adc_n, adc_channel_t channel, adc_atten_t atten);

    return adc_get_calibration_offset(adc_n, chan, atten);
}
#endif  //SOC_ADC_CALIBRATION_V1_SUPPORTED

esp_err_t adc_set_clk_div(uint8_t clk_div)
{
    DIGI_CONTROLLER_ENTER();
    adc_ll_digi_set_clk_div(clk_div);
    DIGI_CONTROLLER_EXIT();
    return ESP_OK;
}

static void adc_rtc_chan_init(adc_unit_t adc_unit)
{
    if (adc_unit & ADC_UNIT_1) {
        /* Workaround: Disable the synchronization operation function of ADC1 and DAC.
           If enabled(default), ADC RTC controller sampling will cause the DAC channel output voltage. */
#if SOC_DAC_SUPPORTED
        dac_hal_rtc_sync_by_adc(false);
#endif
        adc_hal_rtc_output_invert(ADC_NUM_1, SOC_ADC1_DATA_INVERT_DEFAULT);
        adc_ll_set_sar_clk_div(ADC_NUM_1, SOC_ADC_SAR_CLK_DIV_DEFAULT(ADC_NUM_1));
#ifdef CONFIG_IDF_TARGET_ESP32
        adc_ll_hall_disable(); //Disable other peripherals.
        adc_ll_amp_disable();  //Currently the LNA is not open, close it by default.
#endif
    }
    if (adc_unit & ADC_UNIT_2) {
        adc_hal_pwdet_set_cct(SOC_ADC_PWDET_CCT_DEFAULT);
        adc_hal_rtc_output_invert(ADC_NUM_2, SOC_ADC2_DATA_INVERT_DEFAULT);
        adc_ll_set_sar_clk_div(ADC_NUM_2, SOC_ADC_SAR_CLK_DIV_DEFAULT(ADC_NUM_2));
    }
}

/**
 * This function is NOT an API.
 * Now some to-be-deprecated APIs are using this function, so don't make it static for now.
 * Will make this static on v5.0
 */
esp_err_t adc_common_gpio_init(adc_unit_t adc_unit, adc_channel_t channel)
{
    gpio_num_t gpio_num = 0;
    //If called with `ADC_UNIT_BOTH (ADC_UNIT_1 | ADC_UNIT_2)`, both if blocks will be run
    if (adc_unit & ADC_UNIT_1) {
        ADC_CHANNEL_CHECK(ADC_NUM_1, channel);
        gpio_num = ADC_GET_IO_NUM(ADC_NUM_1, channel);
        ADC_CHECK_RET(rtc_gpio_init(gpio_num));
        ADC_CHECK_RET(rtc_gpio_set_direction(gpio_num, RTC_GPIO_MODE_DISABLED));
        ADC_CHECK_RET(rtc_gpio_pulldown_dis(gpio_num));
        ADC_CHECK_RET(rtc_gpio_pullup_dis(gpio_num));
    }
    if (adc_unit & ADC_UNIT_2) {
        ADC_CHANNEL_CHECK(ADC_NUM_2, channel);
        gpio_num = ADC_GET_IO_NUM(ADC_NUM_2, channel);
        ADC_CHECK_RET(rtc_gpio_init(gpio_num));
        ADC_CHECK_RET(rtc_gpio_set_direction(gpio_num, RTC_GPIO_MODE_DISABLED));
        ADC_CHECK_RET(rtc_gpio_pulldown_dis(gpio_num));
        ADC_CHECK_RET(rtc_gpio_pullup_dis(gpio_num));
    }

    return ESP_OK;
}

esp_err_t adc_set_data_inv(adc_unit_t adc_unit, bool inv_en)
{
    if (adc_unit & ADC_UNIT_1) {
        SARADC1_ENTER();
        adc_hal_rtc_output_invert(ADC_NUM_1, inv_en);
        SARADC1_EXIT();
    }
    if (adc_unit & ADC_UNIT_2) {
        SARADC2_ENTER();
        adc_hal_rtc_output_invert(ADC_NUM_2, inv_en);
        SARADC2_EXIT();
    }

    return ESP_OK;
}

esp_err_t adc_set_data_width(adc_unit_t adc_unit, adc_bits_width_t width_bit)
{
#if CONFIG_IDF_TARGET_ESP32
    ADC_CHECK(width_bit < ADC_WIDTH_MAX, "WIDTH ERR: ESP32 support 9 ~ 12 bit width", ESP_ERR_INVALID_ARG);
#else
    ADC_CHECK(width_bit == ADC_WIDTH_MAX - 1, "WIDTH ERR: see `adc_bits_width_t` for supported bit width", ESP_ERR_INVALID_ARG);
#endif

    if (adc_unit & ADC_UNIT_1) {
        SARADC1_ENTER();
        adc_hal_rtc_set_output_format(ADC_NUM_1, width_bit);
        SARADC1_EXIT();
    }
    if (adc_unit & ADC_UNIT_2) {
        SARADC2_ENTER();
        adc_hal_rtc_set_output_format(ADC_NUM_2, width_bit);
        SARADC2_EXIT();
    }

    return ESP_OK;
}

/**
 * @brief Reset RTC controller FSM.
 *
 * @return
 *      - ESP_OK Success
 */
#if !CONFIG_IDF_TARGET_ESP32
esp_err_t adc_rtc_reset(void)
{
    FSM_ENTER();
    adc_ll_rtc_reset();
    FSM_EXIT();
    return ESP_OK;
}
#endif

/*-------------------------------------------------------------------------------------
 *                      ADC1
 *------------------------------------------------------------------------------------*/
esp_err_t adc1_config_channel_atten(adc1_channel_t channel, adc_atten_t atten)
{
    ADC_CHANNEL_CHECK(ADC_NUM_1, channel);
    ADC_CHECK(atten < ADC_ATTEN_MAX, "ADC Atten Err", ESP_ERR_INVALID_ARG);

    adc_common_gpio_init(ADC_UNIT_1, channel);
    SARADC1_ENTER();
    adc_rtc_chan_init(ADC_UNIT_1);
    adc_hal_set_atten(ADC_NUM_1, channel, atten);
    SARADC1_EXIT();

#if SOC_ADC_CALIBRATION_V1_SUPPORTED
    adc_hal_calibration_init(ADC_NUM_1);
#endif

    return ESP_OK;
}

esp_err_t adc1_config_width(adc_bits_width_t width_bit)
{
#if CONFIG_IDF_TARGET_ESP32
    ADC_CHECK(width_bit < ADC_WIDTH_MAX, "WIDTH ERR: ESP32 support 9 ~ 12 bit width", ESP_ERR_INVALID_ARG);
#else
    ADC_CHECK(width_bit == ADC_WIDTH_MAX - 1, "WIDTH ERR: see `adc_bits_width_t` for supported bit width", ESP_ERR_INVALID_ARG);
#endif

    SARADC1_ENTER();
    adc_hal_rtc_set_output_format(ADC_NUM_1, width_bit);
    SARADC1_EXIT();

    return ESP_OK;
}

esp_err_t adc1_dma_mode_acquire(void)
{
    /* Use locks to avoid digtal and RTC controller conflicts.
       for adc1, block until acquire the lock. */
    SARADC1_ACQUIRE();
    ESP_LOGD( ADC_TAG, "dma mode takes adc1 lock." );

    adc_power_acquire();

    SARADC1_ENTER();
    /* switch SARADC into DIG channel */
    adc_ll_set_controller(ADC_NUM_1, ADC_LL_CTRL_DIG);
    SARADC1_EXIT();

    return ESP_OK;
}

esp_err_t adc1_rtc_mode_acquire(void)
{
    /* Use locks to avoid digtal and RTC controller conflicts.
       for adc1, block until acquire the lock. */
    SARADC1_ACQUIRE();
    adc_power_acquire();

    SARADC1_ENTER();
    /* switch SARADC into RTC channel. */
    adc_ll_set_controller(ADC_NUM_1, ADC_LL_CTRL_RTC);
    SARADC1_EXIT();

    return ESP_OK;
}

esp_err_t adc1_lock_release(void)
{
    ADC_CHECK(adc1_dma_lock.lock_count != 0, "adc1 lock release called before acquire", ESP_ERR_INVALID_STATE );
    /* Use locks to avoid digtal and RTC controller conflicts. for adc1, block until acquire the lock. */

    adc_power_release();
    SARADC1_RELEASE();
    return ESP_OK;
}

int adc1_get_raw(adc1_channel_t channel)
{
    int adc_value;
    ADC_CHANNEL_CHECK(ADC_NUM_1, channel);
    adc1_rtc_mode_acquire();

#if SOC_ADC_CALIBRATION_V1_SUPPORTED
    // Get calibration value before going into critical section
    uint32_t cal_val = get_calibration_offset(ADC_NUM_1, channel);
    adc_hal_set_calibration_param(ADC_NUM_1, cal_val);
#endif  //SOC_ADC_CALIBRATION_V1_SUPPORTED

    SARADC1_ENTER();
#ifdef CONFIG_IDF_TARGET_ESP32
    adc_ll_hall_disable(); //Disable other peripherals.
    adc_ll_amp_disable();  //Currently the LNA is not open, close it by default.
#endif
    adc_ll_set_controller(ADC_NUM_1, ADC_LL_CTRL_RTC);    //Set controller
    adc_hal_convert(ADC_NUM_1, channel, &adc_value);   //Start conversion, For ADC1, the data always valid.
#if !CONFIG_IDF_TARGET_ESP32
    adc_ll_rtc_reset();    //Reset FSM of rtc controller
#endif
    SARADC1_EXIT();

    adc1_lock_release();
    return adc_value;
}

int adc1_get_voltage(adc1_channel_t channel)    //Deprecated. Use adc1_get_raw() instead
{
    return adc1_get_raw(channel);
}

#if SOC_ULP_SUPPORTED
void adc1_ulp_enable(void)
{
    adc_power_acquire();

    SARADC1_ENTER();
    adc_ll_set_controller(ADC_NUM_1, ADC_LL_CTRL_ULP);
    /* since most users do not need LNA and HALL with uLP, we disable them here
       open them in the uLP if needed. */
#ifdef CONFIG_IDF_TARGET_ESP32
    /* disable other peripherals. */
    adc_ll_hall_disable();
    adc_ll_amp_disable();
#endif
    SARADC1_EXIT();
}
#endif

/*---------------------------------------------------------------
                    ADC2
---------------------------------------------------------------*/
/** For ESP32S2 the ADC2 The right to use ADC2 is controlled by the arbiter, and there is no need to set a lock.*/
esp_err_t adc2_wifi_acquire(void)
{
    /* Wi-Fi module will use adc2. Use locks to avoid conflicts. */
    SARADC2_ACQUIRE();
    ESP_LOGD( ADC_TAG, "Wi-Fi takes adc2 lock." );
    return ESP_OK;
}

esp_err_t adc2_wifi_release(void)
{
    ADC_CHECK(SARADC2_LOCK_CHECK(), "wifi release called before acquire", ESP_ERR_INVALID_STATE );
    SARADC2_RELEASE();
    ESP_LOGD( ADC_TAG, "Wi-Fi returns adc2 lock." );

    return ESP_OK;
}

esp_err_t adc2_config_channel_atten(adc2_channel_t channel, adc_atten_t atten)
{
    ADC_CHANNEL_CHECK(ADC_NUM_2, channel);
    ADC_CHECK(atten <= ADC_ATTEN_11db, "ADC2 Atten Err", ESP_ERR_INVALID_ARG);

    adc_common_gpio_init(ADC_UNIT_2, channel);

    if ( SARADC2_TRY_ACQUIRE() == -1 ) {
        //try the lock, return if failed (wifi using).
        return ESP_ERR_TIMEOUT;
    }

    //avoid collision with other tasks
    SARADC2_ENTER();
    adc_rtc_chan_init(ADC_UNIT_2);
    adc_hal_set_atten(ADC_NUM_2, channel, atten);
    SARADC2_EXIT();

    SARADC2_RELEASE();

#if SOC_ADC_CALIBRATION_V1_SUPPORTED
    adc_hal_calibration_init(ADC_NUM_2);
#endif

    return ESP_OK;
}

static inline void adc2_init(void)
{
#if CONFIG_IDF_TARGET_ESP32S2
#ifdef CONFIG_PM_ENABLE
    /* Lock APB clock. */
    if (s_adc2_arbiter_lock == NULL) {
        esp_pm_lock_create(ESP_PM_APB_FREQ_MAX, 0, "adc2", &s_adc2_arbiter_lock);
    }
#endif  //CONFIG_PM_ENABLE
#endif  //CONFIG_IDF_TARGET_ESP32S2
}

static inline void adc2_dac_disable( adc2_channel_t channel)
{
#if SOC_DAC_SUPPORTED
#ifdef CONFIG_IDF_TARGET_ESP32
    if ( channel == ADC2_CHANNEL_8 ) { // the same as DAC channel 1
        dac_output_disable(DAC_CHANNEL_1);
    } else if ( channel == ADC2_CHANNEL_9 ) {
        dac_output_disable(DAC_CHANNEL_2);
    }
#else
    if ( channel == ADC2_CHANNEL_6 ) { // the same as DAC channel 1
        dac_output_disable(DAC_CHANNEL_1);
    } else if ( channel == ADC2_CHANNEL_7 ) {
        dac_output_disable(DAC_CHANNEL_2);
    }
#endif
#endif // SOC_DAC_SUPPORTED
}

/**
 * @note For ESP32S2:
 *       The arbiter's working clock is APB_CLK. When the APB_CLK clock drops below 8 MHz, the arbiter must be in shield mode.
 *       Or, the RTC controller will fail when get raw data.
 *       This issue does not occur on digital controllers (DMA mode), and the hardware guarantees that there will be no errors.
 */
esp_err_t adc2_get_raw(adc2_channel_t channel, adc_bits_width_t width_bit, int *raw_out)
{
    esp_err_t ret = ESP_OK;
    int adc_value = 0;

    ADC_CHECK(raw_out != NULL, "ADC out value err", ESP_ERR_INVALID_ARG);
    ADC_CHECK(channel < ADC2_CHANNEL_MAX, "ADC Channel Err", ESP_ERR_INVALID_ARG);
#if CONFIG_IDF_TARGET_ESP32
    ADC_CHECK(width_bit < ADC_WIDTH_MAX, "WIDTH ERR: ESP32 support 9 ~ 12 bit width", ESP_ERR_INVALID_ARG);
#else
    ADC_CHECK(width_bit == ADC_WIDTH_MAX - 1, "WIDTH ERR: see `adc_bits_width_t` for supported bit width", ESP_ERR_INVALID_ARG);
#endif

#if SOC_ADC_CALIBRATION_V1_SUPPORTED
    // Get calibration value before going into critical section
    uint32_t cal_val = get_calibration_offset(ADC_NUM_2, channel);
    adc_hal_set_calibration_param(ADC_NUM_2, cal_val);
#endif  //SOC_ADC_CALIBRATION_V1_SUPPORTED

    if ( SARADC2_TRY_ACQUIRE() == -1 ) {
        //try the lock, return if failed (wifi using).
        return ESP_ERR_TIMEOUT;
    }
    adc_power_acquire();         //in critical section with whole rtc module

    //avoid collision with other tasks
    adc2_init();   // in critical section with whole rtc module. because the PWDET use the same registers, place it here.
    SARADC2_ENTER();

#if SOC_ADC_ARBITER_SUPPORTED
    adc_arbiter_t config = ADC_ARBITER_CONFIG_DEFAULT();
    adc_hal_arbiter_config(&config);
#endif

#ifdef CONFIG_ADC_DISABLE_DAC
    adc2_dac_disable(channel);      //disable other peripherals
#endif
    adc_hal_rtc_set_output_format(ADC_NUM_2, width_bit);

#if CONFIG_IDF_TARGET_ESP32
    adc_ll_set_controller(ADC_NUM_2, ADC_LL_CTRL_RTC);// set controller
#else
    adc_ll_set_controller(ADC_NUM_2, ADC_LL_CTRL_ARB);// set controller
#endif

#if CONFIG_IDF_TARGET_ESP32S2
#ifdef CONFIG_PM_ENABLE
    if (s_adc2_arbiter_lock) {
        esp_pm_lock_acquire(s_adc2_arbiter_lock);
    }
#endif //CONFIG_PM_ENABLE
#endif //CONFIG_IDF_TARGET_ESP32

    ret = adc_hal_convert(ADC_NUM_2, channel, &adc_value);
    if (ret != ESP_OK) {
        adc_value = -1;
    }

#if CONFIG_IDF_TARGET_ESP32S2
#ifdef CONFIG_PM_ENABLE
    /* Release APB clock. */
    if (s_adc2_arbiter_lock) {
        esp_pm_lock_release(s_adc2_arbiter_lock);
    }
#endif //CONFIG_PM_ENABLE
#endif //CONFIG_IDF_TARGET_ESP32
    SARADC2_EXIT();

    adc_power_release();
    SARADC2_RELEASE();

    *raw_out = adc_value;
    return ret;
}

esp_err_t adc2_vref_to_gpio(gpio_num_t gpio)
{
    return adc_vref_to_gpio(ADC_UNIT_2, gpio);
}

esp_err_t adc_vref_to_gpio(adc_unit_t adc_unit, gpio_num_t gpio)
{
#ifdef CONFIG_IDF_TARGET_ESP32
    if (adc_unit & ADC_UNIT_1) {
        return ESP_ERR_INVALID_ARG;
    }
#endif
    adc2_channel_t ch = ADC2_CHANNEL_MAX;
    /* Check if the GPIO supported. */
    for (int i = 0; i < ADC2_CHANNEL_MAX; i++) {
        if (gpio == ADC_GET_IO_NUM(ADC_NUM_2, i)) {
            ch = i;
            break;
        }
    }
    if (ch == ADC2_CHANNEL_MAX) {
        return ESP_ERR_INVALID_ARG;
    }

    adc_power_acquire();
    if (adc_unit & ADC_UNIT_1) {
        VREF_ENTER(1);
        adc_hal_vref_output(ADC_NUM_1, ch, true);
        VREF_EXIT(1);
    } else if (adc_unit & ADC_UNIT_2) {
        VREF_ENTER(2);
        adc_hal_vref_output(ADC_NUM_2, ch, true);
        VREF_EXIT(2);
    }

    //Configure RTC gpio, Only ADC2's channels IO are supported to output reference voltage.
    adc_common_gpio_init(ADC_UNIT_2, ch);
    return ESP_OK;
}

#endif //SOC_ADC_RTC_CTRL_SUPPORTED


#if SOC_ADC_CALIBRATION_V1_SUPPORTED
uint32_t adc_get_calibration_offset(adc_ll_num_t adc_n, adc_channel_t chan, adc_atten_t atten);
#endif

#if SOC_ADC_CALIBRATION_V1_SUPPORTED
/*---------------------------------------------------------------
                    Hardware Calibration Setting
---------------------------------------------------------------*/
#if CONFIG_IDF_TARGET_ESP32S2
  #include "esp_efuse_rtc_table.h"
  #define esp_efuse_rtc_calib_get_ver()                               esp_efuse_rtc_table_read_calib_version()

static inline uint32_t esp_efuse_rtc_calib_get_init_code(int version, uint32_t adc_unit, int atten)
{
    int tag = esp_efuse_rtc_table_get_tag(version, adc_unit + 1, atten, RTCCALIB_V2_PARAM_VINIT);
    return esp_efuse_rtc_table_get_parsed_efuse_value(tag, false);
}
#endif	/* CONFIG_IDF_TARGET_ESP32S2 */

#if !CONFIG_IDF_TARGET_ESP32C3
static uint16_t s_adc_cali_param[SOC_ADC_PERIPH_NUM][ADC_ATTEN_MAX] = {};

//NOTE: according to calibration version, different types of lock may be taken during the process:
//  1. Semaphore when reading efuse
//  2. Lock (Spinlock, or Mutex) if we actually do ADC calibration in the future
//This function shoudn't be called inside critical section or ISR
uint32_t adc_get_calibration_offset(adc_ll_num_t adc_n, adc_channel_t channel, adc_atten_t atten)
{
    if (s_adc_cali_param[adc_n][atten]) {
        //ESP_LOGV(ADC_TAG, "Use calibrated val ADC%d atten=%d: %04X", adc_n, atten, s_adc_cali_param[adc_n][atten]);
        return (uint32_t)s_adc_cali_param[adc_n][atten];
    }

    // check if we can fetch the values from eFuse.
    int version = esp_efuse_rtc_calib_get_ver();

    uint32_t init_code = 0;

    if (version == ESP_EFUSE_ADC_CALIB_VER) {
        init_code = esp_efuse_rtc_calib_get_init_code(version, adc_n, atten);

    } else {
        ESP_LOGD(ADC_TAG, "Calibration eFuse is not configured, use self-calibration for ICode");
        adc_power_acquire();
        ADC_ENTER_CRITICAL();
        const bool internal_gnd = true;
        init_code = adc_hal_self_calibration(adc_n, channel, atten, internal_gnd);
        ADC_EXIT_CRITICAL();
        adc_power_release();
    }

    s_adc_cali_param[adc_n][atten] = init_code;
    //ESP_LOGV(ADC_TAG, "Calib(V%d) ADC%d atten=%d: %04X", version, adc_n, atten, init_code);

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
#endif // CONFIG_IDF_TARGET_ESP32C3
#endif // SOC_ADC_CALIBRATION_V1_SUPPORTED
