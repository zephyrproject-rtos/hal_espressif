/*
 * SPDX-FileCopyrightText: 2015-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdarg.h>
#include <sys/param.h>  //For max/min
#include "esp_attr.h"
#include "esp_private/system_internal.h"
#include "esp_flash.h"
#include "esp_flash_partitions.h"
#include "hal/spi_types.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "esp_compiler.h"
#include "esp_rom_sys.h"
#include "esp_private/spi_flash_os.h"
#include "esp_private/cache_utils.h"

#include "esp_private/spi_common_internal.h"

#include <zephyr/kernel.h>

#define SPI_FLASH_CACHE_NO_DISABLE  (CONFIG_SPI_FLASH_AUTO_SUSPEND || (CONFIG_SPIRAM_FETCH_INSTRUCTIONS && CONFIG_SPIRAM_RODATA) || CONFIG_APP_BUILD_TYPE_RAM)
static const char TAG[] = "spi_flash";

#if SPI_FLASH_CACHE_NO_DISABLE
K_MUTEX_DEFINE(s_spi1_flash_mutex);
static unsigned int s_spi1_key;
#endif  //  #if SPI_FLASH_CACHE_NO_DISABLE

/*
 * OS functions providing delay service and arbitration among chips, and with the cache.
 *
 * The cache needs to be disabled when chips on the SPI1 bus is under operation, hence these functions need to be put
 * into the IRAM,and their data should be put into the DRAM.
 */

/*
 * Time yield algorithm:
 * Every time spi_flash_os_check_yield() is called:
 *
 * 1. If the time since last end() function is longer than CONFIG_SPI_FLASH_ERASE_YIELD_TICKS (time
 *     to yield), all counters will be reset, as if the yield has just ends;
 * 2. If the time since last yield() is longer than CONFIG_SPI_FLASH_ERASE_YIELD_DURATION_MS, will
 *    return a yield request. When the yield() is called, all counters will be reset.
 * Note: Short intervals between start() and end() after the last yield() will not reset the
 *       counter mentioned in #2, but still be counted into the time mentioned in #2.
 */
typedef struct {
    spi_bus_lock_dev_handle_t dev_lock;
    bool no_protect;    //to decide whether to check protected region (for the main chip) or not.
    uint32_t acquired_since_us;    // Time since last explicit yield()
    uint32_t released_since_us;    // Time since last end() (implicit yield)
} app_func_arg_t;

static inline void on_spi_released(app_func_arg_t* ctx);
static inline void on_spi_acquired(app_func_arg_t* ctx);
static inline void on_spi_yielded(app_func_arg_t* ctx);
static inline bool on_spi_check_yield(app_func_arg_t* ctx);

#if !SPI_FLASH_CACHE_NO_DISABLE
IRAM_ATTR static void cache_enable(void* arg)
{
    spi_flash_enable_interrupts_caches_and_other_cpu();
}

IRAM_ATTR static void cache_disable(void* arg)
{
    spi_flash_disable_interrupts_caches_and_other_cpu();
}
#endif  //#if !SPI_FLASH_CACHE_NO_DISABLE

static IRAM_ATTR esp_err_t spi1_start(void *arg)
{
    esp_err_t ret = ESP_OK;
    /**
     * There are three ways for ESP Flash API lock:
     * 1. spi bus lock, this is used when SPI1 is shared with GPSPI Master Driver (not yet supported in Zephyr)
     * 2. mutex, this is used when the Cache isn't need to be disabled.
     * 3. cache lock (from cache_utils.h), this is used when we need to disable Cache to avoid access from SPI0
     *
     * From 1 to 3, the lock efficiency decreases.
     */
#if SPI_FLASH_CACHE_NO_DISABLE
    if (k_is_pre_kernel()) {
        s_spi1_key = irq_lock();
    } else {
        k_mutex_lock(&s_spi1_flash_mutex, K_FOREVER);
    }
#else
    //directly disable the cache and interrupts when lock is not used
    cache_disable(NULL);
#endif
    on_spi_acquired((app_func_arg_t*)arg);
    return ret;
}

static IRAM_ATTR esp_err_t spi1_end(void *arg)
{
    esp_err_t ret = ESP_OK;

    /**
     * There are three ways for ESP Flash API lock, see `spi1_start`
     */
#if SPI_FLASH_CACHE_NO_DISABLE
    if (k_is_pre_kernel()) {
        irq_unlock(s_spi1_key);
    } else {
        k_mutex_unlock(&s_spi1_flash_mutex);
    }
#else
    cache_enable(NULL);
#endif
    on_spi_released((app_func_arg_t*)arg);
    return ret;
}

static IRAM_ATTR esp_err_t spi_flash_os_check_yield(void *arg, uint32_t chip_status, uint32_t* out_request)
{
    assert (chip_status == 0);  //TODO: support suspend
    esp_err_t ret = ESP_ERR_TIMEOUT;    //Nothing happened
    uint32_t request = 0;

    if (on_spi_check_yield((app_func_arg_t *)arg)) {
        request = SPI_FLASH_YIELD_REQ_YIELD;
        ret = ESP_OK;
    }
    if (out_request) {
        *out_request = request;
    }
    return ret;
}

static IRAM_ATTR esp_err_t spi_flash_os_yield(void *arg, uint32_t* out_status)
{
    if (likely(!k_is_pre_kernel())) {
#ifdef CONFIG_SPI_FLASH_ERASE_YIELD_TICKS
        k_sleep(K_TICKS(CONFIG_SPI_FLASH_ERASE_YIELD_TICKS));
#else
        k_sleep(K_TICKS(1));
#endif
    }
    on_spi_yielded((app_func_arg_t*)arg);
    return ESP_OK;
}

static IRAM_ATTR esp_err_t delay_us(void *arg, uint32_t us)
{
    esp_rom_delay_us(us);
    return ESP_OK;
}

static IRAM_ATTR void* get_buffer_malloc(void* arg, size_t reqest_size, size_t* out_size)
{
    /* Allocate temporary internal buffer to use for the actual read. If the preferred size
        doesn't fit in free internal memory, allocate the largest available free block.

        (May need to shrink read_chunk_size and retry due to race conditions with other tasks
        also allocating from the heap.)
    */
    unsigned retries = 5;
    size_t read_chunk_size = reqest_size;
    void* ret = k_malloc(read_chunk_size);
    while(ret == NULL && retries-- && read_chunk_size > 0) {
        read_chunk_size = (read_chunk_size / 2 + 3) & ~3;
        ret = k_malloc(read_chunk_size);
    }
    ESP_LOGV(TAG, "allocate temp buffer: %p (%d)", ret, read_chunk_size);
    *out_size = (ret != NULL? read_chunk_size: 0);
    return ret;
}

static IRAM_ATTR void release_buffer_malloc(void* arg, void *temp_buf)
{
    k_free(temp_buf);
}

static IRAM_ATTR esp_err_t main_flash_region_protected(void* arg, size_t start_addr, size_t size)
{
    return ESP_OK;
}

#if CONFIG_SPI_FLASH_BROWNOUT_RESET
static IRAM_ATTR void main_flash_op_status(uint32_t op_status)
{
    bool is_erasing = op_status & SPI_FLASH_OS_IS_ERASING_STATUS_FLAG;
    spi_flash_set_erasing_flag(is_erasing);
}
#endif

static DRAM_ATTR app_func_arg_t main_flash_arg = {};

//for SPI1, we have to disable the cache and interrupts before using the SPI bus
static const DRAM_ATTR esp_flash_os_functions_t esp_flash_spi1_default_os_functions = {
    .start = spi1_start,
    .end = spi1_end,
    .region_protected = main_flash_region_protected,
    .delay_us = delay_us,
    .get_temp_buffer = get_buffer_malloc,
    .release_temp_buffer = release_buffer_malloc,
    .check_yield = spi_flash_os_check_yield,
    .yield = spi_flash_os_yield,
#if CONFIG_SPI_FLASH_BROWNOUT_RESET
    .set_flash_op_status = main_flash_op_status,
#else
    .set_flash_op_status = NULL,
#endif
};

static bool use_bus_lock(int host_id)
{
    if (host_id != SPI1_HOST) {
        return true;
    }

    return false;
}

// This function is only called by users usually via `spi_bus_add_flash_device` to initialise os functions.
// System will initialise them via `esp_flash_app_enable_os_functions`
esp_err_t esp_flash_init_os_functions(esp_flash_t *chip, int host_id, spi_bus_lock_dev_handle_t dev_handle)
{
    if (use_bus_lock(host_id) && !dev_handle) {
        return ESP_ERR_INVALID_ARG;
    }

    chip->os_func_data = k_malloc(sizeof(app_func_arg_t));
    if (chip->os_func_data == NULL) {
        return ESP_ERR_NO_MEM;
    }

    switch (host_id) {
        case SPI1_HOST:
            chip->os_func = &esp_flash_spi1_default_os_functions;
            break;
        default:
            return ESP_ERR_INVALID_ARG;
            break;
    }

    *(app_func_arg_t*) chip->os_func_data = (app_func_arg_t) {
        .dev_lock = dev_handle,
        .no_protect = true, // This is OK because this code path isn't used for the main flash chip which requires `no_protect = false`
    };

    return ESP_OK;
}

esp_err_t esp_flash_deinit_os_functions(esp_flash_t* chip, spi_bus_lock_dev_handle_t* out_dev_handle)
{
    if (chip->os_func_data) {
        // SPI bus lock is possibly not used on SPI1 bus
        *out_dev_handle = ((app_func_arg_t*)chip->os_func_data)->dev_lock;
        k_free(chip->os_func_data);
    }
    chip->os_func = NULL;
    chip->os_func_data = NULL;
    return ESP_OK;
}

esp_err_t esp_flash_app_enable_os_functions(esp_flash_t* chip)
{
    main_flash_arg = (app_func_arg_t) {
        .dev_lock = 0,
        .no_protect = false, // Required for the main flash chip
    };
    chip->os_func = &esp_flash_spi1_default_os_functions;
    chip->os_func_data = &main_flash_arg;
    return ESP_OK;
}

// The goal of this part is to manually insert one valid task execution interval, if the time since
// last valid interval exceed the limitation (CONFIG_SPI_FLASH_ERASE_YIELD_DURATION_MS).
//
// Valid task execution interval: continuous time with the cache enabled, which is longer than
// CONFIG_SPI_FLASH_ERASE_YIELD_TICKS. Yield time shorter than CONFIG_SPI_FLASH_ERASE_YIELD_TICKS is
// not treated as valid interval.
static inline IRAM_ATTR bool on_spi_check_yield(app_func_arg_t* ctx)
{
#ifdef CONFIG_SPI_FLASH_YIELD_DURING_ERASE
    uint32_t time = k_uptime_get_32();
    // We handle the reset here instead of in `on_spi_acquired()`, when acquire() and release() is
    // larger than CONFIG_SPI_FLASH_ERASE_YIELD_TICKS, to save one `esp_system_get_time()` call
    if ((time - ctx->released_since_us) >= CONFIG_SPI_FLASH_ERASE_YIELD_TICKS * portTICK_PERIOD_MS * 1000) {
        // Reset the acquired time as if the yield has just happened.
        ctx->acquired_since_us = time;
    } else if ((time - ctx->acquired_since_us) >= CONFIG_SPI_FLASH_ERASE_YIELD_DURATION_MS * 1000) {
        return true;
    }
#endif
    return false;
}
static inline IRAM_ATTR void on_spi_released(app_func_arg_t* ctx)
{
#ifdef CONFIG_SPI_FLASH_YIELD_DURING_ERASE
    ctx->released_since_us = k_uptime_get_32();
#endif
}

static inline IRAM_ATTR void on_spi_acquired(app_func_arg_t* ctx)
{
    // Ideally, when the time after `on_spi_released()` before this function is called is larger
    // than CONFIG_SPI_FLASH_ERASE_YIELD_TICKS, the acquired time should be reset. We assume the
    // time after `on_spi_check_yield()` before this function is so short that we can do the reset
    // in that function instead.
}

static inline IRAM_ATTR void on_spi_yielded(app_func_arg_t* ctx)
{
    uint32_t time = k_uptime_get_32();
    ctx->acquired_since_us = time;
}
