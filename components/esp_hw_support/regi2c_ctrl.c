/*
 * SPDX-FileCopyrightText: 2020-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */


#include "esp_attr.h"
#include <stdint.h>
#include <zephyr/kernel.h>

#include "hal/regi2c_ctrl.h"
#include "hal/regi2c_ctrl_ll.h"
#include "esp_hw_log.h"

static int mux;

#define ENTER_CRITICAL_SECTION()    do { mux = irq_lock(); } while(0)
#define LEAVE_CRITICAL_SECTION()    irq_unlock(mux);

static DRAM_ATTR __attribute__((unused)) const char *TAG = "REGI2C";

#ifndef BOOTLOADER_BUILD
uint8_t IRAM_ATTR regi2c_ctrl_read_reg(uint8_t block, uint8_t host_id, uint8_t reg_add)
{
    ENTER_CRITICAL_SECTION();
    uint8_t value = regi2c_read_reg_raw(block, host_id, reg_add);
    LEAVE_CRITICAL_SECTION();
    return value;
}

uint8_t IRAM_ATTR regi2c_ctrl_read_reg_mask(uint8_t block, uint8_t host_id, uint8_t reg_add, uint8_t msb, uint8_t lsb)
{
    ENTER_CRITICAL_SECTION();
    uint8_t value = regi2c_read_reg_mask_raw(block, host_id, reg_add, msb, lsb);
    LEAVE_CRITICAL_SECTION();
    return value;
}

void IRAM_ATTR regi2c_ctrl_write_reg(uint8_t block, uint8_t host_id, uint8_t reg_add, uint8_t data)
{
    ENTER_CRITICAL_SECTION();
    regi2c_write_reg_raw(block, host_id, reg_add, data);
    LEAVE_CRITICAL_SECTION();
}

void IRAM_ATTR regi2c_ctrl_write_reg_mask(uint8_t block, uint8_t host_id, uint8_t reg_add, uint8_t msb, uint8_t lsb, uint8_t data)
{
    ENTER_CRITICAL_SECTION();
    regi2c_write_reg_mask_raw(block, host_id, reg_add, msb, lsb, data);
    LEAVE_CRITICAL_SECTION();
}
#endif /* BOOTLOADER_BUILD */

void IRAM_ATTR regi2c_enter_critical(void)
{
    ENTER_CRITICAL_SECTION();
}

void IRAM_ATTR regi2c_exit_critical(void)
{
    LEAVE_CRITICAL_SECTION();
}

/**
 * Restore regi2c analog calibration related configuration registers.
 * This is a workaround, and is fixed on later chips
 */
#if REGI2C_ANA_CALI_PD_WORKAROUND
#include "soc/regi2c_saradc.h"

static DRAM_ATTR uint8_t reg_val[REGI2C_ANA_CALI_BYTE_NUM];

void IRAM_ATTR regi2c_analog_cali_reg_read(void)
{
    for (int i = 0; i < REGI2C_ANA_CALI_BYTE_NUM; i++) {
        reg_val[i] = regi2c_ctrl_read_reg(I2C_SAR_ADC, I2C_SAR_ADC_HOSTID, i);
    }
}

void IRAM_ATTR regi2c_analog_cali_reg_write(void)
{
    for (int i = 0; i < REGI2C_ANA_CALI_BYTE_NUM; i++) {
        regi2c_ctrl_write_reg(I2C_SAR_ADC, I2C_SAR_ADC_HOSTID, i, reg_val[i]);
    }
}
#endif   //#if ADC_CALI_PD_WORKAROUND

/**
 * REGI2C_SARADC reference count
 */
static int s_i2c_saradc_enable_cnt;

void regi2c_saradc_enable(void)
{
    regi2c_enter_critical();
    s_i2c_saradc_enable_cnt++;
    if (s_i2c_saradc_enable_cnt == 1) {
        regi2c_ctrl_ll_i2c_saradc_enable();
    }
    regi2c_exit_critical();
}

void regi2c_saradc_disable(void)
{
    regi2c_enter_critical();
    s_i2c_saradc_enable_cnt--;
    if (s_i2c_saradc_enable_cnt < 0){
        regi2c_exit_critical();
        ESP_HW_LOGE(TAG, "REGI2C_SARADC is already disabled");
    } else if (s_i2c_saradc_enable_cnt == 0) {
        regi2c_ctrl_ll_i2c_saradc_disable();
    }
    regi2c_exit_critical();

}
