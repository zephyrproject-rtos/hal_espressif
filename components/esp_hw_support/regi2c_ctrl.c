/*
 * SPDX-FileCopyrightText: 2020-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */


#include "esp_attr.h"
#include <stdint.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/__assert.h>
#include "esp_private/critical_section.h"
#include "hal/regi2c_ctrl.h"

static esp_os_spinlock_t regi2c_lock = ESP_OS_SPINLOCK_INIT;

uint8_t regi2c_ctrl_read_reg(uint8_t block, uint8_t host_id, uint8_t reg_add)
{
    REGI2C_CLOCK_ENABLE();
    int __DECLARE_REGI2C_ATOMIC_ENV __attribute__((unused));
    unsigned int key = irq_lock();
    uint8_t value = regi2c_impl_read(block, host_id, reg_add);
    irq_unlock(key);
    REGI2C_CLOCK_DISABLE();
    return value;
}

uint8_t regi2c_ctrl_read_reg_mask(uint8_t block, uint8_t host_id, uint8_t reg_add, uint8_t msb, uint8_t lsb)
{
    REGI2C_CLOCK_ENABLE();
    int __DECLARE_REGI2C_ATOMIC_ENV __attribute__((unused));
    unsigned int key = irq_lock();
    uint8_t value = regi2c_impl_read_mask(block, host_id, reg_add, msb, lsb);
    irq_unlock(key);
    REGI2C_CLOCK_DISABLE();
    return value;
}

void regi2c_ctrl_write_reg(uint8_t block, uint8_t host_id, uint8_t reg_add, uint8_t data)
{
    REGI2C_CLOCK_ENABLE();
    int __DECLARE_REGI2C_ATOMIC_ENV __attribute__((unused));
    unsigned int key = irq_lock();
    regi2c_impl_write(block, host_id, reg_add, data);
    irq_unlock(key);
    REGI2C_CLOCK_DISABLE();
}

void regi2c_ctrl_write_reg_mask(uint8_t block, uint8_t host_id, uint8_t reg_add, uint8_t msb, uint8_t lsb, uint8_t data)
{
    REGI2C_CLOCK_ENABLE();
    int __DECLARE_REGI2C_ATOMIC_ENV __attribute__((unused));
    unsigned int key = irq_lock();
    regi2c_impl_write_mask(block, host_id, reg_add, msb, lsb, data);
    irq_unlock(key);
    REGI2C_CLOCK_DISABLE();
}

void IRAM_ATTR regi2c_enter_critical(void)
{
    esp_os_enter_critical(&regi2c_lock);
}

void IRAM_ATTR regi2c_exit_critical(void)
{
    esp_os_exit_critical(&regi2c_lock);
}
