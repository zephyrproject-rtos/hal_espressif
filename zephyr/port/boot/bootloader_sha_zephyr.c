/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* picolibc does not declare bzero(). The HAL SHA driver uses it. */
#include <string.h>

#define bzero(ptr, len) memset((ptr), 0, (len))

#include "../../../components/bootloader_support/src/bootloader_sha.c"
