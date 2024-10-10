/*
 * wpa_supplicant/hostapd / Internal implementation of OS specific functions
 * Copyright (c) 2005-2006, Jouni Malinen <j@w1.fi>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Alternatively, this software may be distributed under the terms of BSD
 * license.
 *
 * See README and COPYING for more details.
 *
 * This file is an example of operating system specific  wrapper functions.
 * This version implements many of the functions internally, so it can be used
 * to fill in missing functions from the target system C libraries.
 *
 * Some of the functions are using standard C library calls in order to keep
 * this file in working condition to allow the functions to be tested on a
 * Linux target. Please note that OS_NO_C_LIB_DEFINES needs to be defined for
 * this file to work correctly. Note that these implementations are only
 * examples and are not optimized for speed.
 */

#include "os.h"
#include <stdlib.h>
#include "esp_system.h"
#include "utils/common.h"
#include <zephyr/random/random.h>
#include "mbedtls/platform_util.h"

int os_get_time(struct os_time *t)
{
	if (t == NULL) {
		return -1;
	}

	int64_t now = k_uptime_ticks();
	t->sec = now / CONFIG_SYS_CLOCK_TICKS_PER_SEC;
	t->usec = k_ticks_to_us_floor64(now);

	return 0;
}

unsigned long os_random(void)
{
	return sys_rand32_get();
}

int os_get_random(unsigned char *buf, size_t len)
{
	sys_rand_get((void *)buf, len);
	return 0;
}

void os_sleep(os_time_t sec, os_time_t usec)
{
    if (sec) {
        k_sleep(K_SECONDS(sec));
    }
    if (usec) {
        k_sleep(K_USEC(usec));
    }
}

#ifdef CONFIG_CRYPTO_MBEDTLS
void forced_memzero(void *ptr, size_t len)
{
    mbedtls_platform_zeroize(ptr, len);
}
#endif
