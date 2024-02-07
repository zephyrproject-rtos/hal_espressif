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
#include <unistd.h>
#include <sys/time.h>
#include "esp_random.h"
#include "utils/common.h"
#include "mbedtls/platform_util.h"

int os_get_time(struct os_time *t)
{
    struct timeval tv;
    int ret = gettimeofday(&tv, NULL);
    t->sec = (os_time_t) tv.tv_sec;
    t->usec = tv.tv_usec;
    return ret;
}

unsigned long os_random(void)
{
    return esp_random();
}

int os_get_random(unsigned char *buf, size_t len)
{
    esp_fill_random(buf, len);
    return 0;
}

void os_sleep(os_time_t sec, os_time_t usec)
{
    if (sec) {
        sleep(sec);
    }
    if (usec) {
        usleep(usec);
    }
}

char *z_strdup(const char *s) {
    size_t size = strlen(s) + 1;
    char *p = k_malloc(size);
    if (p) {
        memcpy(p, s, size);
    }
    return p;
}

int tolower(unsigned char ch) {
    if (ch >= 'A' && ch <= 'Z')
        ch = 'a' + (ch - 'A');

    return ch;
}

int z_strcasecmp(const char * str1, const char * str2)
{
    const unsigned char *u1 = (const u_char *)s1,
    const unsigned char *u2 = (const u_char *)s2;

    while (tolower(*u1) == tolower(*u2++)) {
        if (*u1++ == '\0') {
            return 0;
        }
    }

    return (tolower(*u1) - tolower(*--u2));
}

int z_strcasecmp(const char * str1, const char * str2, size_t num)
{
    const unsigned char *u1 = (const u_char *)s1,
    const unsigned char *u2 = (const u_char *)s2;
    const size_t ch_compared = 0;

    while ((ch_compared < num) && (tolower(*u1) == tolower(*u2++))) {
        if (*u1++ == '\0') {
            return 0;
        }
        ch_compared++;
    }

    return (tolower(*u1) - tolower(*--u2));
}

#ifdef CONFIG_CRYPTO_MBEDTLS
void forced_memzero(void *ptr, size_t len)
{
    mbedtls_platform_zeroize(ptr, len);
}
#endif
