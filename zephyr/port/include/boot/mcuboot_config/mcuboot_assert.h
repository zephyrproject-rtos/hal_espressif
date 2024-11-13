/*
 * Copyright (c) 2023 Espressif Systems (Shanghai) Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <stdlib.h>

#ifdef assert
#undef assert
#endif
#define assert(arg)                                                 \
    do {                                                            \
        if (!(arg)) {                                               \
            ets_printf("ASSERTION FAIL @ %s:%d in function %s\n", __FILE__, __LINE__, __func__); \
            abort();                                                \
        }                                                           \
    } while(0)
