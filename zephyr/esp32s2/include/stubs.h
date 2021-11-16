/*
 * Copyright (c) 2020 Espressif Systems (Shanghai) Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _STUBS_H_
#define _STUBS_H_

/* Required for C99 compilation (required for GCC-8.x version,               
 * where typeof is used instead of __typeof__)
 */
#ifndef typeof
#define typeof  __typeof__
#endif

/*
 * Used in Zephyr's soc.c during cache initialization.
 * This function prototype is declared here to avoid Zephyr's coding guidelines
 * checks violation (rule 21.2: "Should not use a reserved identifier")
 */
extern void abort(void);

#endif /* _STUBS_H_ */
