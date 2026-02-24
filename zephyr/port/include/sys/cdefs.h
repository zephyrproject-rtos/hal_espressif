/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Compatibility wrapper for sys/cdefs.h
 *
 * Zephyr's minimal libc does not provide all BSD-style macros that ESP-IDF
 * code expects. This header provides fallback definitions when needed.
 */

#ifndef _ESP_SYS_CDEFS_H_
#define _ESP_SYS_CDEFS_H_

#include_next <sys/cdefs.h>

/*
 * __containerof macro - get pointer to containing structure from member pointer
 * Provided by picolibc but not by minimal libc
 */
#ifndef __containerof
#define __containerof(ptr, type, member) \
	((type *)((char *)(ptr) - offsetof(type, member)))
#endif

/*
 * __BEGIN_DECLS / __END_DECLS - C++ extern "C" wrappers
 * Provided by picolibc but not by minimal libc
 */
#ifndef __BEGIN_DECLS
#ifdef __cplusplus
#define __BEGIN_DECLS extern "C" {
#define __END_DECLS }
#else
#define __BEGIN_DECLS
#define __END_DECLS
#endif
#endif

#endif /* _ESP_SYS_CDEFS_H_ */
