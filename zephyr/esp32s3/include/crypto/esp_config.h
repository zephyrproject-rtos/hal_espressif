/*
 * Copyright (c) 2020 Espressif Systems (Shanghai) Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

/**
 * \def MBEDTLS_PK_WRITE_C
 *
 * Enable the generic public (asymetric) key writer.
 * Module:  library/pkwrite.c
 * Caller:  library/x509write.c
 *
 * Requires: MBEDTLS_PK_C
 *
 * Uncomment to enable generic public key write functions.
 */
#define MBEDTLS_PK_WRITE_C
