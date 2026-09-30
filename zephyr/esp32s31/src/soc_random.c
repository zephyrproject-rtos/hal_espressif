/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdbool.h>
#include "soc_random.h"
#include <hal/rng_ll.h>

void soc_random_enable(void)
{
	rng_ll_enable();
}

void soc_random_disable(void)
{
	rng_ll_disable();
}
