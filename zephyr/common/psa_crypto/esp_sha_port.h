/*
 * Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* Private to the ESP32 SHA PSA driver: engine availability. */

#ifndef ESP_SHA_PORT_H_
#define ESP_SHA_PORT_H_

/*
 * Turn the engine's clock on the first time it is needed and report whether
 * the engine can be used: 0 when it can, a negative errno when it cannot. A
 * clock_control_on() failure (or no `sha` node) is latched until reboot, with no
 * retry per operation, and the driver then declines every algorithm, so hashing
 * stays in software. A clock controller that is not ready yet (a hash early in
 * boot) is -EAGAIN and is not latched: the next operation tries again.
 */
int esp_sha_port_ready(void);

#endif /* ESP_SHA_PORT_H_ */
