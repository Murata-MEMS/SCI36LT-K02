/*
 * Copyright (c) 2024-2026 Murata Electronics Oy.
 * SPDX-License-Identifier: BSD-3-Clause
 * See LICENSE at the repository root.
 */

#ifndef INC_FIRMWARE_EXTENSION_H_
#define INC_FIRMWARE_EXTENSION_H_

#include <stdbool.h>
#include <stdint.h>

void firmware_extension_init(void);
void firmware_extension_process(void);
void firmware_extension_before_reset(void);

/* Called from communication receive callback context. Keep implementations bounded. */
bool firmware_extension_intercept_write(uint8_t address, int32_t value);

bool firmware_extension_should_clear_data_ready(uint8_t address, bool default_clear);
void firmware_extension_transform_sensor_data(int32_t *data, uint8_t count);

#endif /* INC_FIRMWARE_EXTENSION_H_ */
