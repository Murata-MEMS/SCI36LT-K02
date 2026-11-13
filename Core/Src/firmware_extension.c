/*
 * Copyright (c) 2024-2026 Murata Electronics Oy.
 * SPDX-License-Identifier: BSD-3-Clause
 * See LICENSE at the repository root.
 */

#include "firmware_extension.h"

#ifdef INTERNAL_TEST_BUILD
#include "internal_test.h"
#endif

void firmware_extension_init(void)
{
#ifdef INTERNAL_TEST_BUILD
	internal_test_init();
#endif
}

void firmware_extension_process(void)
{
#ifdef INTERNAL_TEST_BUILD
	internal_test_process();
#endif
}

void firmware_extension_before_reset(void)
{
#ifdef INTERNAL_TEST_BUILD
	internal_test_before_reset();
#endif
}

bool firmware_extension_intercept_write(uint8_t address, int32_t value)
{
#ifdef INTERNAL_TEST_BUILD
	return internal_test_intercept_write(address, value);
#else
	(void)address;
	(void)value;
	return false;
#endif
}

bool firmware_extension_should_clear_data_ready(uint8_t address, bool default_clear)
{
#ifdef INTERNAL_TEST_BUILD
	return internal_test_should_clear_data_ready(address, default_clear);
#else
	(void)address;
	return default_clear;
#endif
}

void firmware_extension_transform_sensor_data(int32_t *data, uint8_t count)
{
#ifdef INTERNAL_TEST_BUILD
	internal_test_transform_sensor_data(data, count);
#else
	(void)data;
	(void)count;
#endif
}
