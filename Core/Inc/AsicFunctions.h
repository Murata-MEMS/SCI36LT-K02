/*
 * Copyright (c) 2024-2026 Murata Electronics Oy.
 * SPDX-License-Identifier: BSD-3-Clause
 * See LICENSE at the repository root.
 */

#ifndef __AsicFunctions_H
#define __AsicFunctions_H

#include "stdbool.h"
#include "stdint.h"
#include "stm32l4xx_hal.h"
#include "MFI_MEMS.h"

typedef struct
{
	uint64_t raw_frame;
	uint8_t data_frame;
	uint16_t source_address;
	uint8_t internal_error;
	uint8_t command_error;
	uint8_t status;
	uint8_t data_counter;
	uint32_t data;
	uint8_t crc;
	bool crc_ok;
} asic_response48_t;

int asic_read_register(uint16_t register_address, asic_response48_t *response);
int asic_write_register(uint16_t register_address, uint32_t data, asic_response48_t *response);

int asic_initialize(lpf_frequency_t sensor_lpf);
void get_6dof_dataset(struct sensor_LSB32 *sensor);
void get_asic_id();

#endif
