/*
 * Copyright (c) 2024-2026 Murata Electronics Oy.
 * SPDX-License-Identifier: BSD-3-Clause
 * See LICENSE at the repository root.
 */

#include "MFI_MEMS.h"
#include "StrFunctions.h"
#include "stm32l4xx_hal.h"
#include "reg_interface.h"
#include "bit_names.h"
#include "AsicFunctions.h"
#include "main.h"

#define ARRAY_COUNT(array) (sizeof(array) / sizeof((array)[0]))
#define ASIC_PIPELINE_FLUSH_FRAME SCI16LT_SUM_ST
#define ASIC_MAX_INIT_ATTEMPTS 5
#define ASIC_STAT_SUM_MAX_POLLS 15

static void read_all_statuses(struct sensor_Status *status);
static void clear_all_statuses();
static int32_t check_all_statuses();
static void configure_asic(lpf_frequency_t lpf_setting);
static int32_t check_all_configuration(lpf_frequency_t lpf_setting);
static uint64_t asic_spi_transfer(uint64_t tx_frame);
static asic_response48_t asic_transfer_frame(uint64_t tx_frame);
static void asic_transfer_frames(const uint64_t *tx_frames, int tx_frame_count, asic_response48_t *responses);
static void asic_latch_response_errors(const asic_response48_t *response);
static uint8_t asic_crc_fast(uint64_t frame);
static uint64_t asic_pack_frame(uint16_t target_address, uint8_t write, uint8_t response_48bit,
		uint8_t address_extension, uint32_t data);
static uint64_t asic_read_frame(uint16_t register_address);
static uint64_t asic_write_frame(uint16_t register_address, uint32_t data);
static asic_response48_t asic_parse_response(uint64_t frame);
static int32_t sign_extension(int32_t, uint16_t);
static int32_t check_stat_com_pre_init(void);
static int32_t check_stat_sum_pre_init(void);
static int32_t asic_init_cycle(lpf_frequency_t sensor_lpf);

static int32_t asic_init_cycle(lpf_frequency_t sensor_lpf)
{
	// Wait for NVM read and SPI start-up.
	HAL_Delay(32);

	configure_asic(sensor_lpf);

	// Enable the sensor after its configuration has been written.
	asic_spi_transfer(SCI16LT_SET_EN);
	HAL_Delay(70);

	clear_all_statuses();
	if (check_stat_com_pre_init())
	{
		return 1;
	}

	for (int poll = 0; poll < ASIC_STAT_SUM_MAX_POLLS; poll++)
	{
		HAL_Delay(10);
		clear_all_statuses();
		if (!check_stat_sum_pre_init())
		{
			break;
		}

		if (poll == ASIC_STAT_SUM_MAX_POLLS - 1)
		{
			return 1;
		}
	}

	asic_spi_transfer(SCI16LT_SET_EOI);
	HAL_Delay(3);
	clear_all_statuses();

	if (check_all_statuses() || check_all_configuration(sensor_lpf))
	{
		return 1;
	}

	return 0;
}

int asic_initialize(lpf_frequency_t sensor_lpf)
{
	asic_spi_transfer(SCI16LT_SW_RESET);
	HAL_Delay(42);

	for (int attempt = 0; attempt <= ASIC_MAX_INIT_ATTEMPTS; attempt++)
	{
		if (!asic_init_cycle(sensor_lpf))
		{
			goto initialization_succeeded;
		}

		if (attempt < ASIC_MAX_INIT_ATTEMPTS)
		{
			asic_spi_transfer(SCI16LT_SW_RESET);
		}
	}

	return 1;

initialization_succeeded:

	// Settling time depends on low-pass filter setting
	switch (sensor_lpf)
	{
	case LPF_13_Hz:
		HAL_Delay(200);
		break;

	case LPF_30_Hz:
		HAL_Delay(40);
		break;

	case LPF_68_Hz:
		HAL_Delay(20);
		break;
	}

	get_asic_id();

	return 0;
}

static void read_all_statuses(struct sensor_Status *status)
{
	const uint64_t frames[] = {
		SCI16LT_SUM_ST,
		SCI16LT_SUM_SAT_ST,
		SCI16LT_COM_ST,
		SCI16LT_RATE_COM_ST,
		SCI16LT_RATE_X_ST,
		SCI16LT_RATE_Y_ST,
		SCI16LT_RATE_Z_ST,
		SCI16LT_ACC_X_ST,
		SCI16LT_ACC_Y_ST,
		SCI16LT_ACC_Z_ST,
		SCI16LT_SYNC_ACTIVE,
		SCI16LT_STAT_INFO,
	};
	asic_response48_t responses[ARRAY_COUNT(frames)];

	asic_transfer_frames(frames, (int)ARRAY_COUNT(frames), responses);
	status->stat_sum = responses[0].data;
	status->stat_sum_sat = responses[1].data;
	status->stat_sum_com = responses[2].data;
	status->stat_rate_com = responses[3].data;
	status->stat_rate_x = responses[4].data;
	status->stat_rate_y = responses[5].data;
	status->stat_rate_z = responses[6].data;
	status->stat_acc_x = responses[7].data;
	status->stat_acc_y = responses[8].data;
	status->stat_acc_z = responses[9].data;
	status->stat_sync_active = responses[10].data;
	status->stat_info = responses[11].data;
}

static void clear_all_statuses()
{
	static struct sensor_Status status;

	read_all_statuses(&status);
}

static int32_t check_stat_sum_pre_init(void)
{
	const uint32_t expected = 0xFF7E;
	uint32_t stat_sum = asic_transfer_frame(SCI16LT_SUM_ST).data;

	return stat_sum != expected;
}

static int32_t check_stat_com_pre_init(void)
{
	const uint32_t expected = 0xFEF7;
	uint32_t stat_com = asic_transfer_frame(SCI16LT_COM_ST).data;

	return stat_com != expected;
}

static int32_t check_all_statuses()
{
	static const struct sensor_Status expected = {
			0xFFFF,
			0xFFFF,
			0xFFFF,
			0xFFFF,
			0xFFFF,
			0xFFFF,
			0xFFFF,
			0xFFFF,
			0xFFFF,
			0xFFFF,
			0x000,
			0x00,
	};
	struct sensor_Status status;

	read_all_statuses(&status);

	return !(status.stat_sum == expected.stat_sum
			&& status.stat_sum_sat == expected.stat_sum_sat
			&& status.stat_sum_com == expected.stat_sum_com
			&& status.stat_rate_com == expected.stat_rate_com
			&& status.stat_rate_x == expected.stat_rate_x
			&& status.stat_rate_y == expected.stat_rate_y
			&& status.stat_rate_z == expected.stat_rate_z
			&& status.stat_acc_x == expected.stat_acc_x
			&& status.stat_acc_y == expected.stat_acc_y
			&& status.stat_acc_z == expected.stat_acc_z
			&& status.stat_sync_active == expected.stat_sync_active
			&& status.stat_info == expected.stat_info);
}

static void configure_asic(lpf_frequency_t lpf_setting)
{
	uint64_t frames[3];
	int count = 0;

	switch (lpf_setting)
	{
	case LPF_13_Hz:
		frames[count++] = SCI16LT_SET_13_RATE;
		frames[count++] = SCI16LT_SET_13_ACC;
		break;

	case LPF_30_Hz:
		frames[count++] = SCI16LT_SET_30_RATE;
		frames[count++] = SCI16LT_SET_30_ACC;
		break;

	case LPF_68_Hz:
		frames[count++] = SCI16LT_SET_68_RATE;
		frames[count++] = SCI16LT_SET_68_ACC;
		break;
	}

	frames[count++] = SCI16LT_WRITE_CTRL_ST;
	asic_transfer_frames(frames, count, NULL);
}

static int32_t check_all_configuration(lpf_frequency_t lpf_setting)
{
	uint32_t rate_filter;
	uint32_t acc_filter;
	uint32_t ctrl_st;
	const uint64_t frames[] = {
		SCI16LT_READ_RATE_F,
		SCI16LT_READ_ACC_F,
		SCI16LT_READ_CTRL_ST,
	};
	asic_response48_t responses[ARRAY_COUNT(frames)];

	asic_transfer_frames(frames, (int)ARRAY_COUNT(frames), responses);
	rate_filter = responses[0].data;
	acc_filter = responses[1].data;
	ctrl_st = responses[2].data;

	switch (lpf_setting)
	{
	case LPF_13_Hz:
		if (rate_filter == ((SCI16LT_SET_13_RATE >> 8) & 0xFFFFF)
			&& acc_filter == ((SCI16LT_SET_13_ACC >> 8) & 0xFFFFF)
			&& ctrl_st == ((SCI16LT_WRITE_CTRL_ST >> 8) & 0xFFFFF)
			)
		{
			return 0;
		}
		break;

	case LPF_30_Hz:
		if (rate_filter == ((SCI16LT_SET_30_RATE >> 8) & 0xFFFFF)
			&& acc_filter == ((SCI16LT_SET_30_ACC >> 8) & 0xFFFFF)
			&& ctrl_st == ((SCI16LT_WRITE_CTRL_ST >> 8) & 0xFFFFF)
			)
		{
			return 0;
		}
		break;

	case LPF_68_Hz:
		if (rate_filter == ((SCI16LT_SET_68_RATE >> 8) & 0xFFFFF)
			&& acc_filter == ((SCI16LT_SET_68_ACC >> 8) & 0xFFFFF)
			&& ctrl_st == ((SCI16LT_WRITE_CTRL_ST >> 8) & 0xFFFFF)
			)
		{
			return 0;
		}
		break;
	}

	return 1;
}

static void asic_transfer_frames(const uint64_t *tx_frames, int tx_frame_count, asic_response48_t *responses)
{
	if (tx_frames == NULL || tx_frame_count <= 0)
	{
		return;
	}

	asic_spi_transfer(tx_frames[0]);
	for (int i = 1; i < tx_frame_count; i++)
	{
		asic_response48_t response = asic_parse_response(asic_spi_transfer(tx_frames[i]));
		asic_latch_response_errors(&response);
		if (responses != NULL)
		{
			responses[i - 1] = response;
		}
	}

	asic_response48_t response = asic_parse_response(asic_spi_transfer(ASIC_PIPELINE_FLUSH_FRAME));
	asic_latch_response_errors(&response);
	if (responses != NULL)
	{
		responses[tx_frame_count - 1] = response;
	}
}

static asic_response48_t asic_transfer_frame(uint64_t tx_value)
{
	asic_spi_transfer(tx_value);
	asic_response48_t response = asic_parse_response(asic_spi_transfer(ASIC_PIPELINE_FLUSH_FRAME));
	asic_latch_response_errors(&response);
	return response;
}

int asic_read_register(uint16_t register_address, asic_response48_t *response)
{
	asic_response48_t read_response = asic_transfer_frame(asic_read_frame(register_address));

	if (response != NULL)
	{
		*response = read_response;
	}

	return (read_response.crc_ok && !read_response.command_error) ? 0 : 1;
}

int asic_write_register(uint16_t register_address, uint32_t data, asic_response48_t *response)
{
	asic_response48_t write_response = asic_transfer_frame(asic_write_frame(register_address, data));

	if (response != NULL)
	{
		*response = write_response;
	}

	return (write_response.crc_ok && !write_response.command_error) ? 0 : 1;
}

static void asic_latch_response_errors(const asic_response48_t *response)
{
	if (!response->crc_ok)
	{
		main_set_asic_crc_error();
	}
	else if (response->command_error)
	{
		main_set_asic_command_error();
	}
}

static uint64_t asic_spi_transfer(uint64_t tx_value)
{
	uint16_t tx_frames[] = {
		(uint16_t)((tx_value & 0x0000FFFF00000000ULL) >> 32),
		(uint16_t)((tx_value & 0x00000000FFFF0000ULL) >> 16),
		(uint16_t)(tx_value & 0x000000000000FFFFULL),
	};
	uint16_t rx_frames[3];

	GPIOD->BRR = 0x0004; // ASIC_CSB_Pin LOW
	for(int i = 0;i < 3;i++)
	{
		while ((SPI1->SR & SPI_SR_TXE) != SPI_SR_TXE){};	//Wait for SPI TX flag ready
		SPI1->DR = tx_frames[i];
		while ((SPI1->SR & SPI_SR_RXNE) !=SPI_SR_RXNE){};	//Wait while receive buffer is empty
		rx_frames[i] = SPI1->DR;
	}
	GPIOD->BSRR = 0x0004; // ASIC_CSB_Pin HIGH

	return ((uint64_t)rx_frames[0] << 32) |
			((uint64_t)rx_frames[1] << 16) |
			(uint64_t)rx_frames[2];
}

static uint8_t asic_crc_fast(uint64_t frame)
{
	static const uint8_t crc_lut[256] = {
		0x00, 0x2F, 0x5E, 0x71, 0xBC, 0x93, 0xE2, 0xCD, 0x57, 0x78, 0x09, 0x26, 0xEB, 0xC4, 0xB5, 0x9A,
		0xAE, 0x81, 0xF0, 0xDF, 0x12, 0x3D, 0x4C, 0x63, 0xF9, 0xD6, 0xA7, 0x88, 0x45, 0x6A, 0x1B, 0x34,
		0x73, 0x5C, 0x2D, 0x02, 0xCF, 0xE0, 0x91, 0xBE, 0x24, 0x0B, 0x7A, 0x55, 0x98, 0xB7, 0xC6, 0xE9,
		0xDD, 0xF2, 0x83, 0xAC, 0x61, 0x4E, 0x3F, 0x10, 0x8A, 0xA5, 0xD4, 0xFB, 0x36, 0x19, 0x68, 0x47,
		0xE6, 0xC9, 0xB8, 0x97, 0x5A, 0x75, 0x04, 0x2B, 0xB1, 0x9E, 0xEF, 0xC0, 0x0D, 0x22, 0x53, 0x7C,
		0x48, 0x67, 0x16, 0x39, 0xF4, 0xDB, 0xAA, 0x85, 0x1F, 0x30, 0x41, 0x6E, 0xA3, 0x8C, 0xFD, 0xD2,
		0x95, 0xBA, 0xCB, 0xE4, 0x29, 0x06, 0x77, 0x58, 0xC2, 0xED, 0x9C, 0xB3, 0x7E, 0x51, 0x20, 0x0F,
		0x3B, 0x14, 0x65, 0x4A, 0x87, 0xA8, 0xD9, 0xF6, 0x6C, 0x43, 0x32, 0x1D, 0xD0, 0xFF, 0x8E, 0xA1,
		0xE3, 0xCC, 0xBD, 0x92, 0x5F, 0x70, 0x01, 0x2E, 0xB4, 0x9B, 0xEA, 0xC5, 0x08, 0x27, 0x56, 0x79,
		0x4D, 0x62, 0x13, 0x3C, 0xF1, 0xDE, 0xAF, 0x80, 0x1A, 0x35, 0x44, 0x6B, 0xA6, 0x89, 0xF8, 0xD7,
		0x90, 0xBF, 0xCE, 0xE1, 0x2C, 0x03, 0x72, 0x5D, 0xC7, 0xE8, 0x99, 0xB6, 0x7B, 0x54, 0x25, 0x0A,
		0x3E, 0x11, 0x60, 0x4F, 0x82, 0xAD, 0xDC, 0xF3, 0x69, 0x46, 0x37, 0x18, 0xD5, 0xFA, 0x8B, 0xA4,
		0x05, 0x2A, 0x5B, 0x74, 0xB9, 0x96, 0xE7, 0xC8, 0x52, 0x7D, 0x0C, 0x23, 0xEE, 0xC1, 0xB0, 0x9F,
		0xAB, 0x84, 0xF5, 0xDA, 0x17, 0x38, 0x49, 0x66, 0xFC, 0xD3, 0xA2, 0x8D, 0x40, 0x6F, 0x1E, 0x31,
		0x76, 0x59, 0x28, 0x07, 0xCA, 0xE5, 0x94, 0xBB, 0x21, 0x0E, 0x7F, 0x50, 0x9D, 0xB2, 0xC3, 0xEC,
		0xD8, 0xF7, 0x86, 0xA9, 0x64, 0x4B, 0x3A, 0x15, 0x8F, 0xA0, 0xD1, 0xFE, 0x33, 0x1C, 0x6D, 0x42,
	};
	uint64_t data = frame & 0xFFFFFFFFFF00ULL;
	uint8_t crc = 0xFF;

	for (int i = 5; i >= 0; i--)
	{
		uint8_t data_byte = (uint8_t)((data >> (i * 8)) & 0xFFU);
		crc = crc_lut[crc] ^ data_byte;
	}

	return crc;
}

static uint64_t asic_pack_frame(uint16_t target_address, uint8_t write, uint8_t response_48bit,
		uint8_t address_extension, uint32_t data)
{
	uint64_t frame = ((uint64_t)(target_address & 0x03FFU) << 38) |
			((uint64_t)(write & 0x01U) << 37) |
			((uint64_t)(response_48bit & 0x01U) << 35) |
			((uint64_t)(address_extension & 0x7FU) << 28) |
			((uint64_t)(data & 0xFFFFFU) << 8);

	return frame | asic_crc_fast(frame);
}

static uint64_t asic_read_frame(uint16_t register_address)
{
	return asic_pack_frame(register_address, 0, 1, 0, 0);
}

static uint64_t asic_write_frame(uint16_t register_address, uint32_t data)
{
	return asic_pack_frame(register_address, 1, 1, 0, data);
}

static asic_response48_t asic_parse_response(uint64_t frame)
{
	asic_response48_t response = {0};

	response.raw_frame = frame & 0xFFFFFFFFFFFFULL;
	response.data_frame = (uint8_t)((frame >> 47) & 0x01U);
	response.source_address = (uint16_t)((frame >> 37) & 0x03FFU);
	response.internal_error = (uint8_t)((frame >> 36) & 0x01U);
	response.command_error = (uint8_t)((frame >> 35) & 0x01U);
	response.status = (uint8_t)((frame >> 33) & 0x03U);
	response.data_counter = (uint8_t)((frame >> 29) & 0x0FU);
	response.data = (uint32_t)((frame >> 8) & 0xFFFFFU);
	response.crc = (uint8_t)(frame & 0xFFU);
	response.crc_ok = (asic_crc_fast(frame) == response.crc);

	return response;
}

static int32_t sign_extension(int32_t data, uint16_t length)
{
	// sign extension
	if (data & (1 << (length-1))) {
		data |= ~((1 << length) - 1);
	}

	return data;
}

void get_6dof_dataset(struct sensor_LSB32 *sensor)
{
	enum {
		RESPONSE_ACC_X = 0,
		RESPONSE_ACC_Y,
		RESPONSE_ACC_Z,
		RESPONSE_GYRO_X,
		RESPONSE_GYRO_Y,
		RESPONSE_GYRO_Z,
		RESPONSE_TEMP,
		RESPONSE_STATUS,
		RESPONSE_SATURATION,
	};
	static const uint64_t frames[] = {
		SCI16LT_ACC_X_READ,
		SCI16LT_ACC_Y_READ,
		SCI16LT_ACC_Z_READ,
		SCI16LT_GYRO_X_READ,
		SCI16LT_GYRO_Y_READ,
		SCI16LT_GYRO_Z_READ,
		SCI16LT_TEMP_READ,
		SCI16LT_SUM_ST,
		SCI16LT_SUM_SAT_ST,
	};
	asic_response48_t responses[ARRAY_COUNT(frames)];

	asic_transfer_frames(frames, (int)ARRAY_COUNT(frames), responses);

	// Validate Status and Saturation independently of the complete dataset.
	sensor->data_valid = true;
	sensor->status_valid = responses[RESPONSE_STATUS].crc_ok && !responses[RESPONSE_STATUS].command_error;
	sensor->saturation_valid = responses[RESPONSE_SATURATION].crc_ok && !responses[RESPONSE_SATURATION].command_error;

	for (int i = 0; i < (int)ARRAY_COUNT(frames); i++)
	{
		if (!responses[i].crc_ok || responses[i].command_error)
		{
			sensor->data_valid = false;
		}
	}

	sensor->Acc_X = sign_extension(responses[RESPONSE_ACC_X].data, 20);
	sensor->Acc_Y = sign_extension(responses[RESPONSE_ACC_Y].data, 20);
	sensor->Acc_Z = sign_extension(responses[RESPONSE_ACC_Z].data, 20);
	sensor->Gyro_X = sign_extension(responses[RESPONSE_GYRO_X].data, 20);
	sensor->Gyro_Y = sign_extension(responses[RESPONSE_GYRO_Y].data, 20);
	sensor->Gyro_Z = sign_extension(responses[RESPONSE_GYRO_Z].data, 20);
	sensor->Temp = sign_extension(responses[RESPONSE_TEMP].data >> 4, 16);
	sensor->Status = responses[RESPONSE_STATUS].data;
	sensor->Saturation = responses[RESPONSE_SATURATION].data;
}


void get_asic_id()
{
	const uint64_t frames[] = {
		SCI16LT_SERIAL_1,
		SCI16LT_SERIAL_2,
		SCI16LT_SERIAL_3,
		SCI16LT_COMP_ID,
		SCI16LT_ASIC_ID,
	};
	asic_response48_t responses[ARRAY_COUNT(frames)];

	// Fetch SerialNr from ASIC to registers
	asic_transfer_frames(frames, (int)ARRAY_COUNT(frames), responses);
	write_reg(ID_REG_0, responses[0].data);
	write_reg(ID_REG_1, responses[1].data);
	write_reg(ID_REG_2, responses[2].data);
	write_reg(ID_REG_3, responses[3].data);
	write_reg(ID_REG_4, responses[4].data);

	// Fetch MCU ID to registers
	write_reg(MCU_ID_REG_0, HAL_GetUIDw0());
	write_reg(MCU_ID_REG_1, HAL_GetUIDw1());
	write_reg(MCU_ID_REG_2, HAL_GetUIDw2());
}

