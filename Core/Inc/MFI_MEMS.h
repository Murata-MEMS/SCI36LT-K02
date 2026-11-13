/*
 * Copyright (c) 2024-2026 Murata Electronics Oy.
 * SPDX-License-Identifier: BSD-3-Clause
 * See LICENSE at the repository root.
 */

#ifndef __MFI_MEMS_H
#define __MFI_MEMS_H

#include "stdbool.h"
#include "stdint.h"
#include "string.h"
#include "stdio.h"

// function register for SCI16LT
#define SCI16LT_SET_EN			0x0D68000001D3
#define SCI16LT_SW_RESET		0x0DA800000AC3
#define	SCI16LT_SET_EOI			0x0D680000038D

// Interpolated sensor outputs
#define	SCI16LT_ACC_X_READ		0x0108000000F6
#define	SCI16LT_ACC_Y_READ		0x014800000001
#define	SCI16LT_ACC_Z_READ		0x018800000037
#define	SCI16LT_GYRO_X_READ		0x0048000000AC
#define	SCI16LT_GYRO_Y_READ		0x00880000009A
#define	SCI16LT_GYRO_Z_READ		0x00C80000006D

// Decimated sensor outputs
#define SCI16LT_ACC_X2_READ		0x034800000074
#define SCI16LT_ACC_Y2_READ		0x038800000042
#define SCI16LT_ACC_Z2_READ		0x03C8000000B5
#define SCI16LT_GYRO_X2_READ	0x0288000000EF
#define SCI16LT_GYRO_Y2_READ	0x02C800000018
#define SCI16LT_GYRO_Z2_READ	0x030800000083

// Temperature output
#define	SCI16LT_TEMP_READ		0x0408000000B1

// Status registers
#define	SCI16LT_SUM_ST			0x05080000001C
#define SCI16LT_COM_ST       	0x0588000000DD
#define	SCI16LT_SUM_SAT_ST		0x0548000000EB
#define SCI16LT_RATE_COM_ST		0x05C80000002A
#define SCI16LT_RATE_X_ST		0x0608000000C4
#define SCI16LT_RATE_Y_ST		0x064800000033
#define SCI16LT_RATE_Z_ST		0x068800000005
#define SCI16LT_ACC_X_ST		0x06C8000000F2
#define SCI16LT_ACC_Y_ST		0x070800000069
#define SCI16LT_ACC_Z_ST		0x07480000009E
#define SCI16LT_SYNC_ACTIVE		0x0788000000A8
#define SCI16LT_STAT_INFO		0x07C80000005F

// Gyro filter registers
#define SCI16LT_SET_13_RATE		0x096800009205  // Set 13 Hz filter for rate (LPF2)
#define SCI16LT_SET_30_RATE		0x096800004988  // Set 30 Hz filter for rate (LPF1)
#define SCI16LT_SET_68_RATE		0x096800000016  // Set 68 Hz filter for rate (LPF0)
#define SCI16LT_SET_235_RATE	0x096800016DAE  // Set 235 Hz filter for rate (LPF5)
#define SCI16LT_SET_280_RATE	0x09680000DB9B  // Set 280 Hz filter for rate (LPF3)
#define SCI16LT_READ_RATE_F 	0x0948000000FA  // Read filter for rate

// Acc filter registers
#define SCI16LT_SET_13_ACC   	0x09A800009233  // Set 13 Hz filter for acceleration (LPF2)
#define SCI16LT_SET_30_ACC   	0x09A8000049BE  // Set 30 Hz filter for acceleration (LPF1)
#define SCI16LT_SET_68_ACC   	0x09A800000020  // Set 68 Hz filter for acceleration (LPF0)
#define SCI16LT_SET_235_ACC		0x09A800016D98	// Set 235 Hz filter for acceleration (LPF5)
#define SCI16LT_SET_280_ACC  	0x09A80000DBAD  // Set 280 Hz filter for acceleration (LPF3)
#define SCI16LT_READ_ACC_F		0x0988000000CC  // Read filter for acceleration

// Range registers
#define SCI16LT_SET_300DPS		0x0A28002400A6	// Set +/- 300 dps range and decimation off
#define SCI16LT_SET_300DPS_4X	0x0A28002492B5	// Set +/- 300 dps range and 4 x decimation
#define SCI16LT_SET_300DPS_8X	0x0A280012DB6C	// Set +/- 300 dps range and 8 x decimation (DYN1, DEC4)
#define SCI16LT_SET_80MPS2		0x0A6800120016	// Set +/- 80 m/s^2 range and decimation off
#define SCI16LT_SET_80MPS2_4X	0x0A6800129205	// Set +/- 80 m/s^2 range and 4 x decimation
#define SCI16LT_SET_80MPS2_8X	0x0A680012DB9B	// Set +/- 80 m/s^2s range and 8 x decimation

// Write/Read CTRL_ST
#define SCI16LT_WRITE_CTRL_ST	0x0D28000006E9
#define SCI16LT_READ_CTRL_ST	0x0D08000000E7

// Serial number registers
#define SCI16LT_ASIC_ID			0x0EC800000009
#define SCI16LT_COMP_ID			0x0F0800000092
#define SCI16LT_SERIAL_1		0x0F4800000065
#define SCI16LT_SERIAL_2		0x0F8800000053
#define SCI16LT_SERIAL_3		0x0FC8000000A4

// Summary
#define SCI16LT_SUM_CMN			(1 << 7)
#define SCI16LT_SUM_RATE_X		(1 << 6)
#define SCI16LT_SUM_RATE_Y		(1 << 5)
#define SCI16LT_SUM_RATE_Z		(1 << 4)
#define SCI16LT_SUM_ACC_X		(1 << 3)
#define SCI16LT_SUM_ACC_Y		(1 << 2)
#define SCI16LT_SUM_ACC_Z		(1 << 1)
#define SCI16LT_SUM_INIT_RDY	(1 << 0)

// Saturation
#define SCI16LT_SAT_RATE_X		(1 << 14)
#define SCI16LT_SAT_RATE_Y		(1 << 13)
#define SCI16LT_SAT_RATE_Z		(1 << 12)
#define SCI16LT_SAT_ACC_X		(1 << 11)
#define SCI16LT_SAT_ACC_Y		(1 << 10)
#define SCI16LT_SAT_ACC_Z		(1 << 9)

// Common
#define SCI16LT_MCLK			(1 << 10)
#define SCI16LT_DUAL_CLOCK		(1 << 9)
#define SCI16LT_DSP_OK			(1 << 8)
#define SCI16LT_SVM				(1 << 7)
#define SCI16LT_HV_CP			(1 << 6)
#define SCI16LT_SUPPLY			(1 << 5)
#define SCI16LT_TEMP			(1 << 4)
#define SCI16LT_NMODE			(1 << 3)
#define SCI16LT_NVM_STS			(1 << 2)
#define SCI16LT_CMN_STS			(1 << 1)
#define SCI16LT_CMN_STS_RDY		(1 << 0)

// Sensitivities
#define	SENS_ACC			3200.0
#define SENS_GYRO			1060.0
#define	SENS_TEMP			100.0

// Low-pass filter setting
typedef enum {
    LPF_13_Hz = 13,
    LPF_30_Hz = 30,
    LPF_68_Hz = 68
} lpf_frequency_t;

struct sensor_Array
{
	float Gyro_X;
	float Gyro_Y;
	float Gyro_Z;

	float Acc_X;
	float Acc_Y;
	float Acc_Z;

	float Temp;
};

struct sensor_Status
{
	uint32_t stat_sum;
	uint32_t stat_sum_sat;
	uint32_t stat_sum_com;
	uint32_t stat_rate_com;
	uint32_t stat_rate_x;
	uint32_t stat_rate_y;
	uint32_t stat_rate_z;
	uint32_t stat_acc_x;
	uint32_t stat_acc_y;
	uint32_t stat_acc_z;
	uint32_t stat_sync_active;
	uint32_t stat_info;
};

struct sensor_LSB32
{
	int32_t Gyro_X;
	int32_t Gyro_Y;
	int32_t Gyro_Z;

	int32_t Acc_X;
	int32_t Acc_Y;
	int32_t Acc_Z;

	int32_t Temp;

	uint32_t Status;
	uint32_t Saturation;

	bool data_valid;       // All nine responses passed CRC and CE checks.
	bool status_valid;     // Status response passed CRC and CE checks.
	bool saturation_valid; // Saturation response passed CRC and CE checks.

};

struct component_flag
{
	uint16_t Soft_Gyro_X;
	uint16_t Soft_Gyro_Y;
	uint16_t Soft_Gyro_Z;
	uint16_t Hard_Gyro_X;
	uint16_t Hard_Gyro_Y;
	uint16_t Hard_Gyro_Z;
	uint16_t Soft_Acc_X;
	uint16_t Soft_Acc_Y;
	uint16_t Soft_Acc_Z;
	uint16_t Hard_Acc_X;
	uint16_t Hard_Acc_Y;
	uint16_t Hard_Acc_Z;
	uint16_t common;
	uint16_t Hard_Asic_Crc;
	uint16_t Hard_Asic_Command;
};

#endif
