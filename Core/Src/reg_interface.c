/*
 * Copyright (c) 2024-2026 Murata Electronics Oy.
 * SPDX-License-Identifier: BSD-3-Clause
 * See LICENSE at the repository root.
 */

#include <stdint.h>
#include "reg_interface.h"
#include "bit_names.h"
#include "main.h"
#include "firmware_extension.h"
#include "StrFunctions.h"

static reg_bank_t reg_bank;
static uint8_t spi_tx_frame[MAX_BYTES_PER_TRANSFER];
static uint8_t uart_tx_frame[MAX_BYTES_PER_TRANSFER];
static volatile init_stage_t init_stage = stage_no_rw;
static uint8_t is_sensor_data_register(uint8_t reg_addr);

/* Reset registers to initial values */
void init_reg_bank()
{
	for(int reg=0; reg < REG_BANK_SIZE; ++reg)
	{
		// Keep the reset indication valid while the bank is being cleared.
		if (reg == SENSOR_STATUS)
		{
			reg_bank.reg_data[SENSOR_STATUS] = DEFAULT_SENSOR_STATUS;
		}
		else
		{
			reg_bank.reg_data[reg] = 0;
		}
		reg_bank.reg_modified[reg] = 0;
		reg_bank.access[reg] = R;
		reg_bank.write_mask[reg] = 0xFFFFFFFF;
	}

	//Set 'Resetting' bit to one
	reg_bank.reg_data[SENSOR_STATUS] = DEFAULT_SENSOR_STATUS;

	//CONF register requires write access
	reg_bank.access[CONF] = RW;
	reg_bank.write_mask[CONF] = (1UL << DRO) | (1UL << RST);

	reg_bank.reg_data[FWVER_REG] = FWVER;

    // Make sure CRC byte is correct for uninitialized response frames after power-cycle
	spi_tx_frame[CRC_BYTE] = CRC8_LUT(spi_tx_frame);
	uart_tx_frame[CRC_BYTE] = CRC8_LUT(uart_tx_frame);
}

// Return pointer to register
int32_t* get_reg_ptr(uint16_t reg)
{
	return (&reg_bank.reg_data[reg]);
}

// Return pointer to register bank
reg_bank_t* get_reg_bank_ptr(void)
{
	return (&reg_bank);
}


// Write register value
void write_reg(uint16_t reg, int32_t data)
{
	reg_bank.reg_modified[reg] = 1;
	reg_bank.reg_data[reg] = data;
}


// Return register value
int32_t read_reg(uint16_t reg, uint8_t mark_as_read)
{
	if (mark_as_read)
	{
		reg_bank.reg_modified[reg] = 0;
	}
	return reg_bank.reg_data[reg];
}


// Get int32_t bit state
uint8_t get_bit_state(int32_t data, uint8_t bit_index)
{
	return (data >> bit_index) & 0x1;
}


// Set int32_t bit state
int32_t set_bit_state(int32_t data, uint8_t bit_index, uint8_t state)
{
	switch(state)
	{
		case 0:
			return (data & ~(1UL << bit_index));
			break;
		case 1:
			return (data | (1UL << bit_index));
			break;
		default:
			return data;
			break;
	}
}

// Parse UART/SPI frame
void parse_rx_frame(uint8_t* frame, uint8_t interface)
{
	uint8_t cmd_type = frame[0] >> 7;
	uint8_t addr = frame[0] & 0x3f;
	int32_t data = (((int32_t)frame[1]) << 24) | (((int32_t)frame[2]) << 16) | (((int32_t)frame[3]) << 8) | ((int32_t)frame[4]);
	uint8_t miso_status_bits = MISO_OK;

	// Check soft errors from the sensor status register
	if( (read_reg(SENSOR_STATUS, 1) & SOFT_ERROR_MASK) != 0)
	{
		switch(addr)
		{
		case ACC_X:
			miso_status_bits = (get_bit_state(reg_bank.reg_data[SENSOR_STATUS], SAX) == 1) ? MISO_SOFT_ERROR : miso_status_bits;
			break;
		case ACC_Y:
			miso_status_bits = (get_bit_state(reg_bank.reg_data[SENSOR_STATUS], SAY) == 1) ? MISO_SOFT_ERROR : miso_status_bits;
			break;
		case ACC_Z:
			miso_status_bits = (get_bit_state(reg_bank.reg_data[SENSOR_STATUS], SAZ) == 1) ? MISO_SOFT_ERROR : miso_status_bits;
			break;
		case RATE_X:
			miso_status_bits = (get_bit_state(reg_bank.reg_data[SENSOR_STATUS], SGX) == 1) ? MISO_SOFT_ERROR : miso_status_bits;
			break;
		case RATE_Y:
			miso_status_bits = (get_bit_state(reg_bank.reg_data[SENSOR_STATUS], SGY) == 1) ? MISO_SOFT_ERROR : miso_status_bits;
			break;
		case RATE_Z:
			miso_status_bits = (get_bit_state(reg_bank.reg_data[SENSOR_STATUS], SGZ) == 1) ? MISO_SOFT_ERROR : miso_status_bits;
			break;
		default:
			break;
		}

	}

	// Check hard errors from the sensor status register
	if( (read_reg(SENSOR_STATUS, 1) & HARD_ERROR_MASK) != 0)
	{
		switch(addr)
		{
		case ACC_X:
			miso_status_bits = (get_bit_state(reg_bank.reg_data[SENSOR_STATUS], HAX) == 1) ? MISO_HARD_ERROR : miso_status_bits;
			break;
		case ACC_Y:
			miso_status_bits = (get_bit_state(reg_bank.reg_data[SENSOR_STATUS], HAY) == 1) ? MISO_HARD_ERROR : miso_status_bits;
			break;
		case ACC_Z:
			miso_status_bits = (get_bit_state(reg_bank.reg_data[SENSOR_STATUS], HAZ) == 1) ? MISO_HARD_ERROR : miso_status_bits;
			break;
		case RATE_X:
			miso_status_bits = (get_bit_state(reg_bank.reg_data[SENSOR_STATUS], HGX) == 1) ? MISO_HARD_ERROR : miso_status_bits;
			break;
		case RATE_Y:
			miso_status_bits = (get_bit_state(reg_bank.reg_data[SENSOR_STATUS], HGY) == 1) ? MISO_HARD_ERROR : miso_status_bits;
			break;
		case RATE_Z:
			miso_status_bits = (get_bit_state(reg_bank.reg_data[SENSOR_STATUS], HGZ) == 1) ? MISO_HARD_ERROR : miso_status_bits;
			break;
		default:
			break;
		}
	}

	// Sensor init failure will manually set CMS bit. We need to indicate a MISO Hard error also in that case
	if (get_bit_state(reg_bank.reg_data[SENSOR_STATUS], CMS))
	{
		miso_status_bits = MISO_HARD_ERROR;
	}

	// Report ASIC response errors for the published status interval.
	if (get_bit_state(reg_bank.reg_data[SENSOR_STATUS], ACRC) ||
		get_bit_state(reg_bank.reg_data[SENSOR_STATUS], ACE))
	{
		miso_status_bits = MISO_HARD_ERROR;
	}

	// Check CRC from the request frame
	if (frame[CRC_BYTE] != CRC8_LUT(frame))
	{
		miso_status_bits = MISO_INVALID_REQ;
	}

	// Write command
	if (cmd_type == RW)
	{
        // Ignore writes in cases: CRC wrong or read-only
		if (miso_status_bits == MISO_INVALID_REQ || reg_bank.access[addr] != RW
			|| init_stage != stage_normal)
        {
        	miso_status_bits = MISO_INVALID_REQ;
        	data = 0;
        }
        else
        {
			if (addr == CONF && get_bit_state(data, RST) == 1)
			{
				// Expose reset status immediately, before process_tasks() handles CONF.
				reg_bank.reg_data[SENSOR_STATUS] = DEFAULT_SENSOR_STATUS;
			}

			if (!firmware_extension_intercept_write(addr, data))
			{
				// Reset write-enabled bits from existing register content.
				int32_t old_data = reg_bank.reg_data[addr] & (~(reg_bank.write_mask[addr]));

				// Reset read-only bits from incoming register content.
				int32_t new_data = data & reg_bank.write_mask[addr];

				// Combine new and old data.
				new_data = new_data | old_data;

				write_reg(addr, new_data);
				data = new_data;
			}
        }
	}
	// Read command
	else if(cmd_type == R)
	{
		data = read_reg(addr, 0); // Don't mark as read so that task_processor catches modifications

		if (init_stage == stage_no_rw)
		{
			data = 0;
		}

		if (firmware_extension_should_clear_data_ready(
				addr, is_sensor_data_register(addr)))
		{
			main_set_data_ready_pin_state(0);
		}
	}

	if (interface == SPI_INTERFACE)
	{
		spi_tx_frame[4] = (uint8_t)data;
		spi_tx_frame[3] = (uint8_t)(data >> 8);
		spi_tx_frame[2] = (uint8_t)(data >> 16);
		spi_tx_frame[1] = (uint8_t)(data >> 24);
		spi_tx_frame[0] = (miso_status_bits << 6) | addr;
		spi_tx_frame[CRC_BYTE] = CRC8_LUT(spi_tx_frame);
	}
	else
	{
		uart_tx_frame[4] = (uint8_t)data;
		uart_tx_frame[3] = (uint8_t)(data >> 8);
		uart_tx_frame[2] = (uint8_t)(data >> 16);
		uart_tx_frame[1] = (uint8_t)(data >> 24);
		uart_tx_frame[0] = (miso_status_bits << 6) | addr;
		uart_tx_frame[CRC_BYTE] = CRC8_LUT(uart_tx_frame);
	}
}

int8_t data_reg_index(uint8_t reg_addr)
{
	switch (reg_addr)
	{
	case ACC_X: return 0;
	case ACC_Y: return 1;
	case ACC_Z: return 2;
	case RATE_X: return 3;
	case RATE_Y: return 4;
	case RATE_Z: return 5;
	case TEMP: return 6;
	default: return -1;
	}
}

uint8_t data_reg_at(uint8_t index)
{
	static const uint8_t data_registers[DATA_REG_COUNT] = {
		ACC_X, ACC_Y, ACC_Z, RATE_X, RATE_Y, RATE_Z, TEMP
	};

	return data_registers[index];
}

void set_init_stage(init_stage_t stage)
{
	init_stage = stage;
}

init_stage_t get_init_stage(void)
{
	return init_stage;
}

// Return pointer to TX frame
uint8_t * get_tx_frame_ptr(uint8_t interface)
{
	if(interface == SPI_INTERFACE)
	{
		return spi_tx_frame;
	}
	else
	{
		return uart_tx_frame;
	}
}

// Convert floating point to fixed point
int32_t float_to_fixed(float value, uint32_t fractional_bits)
{
	return (int32_t)(value * (1UL << fractional_bits));
}

// Convert fixed point to floating point
float fixed_to_float(int32_t value, uint32_t fractional_bits)
{
	return ((float)value / (float)(1UL << fractional_bits));
}

// Return number of bytes in SPI frame
uint8_t get_transfer_count()
{
	return MAX_BYTES_PER_TRANSFER;
}

/*
 * Calculate CRC8 using look up table
 *
 * Coefficients generated from: http://www.sunshine2k.de/coding/javascript/crc/crc_js.html
 *
 * Example:
 * Polynomial 0x2F, Initial value: 0xFF, Final Xor: 0x00
 * Data: 000D68000003 -> CRC should be 0x8D
*/
uint8_t CRC8_LUT(volatile uint8_t *byte)
{
	static const uint8_t crc_table[] =
	{
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
			0xD8, 0xF7, 0x86, 0xA9, 0x64, 0x4B, 0x3A, 0x15, 0x8F, 0xA0, 0xD1, 0xFE, 0x33, 0x1C, 0x6D, 0x42
	};
	uint8_t CRC1 = 0xFF;

	CRC1 = crc_table[CRC1 ^ 0x00];
	CRC1 = crc_table[CRC1 ^ byte[0]];
	CRC1 = crc_table[CRC1 ^ byte[1]];
	CRC1 = crc_table[CRC1 ^ byte[2]];
	CRC1 = crc_table[CRC1 ^ byte[3]];
	CRC1 = crc_table[CRC1 ^ byte[4]];

	return CRC1;
}

uint8_t is_sensor_data_register(uint8_t reg_addr)
{
	// Preserve the register-interface contract: reads from the complete
	// legacy data range clear DATA_READY, including empty registers.
	return reg_addr >= ACC_X && reg_addr <= 22;
}


