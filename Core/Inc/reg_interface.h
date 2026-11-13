/*
 * Copyright (c) 2024-2026 Murata Electronics Oy.
 * SPDX-License-Identifier: BSD-3-Clause
 * See LICENSE at the repository root.
 */

#ifndef INC_REG_INTERFACE_H_
#define INC_REG_INTERFACE_H_

#define REG_BANK_SIZE				64   // Number of registers
#define REGULAR_FRACTIONAL_BITS		11
#define DEFAULT_SENSOR_STATUS		0x12000
#define MAX_BYTES_PER_TRANSFER		6
#define CRC_BYTE 					MAX_BYTES_PER_TRANSFER-1
#define SAMPLE_TIM_FREQ				2000.0f

#define SPI_INTERFACE 		0
#define UART_INTERFACE 		1

// MISO status bits
#define MISO_OK 			0
#define MISO_SOFT_ERROR		1
#define MISO_HARD_ERROR		2
#define MISO_INVALID_REQ	3


// User register access types
#define R	0 // Read only
#define RW	1 // Read+write

// Register addresses
#define CONF 				0
#define ACC_X 				1
#define ACC_Y 				2
#define ACC_Z 				3
#define RATE_X 				4
#define RATE_Y 				5
#define RATE_Z 				6
#define TEMP 				13
#define SENSOR_STATUS		33
#define ID_REG_0			34
#define ID_REG_1			35
#define	ID_REG_2			36
#define	ID_REG_3			37
#define	ID_REG_4			38
#define	ID_REG_5			39
#define MCU_ID_REG_0        40
#define MCU_ID_REG_1        41
#define MCU_ID_REG_2        42
#define FWVER_REG      		43

// Ordered list of the surviving sensor data registers.
#define DATA_REG_COUNT		7

typedef struct
{
	int32_t reg_data[REG_BANK_SIZE];
	uint8_t reg_modified[REG_BANK_SIZE];
	uint8_t access[REG_BANK_SIZE];
	uint32_t write_mask[REG_BANK_SIZE];
} reg_bank_t;

typedef enum
{
	stage_no_rw,
	stage_no_w,
	stage_normal
} init_stage_t;


void init_reg_bank(void);
int32_t* get_reg_ptr(uint16_t reg);
reg_bank_t* get_reg_bank_ptr(void);
uint8_t get_transfer_count(void);
void write_reg(uint16_t reg, int32_t data);
int32_t read_reg(uint16_t reg, uint8_t mark_as_read);
uint8_t get_bit_state(int32_t data, uint8_t bit_index);
int32_t set_bit_state(int32_t data, uint8_t bit_index, uint8_t state);
int8_t data_reg_index(uint8_t reg_addr);
uint8_t data_reg_at(uint8_t index);
void parse_rx_frame(uint8_t* frame, uint8_t interface);
int32_t float_to_fixed(float value, uint32_t fractional_bits);
float fixed_to_float(int32_t value, uint32_t fractional_bits);
uint8_t* get_tx_frame_ptr(uint8_t interface);
uint8_t CRC8_LUT(volatile uint8_t *byte);
void set_init_stage(init_stage_t stage);
init_stage_t get_init_stage(void);

#endif /* INC_REG_INTERFACE_H_ */
