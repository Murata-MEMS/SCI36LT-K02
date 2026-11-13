/*
 * Copyright (c) 2024-2026 Murata Electronics Oy.
 * SPDX-License-Identifier: BSD-3-Clause
 * See LICENSE at the repository root.
 */

#include <stdint.h>
#include "task_processor.h"
#include "reg_interface.h"
#include "bit_names.h"
#include "main.h"
#include "firmware_extension.h"
#include "MFI_MEMS.h"
#include "StrFunctions.h"
#include "AsicFunctions.h"

static volatile uint16_t averaging_factor = DEFAULT_AVG_FACTOR;
extern TIM_HandleTypeDef htim2;
extern struct component_flag status_flag;

static int find_updated_register(void);
static void prepare_for_init(void);
static void init_asic(void);

void process_tasks(void)
{
	int addr = find_updated_register();

	if (addr == CONF)
	{
		if (get_bit_state(read_reg(CONF, 1), RST) == 1)
		{
			// Stop timer interrupts
			HAL_TIM_Base_Stop_IT(&htim2);
			set_init_stage(stage_no_rw);
			firmware_extension_before_reset();

			// Initialize register bank
			init_reg_bank();
			memset(&status_flag, 0, sizeof(status_flag));

			averaging_factor = DEFAULT_AVG_FACTOR;

			// Initialize ASIC
			init_asic();

			// Clear 'Resetting' bit
			*get_reg_ptr(SENSOR_STATUS) = set_bit_state(read_reg(SENSOR_STATUS, 1), RES, 0);

			// Set data ready pin to zero
			main_set_data_ready_pin_state(0);
		}

		if (get_bit_state(read_reg(CONF, 1), DRO) == 1)
		{
			if (averaging_factor != AVG_FACTOR_200HZ)
			{
				averaging_factor = AVG_FACTOR_200HZ;

				// Initialize ASIC
				init_asic();
			}
		}

		if (get_bit_state(read_reg(CONF, 1), DRO) == 0)
		{
			if (averaging_factor != AVG_FACTOR_100HZ)
			{
				averaging_factor = AVG_FACTOR_100HZ;

				// Initialize ASIC
				init_asic();
			}
		}
	}
}

int find_updated_register(void)
{
	uint16_t modified_register;
	uint8_t register_mod_detected = 0;
	reg_bank_t * reg_bank_ptr;

	reg_bank_ptr = get_reg_bank_ptr();

	for (int i = 0; i < REG_BANK_SIZE; i++)
	{
		//this is a register user can write
		if (reg_bank_ptr->access[i] == RW)
		{
			//this register has been modified by user
			if (reg_bank_ptr->reg_modified[i])
			{
				modified_register = i;
				register_mod_detected = 1;
				break; //Process one register at  time
			}
		}
	}
	if (register_mod_detected == 0)
	{
		return -1;
	}

	return (int) modified_register;
}

static void prepare_for_init(void)
{
	reg_bank_t *register_bank = get_reg_bank_ptr();
	HAL_TIM_Base_Stop_IT(&htim2);

	// Clear stale data readiness before zeroing the output registers.
	main_set_data_ready_pin_state(0);

	set_init_stage(stage_no_w);

	// Keep published data invalid and clear latched ASIC status during reinit.
	for (uint8_t reg = ACC_X; reg <= RATE_Z; ++reg)
	{
		register_bank->reg_data[reg] = 0;
	}
	register_bank->reg_data[TEMP] = 0;
}

static void init_asic(void)
{
	prepare_for_init();

    // Set low-pass filter based on ODR
    lpf_frequency_t lpf = LPF_30_Hz;

	if (averaging_factor == AVG_FACTOR_200HZ)
	{
		lpf = LPF_68_Hz;
	}

	//Initialize ASIC
	int init_error = asic_initialize(lpf);

	if (init_error)
	{
		*get_reg_ptr(SENSOR_STATUS) = set_bit_state(read_reg(SENSOR_STATUS, 1), CMS, 1);

		main_ASIC_GPIO_SPI_disable();

		// Stop execution of main loop context
		// Don't use Error_Handler() to keep user interface alive
		while (1)
		{
		}
	}

	// Start timer interrupt
	HAL_TIM_Base_Start_IT(&htim2);

	set_init_stage(stage_normal);
}

uint16_t get_averaging_factor(void)
{
	return averaging_factor;
}
