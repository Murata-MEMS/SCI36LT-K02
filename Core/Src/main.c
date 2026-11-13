/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body.
  ******************************************************************************
  * @attention
  *
  * Murata contributions: Copyright (c) 2024-2026 Murata Electronics Oy.
  *
  * Portions Copyright (c) STMicroelectronics.
  * All rights reserved.
  *
  * ST-generated code and modifications to it are licensed under the Ultimate
  * Liberty software license agreement SLA0044. See
  * third_party_licenses/ST_SLA0044.txt and THIRD_PARTY_NOTICES.md at the
  * repository root.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdbool.h"
#include "stdint.h"
#include "string.h"
#include "stdio.h"
#include "math.h"
#include "MFI_MEMS.h"
#include "AsicFunctions.h"
#include "StrFunctions.h"
#include "reg_interface.h"
#include "task_processor.h"
#include "firmware_extension.h"
#include "bit_names.h"
#if DEBUG_PRINTS_ON_USB
#include "usbd_cdc_if.h"
#endif


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define HOST_SPI_NSS_GPIO_PORT GPIOB
#define HOST_SPI_NSS_PIN GPIO_PIN_12

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
DMA_HandleTypeDef hdma_spi2_rx;
DMA_HandleTypeDef hdma_spi2_tx;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_rx;
DMA_HandleTypeDef hdma_usart3_tx;

/* USER CODE BEGIN PV */
volatile uint16_t sensor_average = 0;
static uint16_t sensor_valid_samples = 0;
volatile uint8_t averaging_complete = 0;
volatile bool tx_done_flag = true;
volatile uint32_t rx_dma_reset_counter = 0;
static uint8_t asic_spi_enabled = 1;
struct component_flag status_flag={0};
uint8_t spi_rx_data[MAX_BYTES_PER_TRANSFER];
uint8_t uart_rx_data[MAX_BYTES_PER_TRANSFER];
struct sensor_Array out = {0};
struct sensor_LSB32 sensor_raw = {0};
struct sensor_LSB32 sensor_ave = {0};
static volatile bool spi_recovery_pending = false;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

static void try_recover_host_spi(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();
#if DEBUG_PRINTS_ON_USB
  MX_USB_DEVICE_Init();
#endif
  MX_SPI2_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
	HAL_SPI_Init(&hspi1);
	HAL_SPI_Init(&hspi2);

	HAL_Delay(5);

	__HAL_SPI_ENABLE(&hspi1);
	__HAL_SPI_ENABLE(&hspi2);

	while(HAL_SPI_TransmitReceive_DMA(&hspi2, get_tx_frame_ptr(SPI_INTERFACE), spi_rx_data, get_transfer_count()) != HAL_OK);
	while(HAL_UART_Receive_DMA(&huart3, uart_rx_data, get_transfer_count()) != HAL_OK);

#if DEBUG_PRINTS_ON_USB
	HAL_Delay(400); // USB device needs to negotiate with PC, which takes some time
#endif

	// Initialize register bank
	init_reg_bank();
	firmware_extension_init();

	// This will initialize ASIC in process_tasks()
	write_reg(CONF, set_bit_state(0, RST, 1));

	// Allow the ASIC NVM to complete before the first SPI initialization.
	HAL_Delay(50);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

	while (1)
	{
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		if (spi_recovery_pending) {
			try_recover_host_spi();
		}
		process_tasks();
		firmware_extension_process();

		//Average
		if(averaging_complete)
		{
			averaging_complete = 0;

			// Continue processing only if reset has not been requested
			if (get_bit_state(read_reg(SENSOR_STATUS, 0), RES) == 0)
			{
				if (get_init_stage() == stage_normal)
				{
					// Update all data registers before setting DATA_READY.
					const uint8_t data_reg_modified[DATA_REG_COUNT] = {[0 ... DATA_REG_COUNT - 1] = 1};
					int32_t data_array[DATA_REG_COUNT] = {
							float_to_fixed(out.Acc_X, REGULAR_FRACTIONAL_BITS),
							float_to_fixed(out.Acc_Y, REGULAR_FRACTIONAL_BITS),
							float_to_fixed(out.Acc_Z, REGULAR_FRACTIONAL_BITS),
							float_to_fixed(out.Gyro_X, REGULAR_FRACTIONAL_BITS),
							float_to_fixed(out.Gyro_Y, REGULAR_FRACTIONAL_BITS),
							float_to_fixed(out.Gyro_Z, REGULAR_FRACTIONAL_BITS),
							float_to_fixed(out.Temp, REGULAR_FRACTIONAL_BITS)
					};
					firmware_extension_transform_sensor_data(data_array, DATA_REG_COUNT);

					reg_bank_t *register_bank = get_reg_bank_ptr();
					__disable_irq();
					for (uint8_t data_index = 0; data_index < DATA_REG_COUNT; ++data_index)
					{
						uint8_t reg = data_reg_at(data_index);
						register_bank->reg_data[reg] = data_array[data_index];
						register_bank->reg_modified[reg] = data_reg_modified[data_index];
					}

					//All data registers have been written, so set DATA_READY pin and bit to one
					main_set_data_ready_pin_state(1);
					__enable_irq();
				}

				// Update status register
				__disable_irq();
				write_reg(SENSOR_STATUS, (int32_t) (
						(((uint32_t)read_reg(SENSOR_STATUS, 0)) & (1UL << DRY)) |
						(((uint32_t)status_flag.common) << CMS) |
						(((uint32_t)status_flag.Hard_Asic_Crc) << ACRC) |
						(((uint32_t)status_flag.Hard_Asic_Command) << ACE) |
						(((uint32_t)status_flag.Hard_Gyro_X) << HGX) |
						(((uint32_t)status_flag.Soft_Gyro_X) << SGX) |
						(((uint32_t)status_flag.Hard_Gyro_Y) << HGY) |
						(((uint32_t)status_flag.Soft_Gyro_Y) << SGY) |
						(((uint32_t)status_flag.Hard_Gyro_Z) << HGZ) |
						(((uint32_t)status_flag.Soft_Gyro_Z) << SGZ) |
						(((uint32_t)status_flag.Hard_Acc_X) << HAX) |
						(((uint32_t)status_flag.Soft_Acc_X) << SAX) |
						(((uint32_t)status_flag.Hard_Acc_Y) << HAY) |
						(((uint32_t)status_flag.Soft_Acc_Y) << SAY) |
						(((uint32_t)status_flag.Hard_Acc_Z) << HAZ) |
						(((uint32_t)status_flag.Soft_Acc_Z) << SAZ)
						));
				// Clear published flags before an ISR can latch a new error that this reset would erase.
				status_flag.Soft_Gyro_X=0;
				status_flag.Soft_Gyro_Y=0;
				status_flag.Soft_Gyro_Z=0;
				status_flag.Soft_Acc_X=0;
				status_flag.Soft_Acc_Y=0;
				status_flag.Soft_Acc_Z=0;

				status_flag.Hard_Gyro_X = 0;
				status_flag.Hard_Gyro_Y = 0;
				status_flag.Hard_Gyro_Z = 0;
				status_flag.Hard_Acc_X = 0;
				status_flag.Hard_Acc_Y = 0;
				status_flag.Hard_Acc_Z = 0;

				status_flag.common = 0;
				status_flag.Hard_Asic_Crc = 0;
				status_flag.Hard_Asic_Command = 0;
				__enable_irq();

				debug_printf("%.6f\t%.6f\t%.6f\t%.3f\t%.3f\t%.3f\t%.1f\t%08X",
						out.Acc_X, out.Acc_Y, out.Acc_Z,
						out.Gyro_X, out.Gyro_Y, out.Gyro_Z,
						out.Temp,
						read_reg(SENSOR_STATUS, 0)
						);
			}
		}

		//Put CPU into sleep, but all peripherals continue to operate. Next sample timer interrupt will wake the CPU up
		HAL_SuspendTick();
		HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON,PWR_SLEEPENTRY_WFI);
		HAL_ResumeTick();

	} // while

  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_SLAVE;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_HARD_INPUT;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 99;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 399;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 460800;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DATA_READY_GPIO_Port, DATA_READY_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : SPI3_EXT_CS_Pin UART1_TX_Pin UART1_RX_Pin UART1_CK_Pin
                           QUAD_SPI_IO3_Pin QUAD_SPI_IO2_Pin */
  GPIO_InitStruct.Pin = SPI3_EXT_CS_Pin|UART1_TX_Pin|UART1_RX_Pin|UART1_CK_Pin
                          |QUAD_SPI_IO3_Pin|QUAD_SPI_IO2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI3_EXT_MOSI_Pin CFG_RST_Pin */
  GPIO_InitStruct.Pin = SPI3_EXT_MOSI_Pin|CFG_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI1_CS_Pin */
  GPIO_InitStruct.Pin = SPI1_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI1_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : CAN_TX_Pin CAN_RX_Pin QUAD_SPI_IO1_Pin QUAD_SPI_NCS_Pin
                           QUAD_SPI_CLK_Pin QUAD_SPI_IO0_Pin */
  GPIO_InitStruct.Pin = CAN_TX_Pin|CAN_RX_Pin|QUAD_SPI_IO1_Pin|QUAD_SPI_NCS_Pin
                          |QUAD_SPI_CLK_Pin|QUAD_SPI_IO0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : ASIC_LDO_EN_Pin */
  GPIO_InitStruct.Pin = ASIC_LDO_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(ASIC_LDO_EN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DATA_READY_Pin */
  GPIO_InitStruct.Pin = DATA_READY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DATA_READY_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void ISR_2kHz()
{

	check_uart_state();

	get_6dof_dataset(&sensor_raw);
	sensor_average++;

	if (sensor_raw.data_valid)
	{
		sensor_valid_samples++;
		sensor_ave.Acc_X += sensor_raw.Acc_X;
		sensor_ave.Acc_Y += sensor_raw.Acc_Y;
		sensor_ave.Acc_Z += sensor_raw.Acc_Z;

		sensor_ave.Gyro_X += sensor_raw.Gyro_X;
		sensor_ave.Gyro_Y += sensor_raw.Gyro_Y;
		sensor_ave.Gyro_Z += sensor_raw.Gyro_Z;

		sensor_ave.Temp += sensor_raw.Temp;
	}

	// Latch independently valid status words even when measurements are rejected.
	if (sensor_raw.saturation_valid)
	{
		if ((sensor_raw.Saturation & SCI16LT_SAT_RATE_X) == 0) status_flag.Soft_Gyro_X = 1;
		if ((sensor_raw.Saturation & SCI16LT_SAT_RATE_Y) == 0) status_flag.Soft_Gyro_Y = 1;
		if ((sensor_raw.Saturation & SCI16LT_SAT_RATE_Z) == 0) status_flag.Soft_Gyro_Z = 1;
		if ((sensor_raw.Saturation & SCI16LT_SAT_ACC_X) == 0) status_flag.Soft_Acc_X = 1;
		if ((sensor_raw.Saturation & SCI16LT_SAT_ACC_Y) == 0) status_flag.Soft_Acc_Y = 1;
		if ((sensor_raw.Saturation & SCI16LT_SAT_ACC_Z) == 0) status_flag.Soft_Acc_Z = 1;
	}

	if (sensor_raw.status_valid)
	{
		if ((sensor_raw.Status & SCI16LT_SUM_RATE_X) == 0) status_flag.Hard_Gyro_X = 1;
		if ((sensor_raw.Status & SCI16LT_SUM_RATE_Y) == 0) status_flag.Hard_Gyro_Y = 1;
		if ((sensor_raw.Status & SCI16LT_SUM_RATE_Z) == 0) status_flag.Hard_Gyro_Z = 1;
		if ((sensor_raw.Status & SCI16LT_SUM_ACC_X) == 0) status_flag.Hard_Acc_X = 1;
		if ((sensor_raw.Status & SCI16LT_SUM_ACC_Y) == 0) status_flag.Hard_Acc_Y = 1;
		if ((sensor_raw.Status & SCI16LT_SUM_ACC_Z) == 0) status_flag.Hard_Acc_Z = 1;

		if ((sensor_raw.Status & SCI16LT_SUM_CMN) == 0) status_flag.common = 1;
	}

	if(sensor_average >= get_averaging_factor())
	{
		out = (struct sensor_Array){0};
		if (sensor_valid_samples > 0)
		{
			out.Acc_X = ((float)sensor_ave.Acc_X/SENS_ACC/sensor_valid_samples);
			out.Acc_Y = ((float)sensor_ave.Acc_Y/SENS_ACC/sensor_valid_samples);
			out.Acc_Z = ((float)sensor_ave.Acc_Z/SENS_ACC/sensor_valid_samples);

			out.Gyro_X = (float)sensor_ave.Gyro_X/SENS_GYRO/sensor_valid_samples;
			out.Gyro_Y = (float)sensor_ave.Gyro_Y/SENS_GYRO/sensor_valid_samples;
			out.Gyro_Z = (float)sensor_ave.Gyro_Z/SENS_GYRO/sensor_valid_samples;

			out.Temp = (float)sensor_ave.Temp/SENS_TEMP/sensor_valid_samples;
		}

		memset(&sensor_ave, 0, sizeof(struct sensor_LSB32));
		sensor_average = 0;
		sensor_valid_samples = 0;
		averaging_complete = 1;
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	parse_rx_frame(uart_rx_data, UART_INTERFACE);
	DATA_TX(get_tx_frame_ptr(UART_INTERFACE), get_transfer_count());
	rx_dma_reset_counter = 0;
	while(HAL_UART_Receive_DMA(&huart3, uart_rx_data,  get_transfer_count()) != HAL_OK);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    tx_done_flag = true;
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi){
	// SPI1 ASIC transfers are polled directly; only SPI2 uses this DMA callback.
	if (hspi != &hspi2)
	{
		return;
	}

	// Parse received SPI frame and load TX data
	parse_rx_frame(spi_rx_data, SPI_INTERFACE);

	// Re-enable SPI DMA transfer. Recover from main context if rearming fails.
	if (HAL_SPI_TransmitReceive_DMA(&hspi2, get_tx_frame_ptr(SPI_INTERFACE), spi_rx_data, get_transfer_count()) != HAL_OK)
	{
		spi_recovery_pending = true;
	}
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
	if (hspi == &hspi2)
	{
		spi_recovery_pending = true;
	}
}

static void try_recover_host_spi(void)
{
	// The error callback has returned; wait for a clean SPI frame boundary.
	if (HAL_GPIO_ReadPin(HOST_SPI_NSS_GPIO_PORT, HOST_SPI_NSS_PIN) != GPIO_PIN_SET)
	{
		return;
	}

	spi_recovery_pending = false;
	if (HAL_SPI_Abort(&hspi2) != HAL_OK)
	{
		spi_recovery_pending = true;
		return;
	}

	// The OVR handler already disabled the DMA requests, so HAL_SPI_Abort()
	// may have skipped its RX FIFO drain. Flush it before starting a new frame.
	// A timeout leaves recovery pending; the next main-loop pass retries it.
	if (HAL_SPIEx_FlushRxFifo(&hspi2) != HAL_OK)
	{
		spi_recovery_pending = true;
		return;
	}

	// NSS may have gone active while the abort and flush were running.
	if (HAL_GPIO_ReadPin(HOST_SPI_NSS_GPIO_PORT, HOST_SPI_NSS_PIN) != GPIO_PIN_SET)
	{
		spi_recovery_pending = true;
		return;
	}

	if (HAL_SPI_TransmitReceive_DMA(&hspi2, get_tx_frame_ptr(SPI_INTERFACE), spi_rx_data, get_transfer_count()) != HAL_OK)
	{
		spi_recovery_pending = true;
		return;
	}
}

// Resets UART RX DMA transmission if it is frozen
void check_uart_state()
{
	static uint32_t bytes_received_prev = 0;
	uint16_t bytes_received =  huart3.hdmarx->Instance->CNDTR;
	if((bytes_received != get_transfer_count()) && (bytes_received == bytes_received_prev))
	{
		rx_dma_reset_counter++;
	}
	else
	{
		rx_dma_reset_counter = 0;
	}
	bytes_received_prev = bytes_received;

	if ( (rx_dma_reset_counter > UART_DMA_RESET_TIMEOUT) || ((huart3.hdmarx->Instance->CCR & 0x01) == 0) )
	{
		HAL_UART_DMAStop(&huart3);                          // STOP Uart
		MX_USART3_UART_Init();                              // INIT Uart
		tx_done_flag = true;                                // DMAStop may abort an active TX
		while(HAL_UART_Receive_DMA(&huart3, uart_rx_data,  get_transfer_count()) != HAL_OK);
	}
}

void main_set_data_ready_pin_state(uint8_t state)
{

	if (state)
	{
		HAL_GPIO_WritePin(DATA_READY_GPIO_Port, DATA_READY_Pin, GPIO_PIN_SET);
	}
	else
	{
		HAL_GPIO_WritePin(DATA_READY_GPIO_Port, DATA_READY_Pin, GPIO_PIN_RESET);
	}

	*get_reg_ptr(SENSOR_STATUS) = set_bit_state(read_reg(SENSOR_STATUS, 1), DRY, state);

}

void main_set_asic_crc_error(void)
{
	status_flag.Hard_Asic_Crc = 1;
}

void main_set_asic_command_error(void)
{
	status_flag.Hard_Asic_Command = 1;
}

void main_ASIC_GPIO_SPI_disable(void)
{
	// Disable SPI and release the ASIC interface pins after initialization failure.
	__HAL_SPI_DISABLE(&hspi1);
	HAL_SPI_DeInit(&hspi1);
	HAL_GPIO_DeInit(SPI1_CS_GPIO_Port, SPI1_CS_Pin);
	asic_spi_enabled = 0;
}

void main_ASIC_GPIO_SPI_reinit(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	__HAL_RCC_GPIOD_CLK_ENABLE();
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);

	GPIO_InitStruct.Pin = SPI1_CS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(SPI1_CS_GPIO_Port, &GPIO_InitStruct);

	MX_SPI1_Init();
	HAL_SPI_Init(&hspi1);
	__HAL_SPI_ENABLE(&hspi1);
	asic_spi_enabled = 1;
}

uint8_t main_ASIC_GPIO_SPI_is_enabled(void)
{
	return asic_spi_enabled;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	debug_printf("ASSERT");

	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1)
	{
	}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */

  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
