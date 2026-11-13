/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
void ISR_2kHz(void);

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

#define FWVER 						2026101402
#define DEBUG_PRINTS_ON				0
#define DEBUG_PRINTS_USE_UART		0
#define DEBUG_PRINTS_ON_USB		(DEBUG_PRINTS_ON && !DEBUG_PRINTS_USE_UART)


/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SPI3_EXT_CS_Pin GPIO_PIN_15
#define SPI3_EXT_CS_GPIO_Port GPIOA
#define SPI3_EXT_MOSI_Pin GPIO_PIN_12
#define SPI3_EXT_MOSI_GPIO_Port GPIOC
#define SPI1_CS_Pin GPIO_PIN_2
#define SPI1_CS_GPIO_Port GPIOD
#define CAN_TX_Pin GPIO_PIN_9
#define CAN_TX_GPIO_Port GPIOB
#define CAN_RX_Pin GPIO_PIN_8
#define CAN_RX_GPIO_Port GPIOB
#define UART1_TX_Pin GPIO_PIN_9
#define UART1_TX_GPIO_Port GPIOA
#define UART1_RX_Pin GPIO_PIN_10
#define UART1_RX_GPIO_Port GPIOA
#define CFG_RST_Pin GPIO_PIN_0
#define CFG_RST_GPIO_Port GPIOC
#define ASIC_LDO_EN_Pin GPIO_PIN_7
#define ASIC_LDO_EN_GPIO_Port GPIOC
#define UART1_CK_Pin GPIO_PIN_8
#define UART1_CK_GPIO_Port GPIOA
#define DATA_READY_Pin GPIO_PIN_1
#define DATA_READY_GPIO_Port GPIOC
#define QUAD_SPI_IO1_Pin GPIO_PIN_0
#define QUAD_SPI_IO1_GPIO_Port GPIOB
#define QUAD_SPI_IO3_Pin GPIO_PIN_6
#define QUAD_SPI_IO3_GPIO_Port GPIOA
#define QUAD_SPI_NCS_Pin GPIO_PIN_11
#define QUAD_SPI_NCS_GPIO_Port GPIOB
#define QUAD_SPI_CLK_Pin GPIO_PIN_10
#define QUAD_SPI_CLK_GPIO_Port GPIOB
#define QUAD_SPI_IO0_Pin GPIO_PIN_1
#define QUAD_SPI_IO0_GPIO_Port GPIOB
#define QUAD_SPI_IO2_Pin GPIO_PIN_7
#define QUAD_SPI_IO2_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */
#define UART_DMA_RESET_TIMEOUT 2000 // Times 2kHz (2000 = 1s)
#define USB_DM_Pin GPIO_PIN_11
#define USB_DM_GPIO_Port GPIOA
#define USB_DP_Pin GPIO_PIN_12
#define USB_DP_GPIO_Port GPIOA

void check_uart_state(void);
void main_set_data_ready_pin_state(uint8_t state);
void main_set_asic_crc_error(void);
void main_set_asic_command_error(void);
void main_ASIC_GPIO_SPI_disable(void);
void main_ASIC_GPIO_SPI_reinit(void);
uint8_t main_ASIC_GPIO_SPI_is_enabled(void);

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
