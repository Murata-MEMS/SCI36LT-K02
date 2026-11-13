/*
 * Copyright (c) 2024-2026 Murata Electronics Oy.
 * SPDX-License-Identifier: BSD-3-Clause
 * See LICENSE at the repository root.
 */

#include "stm32l4xx_hal.h"
#include "stdbool.h"
#include "stdint.h"
#include "string.h"
#include "stdio.h"
#include "stdarg.h"
#include "ctype.h"
#include "StrFunctions.h"
#include "main.h"
#if DEBUG_PRINTS_ON_USB
#include "usbd_cdc_if.h"
#endif

#define DEBUG_BUFFER_SIZE	1024U
#define NEWLINE_LEN			2U // \r\n

extern volatile bool tx_done_flag;
extern UART_HandleTypeDef huart3;

void debug_printf(const char *format, ...) {
#if DEBUG_PRINTS_ON
	static char buffer[DEBUG_BUFFER_SIZE];

#if DEBUG_PRINTS_USE_UART
    while (!tx_done_flag) {
        // Wait until DMA is finished with buffer
    }
#endif

    va_list args;
    va_start(args, format);
    int length = vsnprintf(
        buffer,
        sizeof(buffer) - NEWLINE_LEN,
        format,
        args
    );
    va_end(args);

    if (length < 0)
    {
        return;
    }

    size_t used = (size_t)length;

    // vsnprintf() returns the length it wanted to write, even if truncation occurred.
    if (used >= sizeof(buffer) - NEWLINE_LEN)
    {
        used = sizeof(buffer) - NEWLINE_LEN - 1U;
    }

    buffer[used++] = '\r';
    buffer[used++] = '\n';
    buffer[used] = '\0';

#if DEBUG_PRINTS_USE_UART
    DATA_TX((uint8_t *)buffer, (uint16_t)used);
#else
    USB_DATA_TX((uint8_t *)buffer, (uint16_t)used);
#endif
#endif
}

void DATA_TX(uint8_t* tx_buf, uint16_t tx_len) {
	while(!tx_done_flag) {
		// Wait for previous transmission to finish (possibly redundant)
	}
	tx_done_flag = false;
	while(HAL_UART_Transmit_DMA(&huart3, tx_buf, tx_len) != HAL_OK) {

	}
}

#if DEBUG_PRINTS_ON_USB
void USB_DATA_TX(uint8_t *tx_buf, uint16_t tx_len) {
    CDC_Transmit_FS(tx_buf, tx_len);
    HAL_Delay(1);
}
#endif


