/*
 * Copyright (c) 2024-2026 Murata Electronics Oy.
 * SPDX-License-Identifier: BSD-3-Clause
 * See LICENSE at the repository root.
 */

#include "stdint.h"
#include "main.h"

void debug_printf(const char* format, ...);
void DATA_TX(uint8_t* tx_buf, uint16_t tx_len);
#if DEBUG_PRINTS_ON_USB
void USB_DATA_TX(uint8_t* tx_buf, uint16_t tx_len);
#endif
