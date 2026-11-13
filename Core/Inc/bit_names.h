/*
 * Copyright (c) 2024-2026 Murata Electronics Oy.
 * SPDX-License-Identifier: BSD-3-Clause
 * See LICENSE at the repository root.
 */

#ifndef INC_BIT_NAMES_H_
#define INC_BIT_NAMES_H_

// CONF
#define RST					0
#define DRO					5 //in documentation this is ODR, but that name is reserved by HAL

// STATUS
#define HARD_ERROR_MASK		0xAAA
#define SOFT_ERROR_MASK     0x555
#define SAZ					0
#define HAZ					1
#define SAY					2
#define HAY					3
#define SAX					4
#define HAX					5
#define SGZ					6
#define HGZ					7
#define SGY					8
#define HGY					9
#define SGX					10
#define HGX					11
#define RES					13
#define CMS                 18
#define ACRC                19
#define ACE                 20
#define DRY                 31

#endif /* INC_BIT_NAMES_H_ */
