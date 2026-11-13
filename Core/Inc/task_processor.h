/*
 * Copyright (c) 2024-2026 Murata Electronics Oy.
 * SPDX-License-Identifier: BSD-3-Clause
 * See LICENSE at the repository root.
 */

#ifndef INC_TASK_PROCESSOR_H_
#define INC_TASK_PROCESSOR_H_

#define	AVG_FACTOR_100HZ	20
#define	AVG_FACTOR_200HZ	10
#define DEFAULT_AVG_FACTOR	AVG_FACTOR_100HZ

void process_tasks(void);
uint16_t get_averaging_factor(void);

#endif /* INC_TASK_PROCESSOR_H_ */
