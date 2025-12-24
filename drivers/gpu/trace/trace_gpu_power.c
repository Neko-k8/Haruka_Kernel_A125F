// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2025 Google LLC
 */

#include <linux/types.h>
#include <linux/module.h>

#define CREATE_TRACE_POINTS
#include <trace/events/gpu_power.h>

#if !defined(CONFIG_MALI_BIFROST_R38P1)
EXPORT_TRACEPOINT_SYMBOL(gpu_frequency);
#endif
EXPORT_TRACEPOINT_SYMBOL(gpu_work_period);
