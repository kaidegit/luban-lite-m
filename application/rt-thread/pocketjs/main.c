/*
 * Copyright (c) 2022-2026, ArtInChip Technology Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <rtthread.h>
#include <stdint.h>

#ifdef RT_USING_ULOG
#define LOG_TAG     "pocketjs"
#define LOG_LVL     LOG_LVL_DBG
#include <ulog.h>
#endif

#ifndef POCKETJS_ROOT
#define POCKETJS_ROOT ""
#endif

/* The guest package is linked into the firmware: tools/aic.ts package emits
 * generated/pocket_bin.c and hosts/aic/SConscript compiles it. */
extern const uint8_t pocketjs_embedded_package[];
extern const uint32_t pocketjs_embedded_package_size;

int pocketjs_aic_run_embedded(const uint8_t *bytes, size_t size);

int main(void)
{
    pocketjs_aic_run_embedded(pocketjs_embedded_package,
                              pocketjs_embedded_package_size);
    return 0;
}
