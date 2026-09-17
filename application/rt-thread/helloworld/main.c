/*
 * Copyright (c) 2022-2024, ArtInChip Technology Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Authors: weilin.peng@artinchip.com
 */

#include <rtthread.h>
#ifdef RT_USING_ULOG
#define LOG_TAG     "helloworld"
#define LOG_LVL     LOG_LVL_DBG
#include <ulog.h>
#endif

#ifdef LPKG_USING_LVGL
#include <lvgl.h>

static lv_obj_t *g_count_label;
static int g_count;

static void btn_event_cb(lv_event_t *e)
{
    g_count += 3;
    lv_label_set_text_fmt(g_count_label, "Count: %d", g_count);
}

/* Called by the LVGL thread via lv_user_gui_init() */
void ui_init(void)
{
    lv_obj_t *btn = lv_button_create(lv_screen_active());
    lv_obj_set_size(btn, 160, 60);
    lv_obj_center(btn);

    g_count_label = lv_label_create(btn);
    lv_label_set_text(g_count_label, "Count: 0");

    lv_obj_add_event_cb(btn, btn_event_cb, LV_EVENT_CLICKED, NULL);
}
#endif /* LPKG_USING_LVGL */

int main(void)
{
#ifdef ULOG_USING_FILTER
    ulog_global_filter_lvl_set(ULOG_OUTPUT_LVL);
#endif
    while (1)
    {
#ifdef RT_USING_ULOG
        LOG_I("Hello, ArtInChip! tick = %d", rt_tick_get());
#else
        rt_kprintf("Hello, ArtInChip! tick = %d\n", rt_tick_get());
#endif
        rt_thread_mdelay(1000);
    }
    return 0;
}
