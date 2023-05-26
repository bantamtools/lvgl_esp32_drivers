/**
 * @file lv_templ.h
 *
 */

#ifndef ST7565_H
#define ST7565_H

#ifdef __cplusplus
extern "C" {
#endif

/*********************
 *      INCLUDES
 *********************/
#include <stdbool.h>
#ifdef LV_LVGL_H_INCLUDE_SIMPLE
#include "lvgl.h"
#else
#include "lvgl/lvgl.h"
#endif

/*********************
 *      DEFINES
 *********************/

#define ST7565_DC   CONFIG_LV_DISP_PIN_DC
#define ST7565_RST  CONFIG_LV_DISP_PIN_RST

#if LVGL_VERSION_MAJOR < 8
#define ST7565_HOR_RES_DEFAULT  CONFIG_LV_HOR_RES_MAX
#define ST7565_VER_RES_DEFAULT  CONFIG_LV_VER_RES_MAX
#else
#define ST7565_HOR_RES_DEFAULT  128
#define ST7565_VER_RES_DEFAULT  64
#endif

#define ST7565_COL_OFFSET       0x4 // Column offset to shift display (needed for SEG/COM reversed)

// ST7565 specific commands used in init
#define ST7565_DISPLAY_OFF            0xAE
#define ST7565_DISPLAY_ON             0xAF

#define ST7565_SET_DISP_START_LINE    0x40
#define ST7565_SET_PAGE               0xB0

#define ST7565_SET_COLUMN_UPPER  	  0x10
#define ST7565_SET_COLUMN_LOWER  	  0x00

#define ST7565_SET_SEG_NORMAL         0xA0
#define ST7565_SET_SEG_REVERSE        0xA1

#define ST7565_SET_DISP_NORMAL        0xA6
#define ST7565_SET_DISP_INVERSE       0xA7

#define ST7565_SET_ALLPTS_NORMAL      0xA4
#define ST7565_SET_ALLPTS_ON          0xA5
#define ST7565_SET_BIAS_9             0xA2 
#define ST7565_SET_BIAS_7             0xA3

#define ST7565_RMW                    0xE0
#define ST7565_RMW_CLEAR              0xEE
#define ST7565_INTERNAL_RESET         0xE2
#define ST7565_SET_COM_NORMAL         0xC0
#define ST7565_SET_COM_REVERSE        0xC8
#define ST7565_SET_POWER_CONTROL      0x28
#define ST7565_SET_RESISTOR_RATIO     0x20
#define ST7565_SET_VOLUME_FIRST       0x81
#define ST7565_SET_VOLUME_SECOND      0
#define ST7565_SET_BOOSTER_FIRST      0xF8
#define ST7565_SET_BOOSTER_234        0
#define ST7565_SET_BOOSTER_5          1
#define ST7565_SET_BOOSTER_6          3
#define ST7565_NOP                    0xE3
#define ST7565_TEST                   0xF0

/**********************
 *      TYPEDEFS
 **********************/

/**********************
 * GLOBAL PROTOTYPES
 **********************/
void st7565_init(lv_disp_drv_t *drv);
void st7565_set_px_cb(lv_disp_drv_t *drv, uint8_t *buf, lv_coord_t buf_w, lv_coord_t x, lv_coord_t y,
        lv_color_t color, lv_opa_t opa);
void st7565_flush(lv_disp_drv_t * disp_drv, const lv_area_t * area, lv_color_t * color_p);
void st7565_rounder(lv_disp_drv_t * disp_drv, lv_area_t *area);

void st7565_sleep_in(void);
void st7565_sleep_out(void);

/**********************
 *      MACROS
 **********************/

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /*ST7565_H*/
