
/**
 * @file pcd8544.h
 *
 */

#ifndef PCD8544_H
#define PCD8544_H

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

/**********************
 *      TYPEDEFS
 **********************/

/**********************
 * GLOBAL PROTOTYPES
 **********************/

void pcd8544_init(lv_disp_drv_t *drv);
void pcd8544_flush(lv_disp_drv_t * drv, const lv_area_t * area, lv_color_t * color_map);
void pcd8544_rounder(lv_disp_drv_t * disp_drv, lv_area_t *area);
void pcd8544_set_px_cb(lv_disp_drv_t * disp_drv, uint8_t * buf, lv_coord_t buf_w, lv_coord_t x, lv_coord_t y,
                       lv_color_t color, lv_opa_t opa);
void pcd8544_set_contrast(lv_disp_drv_t *drv, uint8_t contrast);

/**********************
 *      MACROS
 **********************/


#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /*PCD8544_H*/
