/**
 * @file mpi3501.c
 *
 */

/*********************
 *      INCLUDES
 *********************/
#include "ili9486.h"
#include "disp_spi.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/*********************
 *      DEFINES
 *********************/

/**********************
 *      TYPEDEFS
 **********************/

/*The LCD needs a bunch of command/argument values to be initialized. They are stored in this struct. */
typedef struct {
    uint8_t cmd;
    uint8_t data[16];
    uint8_t databytes; //No of data in data; bit 7 = delay after set; 0xFF = end of cmds.
} lcd_init_cmd_t;

/**********************
 *  STATIC PROTOTYPES
 **********************/
static void ili9486_set_orientation(uint8_t orientation);

static void ili9486_send_cmd(uint8_t cmd);
static void ili9486_send_data(void * data, uint16_t length);
static void ili9486_send_color(void * data, uint16_t length);
static void ili9486_reset(void);
/**********************
 *  STATIC VARIABLES
 **********************/

/**********************
 *      MACROS
 **********************/

/**********************
 *   GLOBAL FUNCTIONS
 **********************/

void ili9486_init(void)
{
    lcd_init_cmd_t ili_init_cmds[]={
        {0x11, {0}, 0x80},
        {0x3A, {0x55}, 1},
        {0x2C, {0x44}, 1},
        {0xC5, {0x00, 0x00, 0x00, 0x00}, 4},
        {0xE0, {0x0F, 0x1F, 0x1C, 0x0C, 0x0F, 0x08, 0x48, 0x98, 0x37, 0x0A, 0x13, 0x04, 0x11, 0x0D, 0x00}, 15},
        {0XE1, {0x0F, 0x32, 0x2E, 0x0B, 0x0D, 0x05, 0x47, 0x75, 0x37, 0x06, 0x10, 0x03, 0x24, 0x20, 0x00}, 15},
        {0x20, {0}, 0},				/* display inversion OFF */
        {0x36, {0x48}, 1},
        {0x29, {0}, 0x80},			/* display on */
        {0x00, {0}, 0xff},
    };

    ili9486_reset();

    LV_LOG_INFO("ILI9486 Initialization.");

    //Send all the commands
    uint16_t cmd = 0;
    while (ili_init_cmds[cmd].databytes!=0xff) {
        ili9486_send_cmd(ili_init_cmds[cmd].cmd);
        ili9486_send_data(ili_init_cmds[cmd].data, ili_init_cmds[cmd].databytes&0x1F);
        if (ili_init_cmds[cmd].databytes & 0x80) {
                vTaskDelay(pdMS_TO_TICKS(100));
        }
        cmd++;
    }

    ili9486_set_orientation(CONFIG_LV_DISPLAY_ORIENTATION);
}

void ili9486_flush(lv_disp_drv_t * drv, const lv_area_t * area, lv_color_t * color_map)
{
    uint8_t data[4] = {0};
    /* 2 is the number of bytes in color depth */
    uint32_t size = lv_area_get_width(area) * lv_area_get_height(area) * 2;

    /*Column addresses*/
    ili9486_send_cmd(0x2A);
    data[0] = (area->x1 >> 8) & 0xFF;
    data[1] = area->x1 & 0xFF;
    data[2] = (area->x2 >> 8) & 0xFF;
    data[3] = area->x2 & 0xFF;
    ili9486_send_data(data, 4);

    /*Page addresses*/
    ili9486_send_cmd(0x2B);
    data[0] = (area->y1 >> 8) & 0xFF;
    data[1] = area->y1 & 0xFF;
    data[2] = (area->y2 >> 8) & 0xFF;
    data[3] = area->y2 & 0xFF;
    ili9486_send_data(data, 4);

    /*Memory write*/
    ili9486_send_cmd(0x2C);
    ili9486_send_color((void*) color_map, size);
}

/**********************
 *   STATIC FUNCTIONS
 **********************/
static void ili9486_send_cmd(uint8_t cmd)
{
    uint8_t to16bit[] = {
        0x00, cmd
    };

    disp_wait_for_pending_transactions();
    gpio_set_level(ILI9486_DC, 0);	 /*Command mode*/
    disp_spi_send_data(to16bit, sizeof to16bit);
}

static void ili9486_send_data(void * data, uint16_t length)
{
    uint32_t i;
    uint8_t to16bit[32];
    uint8_t * dummy = data;

    for(i=0; i < (length); i++)
    {
      to16bit[2*i+1] = dummy[i];
      to16bit[2*i] = 0x00;
    }

    disp_wait_for_pending_transactions();
    gpio_set_level(ILI9486_DC, 1);	 /*Data mode*/
    disp_spi_send_data(to16bit, (length*2));
}

static void ili9486_send_color(void * data, uint16_t length)
{
    disp_wait_for_pending_transactions();
    gpio_set_level(ILI9486_DC, 1);   /*Data mode*/
    disp_spi_send_colors(data, length);
}

static void ili9486_set_orientation(uint8_t orientation)
{
    assert(orientation < 4);

    const uint8_t data[] = {0x48, 0x88, 0x28, 0xE8};

#if (LV_USE_LOG == 1)
    const char *orientation_str[] = {
        "PORTRAIT", "PORTRAIT_INVERTED", "LANDSCAPE", "LANDSCAPE_INVERTED"
    };

    LV_LOG_INFO("Display orientation: %s", orientation_str[orientation]);
    LV_LOG_INFO("0x36 command value: 0x%02X", data[orientation]);
#endif

    ili9486_send_cmd(0x36);
    ili9486_send_data((void *) &data[orientation], 1);
}

static void ili9486_reset(void)
{
#if ILI9486_USE_RST
    gpio_set_level(ILI9486_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
    gpio_set_level(ILI9486_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(100));
#endif
}
