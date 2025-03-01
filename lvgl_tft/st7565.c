/**
 * @file st7565.c
 *
 */

/*********************
 *      INCLUDES
 *********************/
#include "st7565.h"
#include "disp_spi.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/*********************
 *      DEFINES
 *********************/
 #define TAG "ST7565"

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
static void st7565_send_cmd(uint8_t cmd);
static void st7565_send_data(void * data, uint16_t length);
static void st7565_send_color(void * data, uint16_t length);
static void st7565_reset(void);
static lv_coord_t get_display_hor_res(lv_disp_drv_t *disp_drv);
static lv_coord_t get_display_ver_res(lv_disp_drv_t *disp_drv);

/**********************
 *  STATIC VARIABLES
 **********************/

/**********************
 *      MACROS
 **********************/
#define BIT_SET(a,b) ((a) |= (1U<<(b)))
#define BIT_CLEAR(a,b) ((a) &= ~(1U<<(b)))

/**********************
 *   GLOBAL FUNCTIONS
 **********************/

void st7565_init(lv_disp_drv_t *drv)
{
    // Set up LCD initialization structure
    lcd_init_cmd_t init_cmds[]={
		{ST7565_INTERNAL_RESET, {0}, 0x80},             // Soft reset with delay
		{ST7565_DISPLAY_OFF, {0}, 0x00},                // Display off
#if defined (CONFIG_LV_DISPLAY_ORIENTATION_PORTRAIT_INVERTED) || defined (CONFIG_LV_DISPLAY_ORIENTATION_LANDSCAPE_INVERTED)
 		{ST7565_SET_DISP_INVERSE, {0}, 0x00},           // Set inverted mode
#else
 		{ST7565_SET_DISP_NORMAL, {0}, 0x00},            // Set non-inverted mode
#endif
		{ST7565_SET_BIAS_7, {0}, 0x00},                 // LCD bias select
		{ST7565_SET_SEG_NORMAL, {0}, 0x00},             // SEG select - TODO: Why is SEG/COM flipped for LVGL?
		{ST7565_SET_COM_REVERSE, {0}, 0x00},            // COM select
#if defined (CONFIG_LV_DISPLAY_ORIENTATION_PORTRAIT_INVERTED) || defined (CONFIG_LV_DISPLAY_ORIENTATION_LANDSCAPE_INVERTED)
		{ST7565_SET_RESISTOR_RATIO | 0x06, {0}, 0x00},  // Set lcd operating voltage (regulator resistor, ref voltage resistor); RR = 6.0
		{ST7565_SET_VOLUME_FIRST, {0}, 0x00},     
		{ST7565_SET_VOLUME_SECOND | 0x18, {0}, 0x00},   // Set the contrast; EV = 24 = 0x18
#else
		{ST7565_SET_RESISTOR_RATIO | 0x05, {0}, 0x00},  // Set lcd operating voltage (regulator resistor, ref voltage resistor); RR = 5.5
		{ST7565_SET_VOLUME_FIRST, {0}, 0x00},     
		{ST7565_SET_VOLUME_SECOND | 0x17, {0}, 0x00},   // Set the contrast; EV = 23 = 0x17
#endif
		{ST7565_SET_POWER_CONTROL | 0x07, {0}, 0x00},   // Turn on voltage convert, regulator and follower (VC=1, VR=1, VF=1)
		{ST7565_DISPLAY_ON, {0x00}, 0x00},              // Turn on display  
		{0, {0}, 0xff}
    };

	// Initialize non-SPI GPIOs
    gpio_reset_pin(ST7565_DC);
	gpio_set_direction(ST7565_DC, GPIO_MODE_OUTPUT);
    gpio_reset_pin(ST7565_RST);
	gpio_set_direction(ST7565_RST, GPIO_MODE_OUTPUT);

	// Reset the display
    st7565_reset();

	ESP_LOGI(TAG, "ST7565 initialization.");

	// Send all the commands
	uint16_t cmd = 0;
	while (init_cmds[cmd].databytes != 0xff) {

		st7565_send_cmd(init_cmds[cmd].cmd);

        // Send data if present
        if (init_cmds[cmd].data) {
		    st7565_send_data(init_cmds[cmd].data, init_cmds[cmd].databytes & 0x1F);
        }
		
        // Delay if requested
        if (init_cmds[cmd].databytes & 0x80) {
			vTaskDelay(100 / portTICK_PERIOD_MS);
		}
		
        cmd++;
	}
}

void st7565_set_px_cb(lv_disp_drv_t *drv, uint8_t *buf, lv_coord_t buf_w, lv_coord_t x, lv_coord_t y,
        lv_color_t color, lv_opa_t opa)
{
    uint16_t byte_index = 0;
    uint8_t  bit_index = 0;

#if defined (CONFIG_LV_DISPLAY_ORIENTATION_LANDSCAPE) || defined (CONFIG_LV_DISPLAY_ORIENTATION_LANDSCAPE_INVERTED)
	byte_index = y + (( x>>3 ) * get_display_ver_res(drv));
	bit_index  = x & 0x7;
#else
    byte_index = x + (( y>>3 ) * get_display_hor_res(drv));
    bit_index  = y & 0x7;
#endif

    if ((color.full == 0) && (LV_OPA_TRANSP != opa)) {
        BIT_SET(buf[byte_index], bit_index);
    } else {
        BIT_CLEAR(buf[byte_index], bit_index);
    }
}

void st7565_flush(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map)
{
    uint8_t columnLow = (area->x1 + ST7565_COL_OFFSET) & 0x0F;
	uint8_t columnHigh = ((area->x1 + ST7565_COL_OFFSET) >> 4) & 0x0F;
    uint8_t row1 = 0, row2 = 0, col1 = 0, col2 = 0;
    void *ptr;

#if defined (CONFIG_LV_DISPLAY_ORIENTATION_LANDSCAPE) || defined (CONFIG_LV_DISPLAY_ORIENTATION_LANDSCAPE_INVERTED)
    row1 = area->x1 >> 3;
    row2 = area->x2 >> 3;
    col1 = area->y1;
    col2 = area->y2;
#else
    row1 = area->y1 >> 3;
    row2 = area->y2 >> 3;
    col1 = area->x1;
    col2 = area->x2;
#endif

    for(int i = row1; i < (row2 + 1); i++) {

	    st7565_send_cmd(ST7565_SET_COLUMN_LOWER | columnLow);   // Set Higher Column Start Address for Page Addressing Mode
	    st7565_send_cmd(ST7565_SET_COLUMN_UPPER | columnHigh);  // Set Lower Column Start Address for Page Addressing Mode
	    st7565_send_cmd(ST7565_SET_PAGE | i);                   // Set Page Start Address for Page Addressing Mode
        
#if defined (CONFIG_LV_DISPLAY_ORIENTATION_LANDSCAPE) || defined (CONFIG_LV_DISPLAY_ORIENTATION_LANDSCAPE_INVERTED)
        ptr = color_map + i * get_display_ver_res(drv);
#else
        ptr = color_map + i * get_display_hor_res(drv);
#endif

        for(int j = col1; j < (col2 + 1); j++) {
            if (i != row2 || j != col2) {
                st7565_send_data(ptr + j, 1);
            } else {
                st7565_send_color(ptr + j, 1);  // Complete sending data by st7565_send_color() and thus call lv_flush_ready()
            }
        }
    }
}

void st7565_rounder(lv_disp_drv_t * disp_drv, lv_area_t *area)
{
    // workaround: always send complete size display buffer
    area->x1 = 0;
    area->y1 = 0;
    area->x2 = get_display_hor_res(disp_drv) - 1;
    area->y2 = get_display_ver_res(disp_drv) - 1;
}

void st7565_sleep_in()
{
	st7565_send_cmd(ST7565_DISPLAY_OFF);
	st7565_send_cmd(ST7565_SET_ALLPTS_ON);
}

void st7565_sleep_out()
{
    st7565_send_cmd(ST7565_SET_ALLPTS_NORMAL);
	st7565_send_cmd(ST7565_DISPLAY_ON);
}

/**********************
 *   STATIC FUNCTIONS
 **********************/

static void st7565_send_cmd(uint8_t cmd)
{
    disp_wait_for_pending_transactions();
    gpio_set_level(ST7565_DC, 0);	 /*Command mode*/
    disp_spi_send_data(&cmd, 1);
}

static void st7565_send_data(void * data, uint16_t length)
{
    disp_wait_for_pending_transactions();
    gpio_set_level(ST7565_DC, 1);	 /*Data mode*/
    disp_spi_send_data(data, length);
}

static void st7565_send_color(void * data, uint16_t length)
{
    disp_wait_for_pending_transactions();
    gpio_set_level(ST7565_DC, 1);   /*Data mode*/
    disp_spi_send_colors(data, length);
}

static void st7565_reset(void)
{
	gpio_set_level(ST7565_RST, 0);
	vTaskDelay(100 / portTICK_PERIOD_MS);
	gpio_set_level(ST7565_RST, 1);
	vTaskDelay(100 / portTICK_PERIOD_MS);
}

static lv_coord_t get_display_ver_res(lv_disp_drv_t *disp_drv)
{
    return disp_drv->ver_res;
}

static lv_coord_t get_display_hor_res(lv_disp_drv_t *disp_drv)
{
    return disp_drv->hor_res; 
}
