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
static uint8_t lcd_fb[ST7565_HOR_RES_DEFAULT * ST7565_VER_RES_DEFAULT / 8] = {0xAA, 0xAA};

/**********************
 *      MACROS
 **********************/

/**********************
 *   GLOBAL FUNCTIONS
 **********************/

void st7565_init(lv_disp_drv_t *drv)
{
    // Set up LCD initialization structure
    lcd_init_cmd_t init_cmds[]={
		{ST7565_INTERNAL_RESET, {0}, 0x80},             // Soft reset with delay
		{ST7565_DISPLAY_OFF, {0}, 0x00},                // Display off
 		{ST7565_SET_DISP_NORMAL, {0}, 0x00},            // Set non-inverted mode
		{ST7565_SET_BIAS_7, {0}, 0x00},                 // LCD bias select
		{ST7565_SET_SEG_NORMAL, {0}, 0x00},             // SEG select - TODO: Why is this flipped for LVGL?
		{ST7565_SET_COM_NORMAL, {0}, 0x00},             // COM select
		{ST7565_SET_RESISTOR_RATIO | 0x05, {0}, 0x00},  // Set lcd operating voltage (regulator resistor, ref voltage resistor); RR = 5.5
		{ST7565_SET_VOLUME_FIRST, {0}, 0x00},     
		{ST7565_SET_VOLUME_SECOND | 0x17, {0}, 0x00},   // Set the contrast; EV = 23 = 0x17
		{ST7565_SET_POWER_CONTROL | 0x07, {0}, 0x00},   // Turn on voltage convert, regulator and follower (VC=1, VR=1, VF=1)
		{ST7565_DISPLAY_ON, {0x00}, 0x00},              // Turn on display  
		{0, {0}, 0xff}
    };

    // Apply the backlight LEDC PWM timer/channel configuration, turn backlight on
    ledc_timer_config_t disp_bl_timer = {
        .speed_mode       = DISP_BL_LEDC_MODE,
        .timer_num        = DISP_BL_LEDC_TIMER,
        .duty_resolution  = DISP_BL_LEDC_DUTY_RES,
        .freq_hz          = DISP_BL_LEDC_FREQUENCY,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&disp_bl_timer));

    ledc_channel_config_t disp_bl_channel = {
        .speed_mode     = DISP_BL_LEDC_MODE,
        .channel        = DISP_BL_LEDC_CHANNEL,
        .timer_sel      = DISP_BL_LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = DISP_BL_PWM_PIN,
        .duty           = DUTY_TO_LEDC_VALUE(100), // TEMP: Make it 100%
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&disp_bl_channel));

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

    // Clear the local buffer
    memset(lcd_fb, 0x00, sizeof(lcd_fb));
}

void st7565_flush(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map)
{
    // Return if the area is out the screen
    //if (area->x2 < 0) return;
    //if (area->y2 < 0) return;
    //if (area->x1 > st7565_hor_res - 1) return;
    //if (area->y1 > st7565_ver_res - 1) return;

    // Truncate the area to the screen
    //int32_t act_x1 = area->x1 < 0 ? 0 : area->x1;
    //int32_t act_y1 = area->y1 < 0 ? 0 : area->y1;
    //int32_t act_x2 = area->x2 > st7565_hor_res - 1 ? st7565_hor_res - 1 : area->x2;
    //int32_t act_y2 = area->y2 > st7565_ver_res - 1 ? st7565_ver_res - 1 : area->y2;

    int32_t x, y;

    // Refresh frame buffer
    for(y = area->y1; y <= area->y2; y++) {
        for(x = area->x1; x <= area->x2; x++) {
            if (lv_color_to1(*color_map) != 0) {
                lcd_fb[x + (y / 8)*get_display_hor_res(drv)] &= ~(1 << (7 - (y % 8)));
            } else {
                lcd_fb[x + (y / 8)*get_display_hor_res(drv)] |= (1 << (7 - (y % 8)));
            }
            color_map++;
        }
        //color_map += area->x2 - area->x2; // Next row
    }

    uint8_t columnLow = area->x1 & 0x0F;
	uint8_t columnHigh = (area->x1 >> 4) & 0x0F;
    uint8_t row1 = 0, row2 = 0;
    uint32_t size = 0;
    void *ptr;

    row1 = area->y1>>3;
    row2 = area->y2>>3;

    for(int i = row1; i < row2+1; i++) {

	    st7565_send_cmd(ST7565_SET_COLUMN_LOWER | columnHigh);  // Set Higher Column Start Address for Page Addressing Mode
	    st7565_send_cmd(ST7565_SET_COLUMN_UPPER | columnLow);   // Set Lower Column Start Address for Page Addressing Mode
	    st7565_send_cmd(ST7565_SET_PAGE | (7 - i));                   // Set Page Start Address for Page Addressing Mode
	    size = area->y2 - area->y1 + 1;
        ptr = color_map + i * get_display_hor_res(drv);
        if(i != row2) {
	        //st7565_send_data( (void *) ptr, size);
            st7565_send_data(&lcd_fb[(get_display_hor_res(drv) * i)], size);
	    } else {
	        // complete sending data by sh1107_send_color() and thus call lv_flush_ready()
	        //st7565_send_color( (void *) ptr, size);
            st7565_send_color(&lcd_fb[(get_display_hor_res(drv) * i)], size);
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
