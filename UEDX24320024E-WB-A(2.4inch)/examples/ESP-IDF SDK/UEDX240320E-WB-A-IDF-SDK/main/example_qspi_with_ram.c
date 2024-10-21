/*
 * SPDX-FileCopyrightText: 2023-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/spi_master.h"
#include "esp_timer.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lvgl_port.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_check.h"

#include "esp_lcd_touch_cst816s.h"
#include "esp_lcd_gc9a01.h"
#include "lvgl.h"
#include "lv_demos.h"

//*************************************************** */

/* LCD size */
#define EXAMPLE_LCD_H_RES   (240)
#define EXAMPLE_LCD_V_RES   (320)

/* LCD settings */
#define EXAMPLE_LCD_SPI_NUM         (SPI3_HOST)
#define EXAMPLE_LCD_PIXEL_CLK_HZ    (80 * 1000 * 1000)
#define EXAMPLE_LCD_CMD_BITS        (8)
#define EXAMPLE_LCD_PARAM_BITS      (8)
#define EXAMPLE_LCD_COLOR_SPACE     (ESP_LCD_COLOR_SPACE_BGR)
#define EXAMPLE_LCD_BITS_PER_PIXEL  (16)
#define EXAMPLE_LCD_DRAW_BUFF_DOUBLE (0)
#define EXAMPLE_LCD_DRAW_BUFF_HEIGHT (80)
#define EXAMPLE_LCD_BL_ON_LEVEL      (1)

// The MCU interface mode select
#define EXAMPLE_PIN_NUM_IM0             (47)  
#define EXAMPLE_PIN_NUM_IM1             (48)  
#define EXAMPLE_PIN_NUM_IM_ON_LEVEL     (1)
#define EXAMPLE_PIN_NUM_IM_OFF_LEVEL    (0)


/* LCD pins */
#define EXAMPLE_LCD_GPIO_SCLK       (GPIO_NUM_40)
#define EXAMPLE_LCD_GPIO_MOSI       (GPIO_NUM_45)
#define EXAMPLE_LCD_GPIO_RST        (GPIO_NUM_NC)
#define EXAMPLE_LCD_GPIO_DC         (GPIO_NUM_41)
#define EXAMPLE_LCD_GPIO_CS         (GPIO_NUM_42)
#define EXAMPLE_LCD_GPIO_BL         (GPIO_NUM_13)

/* Touch settings */
#define EXAMPLE_TOUCH_I2C_NUM       (0)
#define EXAMPLE_TOUCH_I2C_CLK_HZ    (400000)

/* LCD touch pins */
#define EXAMPLE_TOUCH_I2C_SCL       (GPIO_NUM_3)
#define EXAMPLE_TOUCH_I2C_SDA       (GPIO_NUM_1)
#define EXAMPLE_TOUCH_GPIO_INT      (GPIO_NUM_4)

static const char *TAG = "EXAMPLE";

// LVGL image declare
// LV_IMG_DECLARE(esp_logo)

/* LCD IO and panel */
static esp_lcd_panel_io_handle_t lcd_io = NULL;
static esp_lcd_panel_handle_t lcd_panel = NULL;
static esp_lcd_touch_handle_t touch_handle = NULL;

/* LVGL display and touch */
static lv_display_t *lvgl_disp = NULL;
static lv_indev_t *lvgl_touch_indev = NULL;

static const gc9a01_lcd_init_cmd_t lcd_init_cmds[] = {
//  {cmd, { data }, data_size, delay_ms}
    {0xfe, (uint8_t []){0x00}, 0, 0},
    {0xef, (uint8_t []){0x00}, 0, 0},
    {0x36, (uint8_t []){0x48}, 1, 0},
    {0x3a, (uint8_t []){0x05}, 1, 0},

    {0x86, (uint8_t []){0x98}, 1, 0},

    {0x89, (uint8_t []){0x13}, 1, 0},

    {0x8b, (uint8_t []){0x80}, 1, 0},

    {0x8d, (uint8_t []){0x33}, 1, 0},
    {0x8e, (uint8_t []){0x0f}, 1, 0},

    {0xe8, (uint8_t []){0x12, 0x00}, 2, 0},

    {0xec, (uint8_t []){0x13, 0x02, 0x88}, 3, 0},

    {0xff, (uint8_t []){0x62}, 1, 0},

    {0x99, (uint8_t []){0x3e}, 1, 0},
    {0x9d, (uint8_t []){0x4b}, 1, 0},
    {0x98, (uint8_t []){0x3e}, 1, 0},
    {0x9c, (uint8_t []){0x4b}, 1, 0},

    {0xc3, (uint8_t []){0x27}, 1, 0},

    {0xc4, (uint8_t []){0x18}, 1, 0},

    {0xc9, (uint8_t []){0x0a}, 1, 0},

    {0xf0, (uint8_t []){0x47, 0x0c, 0x0A, 0x09, 0x15, 0x33}, 6, 0},

    {0xf1, (uint8_t []){0x4b, 0x8F, 0x8f, 0x3B, 0x3F, 0x6f}, 6, 0},

    {0xf2, (uint8_t []){0x47, 0x0c, 0x0A, 0x09, 0x15, 0x33}, 6, 0},

    {0xf3, (uint8_t []){0x4b, 0x8f, 0x8f, 0x3B, 0x3F, 0x6f}, 6, 0},

    {0x11, (uint8_t []){0x00}, 0, 100},  // delay_ms(120)
    {0x29, (uint8_t []){0x00}, 0, 0},
    {0x2c, (uint8_t []){0x00}, 0, 0},

    // /*2.8inch*/
    // {0xfe, (uint8_t []){0x00}, 0, 0},                            // 0xfe command, no data
    // {0xfe, (uint8_t []){0x00}, 0, 0},                            // 0xfe command, no data
    // {0xef, (uint8_t []){0x00}, 0, 0},                            // 0xef command, no data
    // {0x36, (uint8_t []){0x48}, 1, 0},              // 0x36 command with data 0x48
    // {0x3a, (uint8_t []){0x05}, 1, 0},              // 0x3a command with data 0x05

    // {0x86, (uint8_t []){0x98}, 1, 0},              // 0x86 command with data 0x98
    // {0x89, (uint8_t []){0x03}, 1, 0},              // 0x89 command with data 0x03
    // {0x8b, (uint8_t []){0x80}, 1, 0},              // 0x8b command with data 0x80
    // {0x8d, (uint8_t []){0x33}, 1, 0},              // 0x8d command with data 0x33
    // {0x8e, (uint8_t []){0x0f}, 1, 0},              // 0x8e command with data 0x0f

    // {0xe8, (uint8_t []){0x12, 0x00}, 2, 0},        // 0xe8 command with data 0x12, 0x00
    // {0xc3, (uint8_t []){0x1d}, 1, 0},              // 0xc3 command with data 0x1d
    // {0xc4, (uint8_t []){0x1d}, 1, 0},              // 0xc4 command with data 0x1d
    // {0xc9, (uint8_t []){0x0f}, 1, 0},              // 0xc9 command with data 0x0f

    // {0xff, (uint8_t []){0x62}, 1, 0},              // 0xff command with data 0x62

    // {0x99, (uint8_t []){0x3e}, 1, 0},              // 0x99 command with data 0x3e
    // {0x9d, (uint8_t []){0x4b}, 1, 0},              // 0x9d command with data 0x4b
    // {0x98, (uint8_t []){0x3e}, 1, 0},              // 0x98 command with data 0x3e
    // {0x9c, (uint8_t []){0x4b}, 1, 0},              // 0x9c command with data 0x4b

    // {0xf0, (uint8_t []){0x49, 0x0b, 0x09, 0x08, 0x06, 0x2e}, 6, 0},  // 0xf0 command with 6 data bytes
    // {0xf2, (uint8_t []){0x49, 0x0b, 0x09, 0x08, 0x06, 0x2e}, 6, 0},  // 0xf2 command with 6 data bytes

    // {0xf1, (uint8_t []){0x45, 0x92, 0x93, 0x2b, 0x31, 0x6F}, 6, 0},  // 0xf1 command with 6 data bytes
    // {0xf3, (uint8_t []){0x45, 0x92, 0x93, 0x2b, 0x31, 0x6F}, 6, 0},  // 0xf3 command with 6 data bytes

    // {0x35, (uint8_t []){0x00}, 1, 0},              // 0x35 command with data 0x00
    // {0x11, NULL, 0, 120},                          // 0x11 command, no data, 120ms delay
    // {0x29, NULL, 0, 0},                            // 0x29 command, no data
    // {0x2c, NULL, 0, 0},                            // 0x2c command, no data
};

static esp_err_t app_lcd_init(void)
{
    esp_err_t ret = ESP_OK;

    /* LCD backlight */
    gpio_config_t bk_gpio_config = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << EXAMPLE_LCD_GPIO_BL
    };
    ESP_ERROR_CHECK(gpio_config(&bk_gpio_config));

    gpio_config_t im1_gpio_config = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << EXAMPLE_PIN_NUM_IM1
    };
    ESP_ERROR_CHECK(gpio_config(&im1_gpio_config));
    gpio_config_t im0_gpio_config = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << EXAMPLE_PIN_NUM_IM0
    };
    ESP_ERROR_CHECK(gpio_config(&im0_gpio_config));

    ESP_ERROR_CHECK(gpio_set_level(EXAMPLE_PIN_NUM_IM0, EXAMPLE_PIN_NUM_IM_OFF_LEVEL));
    ESP_ERROR_CHECK(gpio_set_level(EXAMPLE_PIN_NUM_IM1, EXAMPLE_PIN_NUM_IM_ON_LEVEL));

    /* LCD initialization */
    ESP_LOGD(TAG, "Initialize SPI bus");
    const spi_bus_config_t buscfg = {
        .sclk_io_num = EXAMPLE_LCD_GPIO_SCLK,
        .mosi_io_num = EXAMPLE_LCD_GPIO_MOSI,
        .miso_io_num = GPIO_NUM_NC,
        .quadwp_io_num = GPIO_NUM_NC,
        .quadhd_io_num = GPIO_NUM_NC,
        .max_transfer_sz = EXAMPLE_LCD_H_RES * EXAMPLE_LCD_V_RES * sizeof(uint16_t),
    };
    ESP_RETURN_ON_ERROR(spi_bus_initialize(EXAMPLE_LCD_SPI_NUM, &buscfg, SPI_DMA_CH_AUTO), TAG, "SPI init failed");

    ESP_LOGD(TAG, "Install panel IO");
    const esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = EXAMPLE_LCD_GPIO_DC,
        .cs_gpio_num = EXAMPLE_LCD_GPIO_CS,
        .pclk_hz = EXAMPLE_LCD_PIXEL_CLK_HZ,
        .lcd_cmd_bits = EXAMPLE_LCD_CMD_BITS,
        .lcd_param_bits = EXAMPLE_LCD_PARAM_BITS,
        .spi_mode = 0,
        .trans_queue_depth = 10,
    };
    ESP_GOTO_ON_ERROR(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)EXAMPLE_LCD_SPI_NUM, &io_config, &lcd_io), err, TAG, "New panel IO failed");

    ESP_LOGD(TAG, "Install LCD driver");

    gc9a01_vendor_config_t vendor_config = {  // Uncomment these lines if use custom initialization commands
        .init_cmds = lcd_init_cmds,
        .init_cmds_size = sizeof(lcd_init_cmds) / sizeof(gc9a01_lcd_init_cmd_t),
    };

    const esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = EXAMPLE_LCD_GPIO_RST,
        .color_space = EXAMPLE_LCD_COLOR_SPACE,
        .bits_per_pixel = EXAMPLE_LCD_BITS_PER_PIXEL,
        .vendor_config = &vendor_config,            // Uncomment this line if use custom initialization commands
    };
    ESP_GOTO_ON_ERROR(esp_lcd_new_panel_gc9a01(lcd_io, &panel_config, &lcd_panel), err, TAG, "New panel failed");

    esp_lcd_panel_reset(lcd_panel);
    esp_lcd_panel_init(lcd_panel);
    // esp_lcd_panel_swap_xy(lcd_panel, true);
    esp_lcd_panel_mirror(lcd_panel, true, false);
    esp_lcd_panel_invert_color(lcd_panel, false);
    esp_lcd_panel_disp_on_off(lcd_panel, true);

    /* LCD backlight on */
    ESP_ERROR_CHECK(gpio_set_level(EXAMPLE_LCD_GPIO_BL, EXAMPLE_LCD_BL_ON_LEVEL));
    // ESP_ERROR_CHECK(gpio_set_level(EXAMPLE_PIN_NUM_IM0, EXAMPLE_PIN_NUM_IM_OFF_LEVEL));
    // ESP_ERROR_CHECK(gpio_set_level(EXAMPLE_PIN_NUM_IM1, EXAMPLE_PIN_NUM_IM_ON_LEVEL));

    return ret;

err:
    if (lcd_panel) {
        esp_lcd_panel_del(lcd_panel);
    }
    if (lcd_io) {
        esp_lcd_panel_io_del(lcd_io);
    }
    spi_bus_free(EXAMPLE_LCD_SPI_NUM);
    return ret;
}

static esp_err_t app_touch_init(void)
{
    /* Initilize I2C */
    const i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = EXAMPLE_TOUCH_I2C_SDA,
        .sda_pullup_en = GPIO_PULLUP_DISABLE,
        .scl_io_num = EXAMPLE_TOUCH_I2C_SCL,
        .scl_pullup_en = GPIO_PULLUP_DISABLE,
        .master.clk_speed = EXAMPLE_TOUCH_I2C_CLK_HZ
    };
    ESP_RETURN_ON_ERROR(i2c_param_config(EXAMPLE_TOUCH_I2C_NUM, &i2c_conf), TAG, "I2C configuration failed");
    ESP_RETURN_ON_ERROR(i2c_driver_install(EXAMPLE_TOUCH_I2C_NUM, i2c_conf.mode, 0, 0, 0), TAG, "I2C initialization failed");

    /* Initialize touch HW */
    const esp_lcd_touch_config_t tp_cfg = {
        .x_max = EXAMPLE_LCD_H_RES,
        .y_max = EXAMPLE_LCD_V_RES,
        .rst_gpio_num = GPIO_NUM_NC, // Shared with LCD reset
        .int_gpio_num = EXAMPLE_TOUCH_GPIO_INT,
        .levels = {
            .reset = 0,
            .interrupt = 0,
        },
        .flags = {//屏幕方向一般在这里改
            .swap_xy = 0,
            .mirror_x = 0,
            .mirror_y = 0,
        },
    };
    esp_lcd_panel_io_handle_t tp_io_handle = NULL;
    const esp_lcd_panel_io_i2c_config_t tp_io_config = ESP_LCD_TOUCH_IO_I2C_CST816S_CONFIG();
    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_io_i2c((esp_lcd_i2c_bus_handle_t)EXAMPLE_TOUCH_I2C_NUM, &tp_io_config, &tp_io_handle), TAG, "");
    return esp_lcd_touch_new_i2c_cst816s(tp_io_handle, &tp_cfg, &touch_handle);
}

static esp_err_t app_lvgl_init(void)
{
    /* Initialize LVGL */
    const lvgl_port_cfg_t lvgl_cfg = {
        .task_priority = 4,         /* LVGL task priority */
        .task_stack = 7096,         /* LVGL task stack size */
        .task_affinity = -1,        /* LVGL task pinned to core (-1 is no affinity) */
        .task_max_sleep_ms = 500,   /* Maximum sleep in LVGL task */
        .timer_period_ms = 5        /* LVGL timer tick period in ms */
    };
    ESP_RETURN_ON_ERROR(lvgl_port_init(&lvgl_cfg), TAG, "LVGL port initialization failed");

    /* Add LCD screen */
    ESP_LOGD(TAG, "Add LCD screen");
    const lvgl_port_display_cfg_t disp_cfg = {
        .io_handle = lcd_io,
        .panel_handle = lcd_panel,
        .buffer_size = EXAMPLE_LCD_H_RES * EXAMPLE_LCD_DRAW_BUFF_HEIGHT,
        .double_buffer = EXAMPLE_LCD_DRAW_BUFF_DOUBLE,
        .hres = EXAMPLE_LCD_H_RES,
        .vres = EXAMPLE_LCD_V_RES,
        .monochrome = false,
#if LVGL_VERSION_MAJOR >= 9
        .color_format = LV_COLOR_FORMAT_RGB565,
#endif
        .rotation = {
            .swap_xy = false,//false,  屏幕方向一般在这里改
            .mirror_x = true,//true,
            .mirror_y = false,
        },
        .flags = {
            .buff_dma = true,
#if LVGL_VERSION_MAJOR >= 9
            .swap_bytes = true,
#endif
        }
    };
    lvgl_disp = lvgl_port_add_disp(&disp_cfg);

    /* Add touch input (for selected screen) */
    const lvgl_port_touch_cfg_t touch_cfg = {
        .disp = lvgl_disp,
        .handle = touch_handle,
    };
    lvgl_touch_indev = lvgl_port_add_touch(&touch_cfg);

    return ESP_OK;
}

// static void _app_button_cb(lv_event_t *e)
// {
//     lv_disp_rotation_t rotation = lv_disp_get_rotation(lvgl_disp);
//     rotation++;
//     if (rotation > LV_DISPLAY_ROTATION_270) {
//         rotation = LV_DISPLAY_ROTATION_0;
//     }

//     /* LCD HW rotation */
//     lv_disp_set_rotation(lvgl_disp, rotation); 
// }

// static void app_main_display(void)
// {
//     // lv_obj_t *scr = lv_scr_act();

//     /* Task lock */
//     lvgl_port_lock(0);

//     lv_demo_stress();

    /* Your LVGL objects code here .... */

    /* Create image */
//     lv_obj_t *img_logo = lv_img_create(scr);
//     lv_img_set_src(img_logo, &esp_logo);
//     lv_obj_align(img_logo, LV_ALIGN_TOP_MID, 0, 20);

//     /* Label */
//     lv_obj_t *label = lv_label_create(scr);
//     lv_obj_set_width(label, EXAMPLE_LCD_H_RES);
//     lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
// #if LVGL_VERSION_MAJOR == 8
//     lv_label_set_recolor(label, true);
//     lv_label_set_text(label, "#FF0000 "LV_SYMBOL_BELL" Hello world Espressif and LVGL "LV_SYMBOL_BELL"#\n#FF9400 "LV_SYMBOL_WARNING" For simplier initialization, use BSP "LV_SYMBOL_WARNING" #");
// #else
//     lv_label_set_text(label, LV_SYMBOL_BELL" Hello world Espressif and LVGL "LV_SYMBOL_BELL"\n "LV_SYMBOL_WARNING" For simplier initialization, use BSP "LV_SYMBOL_WARNING);
// #endif
//     lv_obj_align(label, LV_ALIGN_CENTER, 0, 20);

//     /* Button */
//     lv_obj_t *btn = lv_btn_create(scr);
//     label = lv_label_create(btn);
//     lv_label_set_text_static(label, "Rotate screen");
//     lv_obj_align(btn, LV_ALIGN_BOTTOM_MID, 0, -30);
//     lv_obj_add_event_cb(btn, _app_button_cb, LV_EVENT_CLICKED, NULL);

//     /* Task unlock */
//     lvgl_port_unlock();
// }

// void touch_task(void *pvParameter)
// {
//     uint16_t x, y; // 用于保存触摸点的坐标
//     uint8_t point_num = 0; // 用于保存触摸点的数量

//      while (1) {
//         if (touch_handle != NULL) {
//             ESP_LOGI("TOUCH", "Checking touch...");
//             if (touch_chsc6540_get_xy(touch_handle, &x, &y, &point_num)) {
//                 ESP_LOGI("TOUCH", "Touch detected at: X=%d, Y=%d", x, y);
//             } else {
//                 ESP_LOGI("TOUCH", "No touch detected.");
//             }
//         }

//         vTaskDelay(pdMS_TO_TICKS(100));
//     }
// }

void app_main(void)
{
    /* LCD HW initialization */
    ESP_ERROR_CHECK(app_lcd_init());
    ESP_ERROR_CHECK(app_touch_init());
    /* Touch initialization */
    // ESP_ERROR_CHECK(app_touch_init());
    // CHSC6540_init();
    // uint16_t x, y;
    // while (1) {
    //     if (CHSC6540_Scan(&x, &y)) {
    //         printf("Touch detected at: (%d, %d)\n", x, y);
    //     }
    //     vTaskDelay(100 / portTICK_PERIOD_MS);
    // }

    /* LVGL initialization */
    ESP_ERROR_CHECK(app_lvgl_init());

    // gpio_set_level(EXAMPLE_LCD_GPIO_BL, 0);
    /* Show LVGL objects */
    // app_main_display();
    lvgl_port_lock(0);
    // lv_demo_stress();
    lv_demo_widgets();
    // lv_demo_music();


    //  lv_obj_t * label;

    //     lv_obj_t * btn1 = lv_btn_create(lv_scr_act());
    //     lv_obj_align(btn1, LV_ALIGN_CENTER, -100, 40);
    //     lv_obj_add_flag(btn1, LV_OBJ_FLAG_CHECKABLE);
    //     lv_obj_set_height(btn1, 100);
    //     lv_obj_set_width(btn1,100);
    //     lv_obj_set_pos(btn1, -30, -80);

    //     label = lv_label_create(btn1);
    //     lv_label_set_text(label, "Button");
    //     lv_obj_center(label);

    //     lv_obj_t * btn2 = lv_btn_create(lv_scr_act());
    //     lv_obj_align(btn2, LV_ALIGN_CENTER, -80, -40);
    //     lv_obj_add_flag(btn2, LV_OBJ_FLAG_CHECKABLE);
    //      lv_obj_set_height(btn2, 80);
    //     lv_obj_set_width(btn2,80);
    //      lv_obj_set_pos(btn2, 80, 10);

    //     label = lv_label_create(btn2);
    //     lv_label_set_text(label, "Toggle");
    //     lv_obj_center(label);
    lvgl_port_unlock();
}
