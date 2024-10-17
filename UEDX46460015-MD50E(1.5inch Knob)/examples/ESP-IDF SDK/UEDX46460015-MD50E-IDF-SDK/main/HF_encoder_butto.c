// #include <stdio.h>

// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "driver/gpio.h"
// #include "driver/i2c.h"
// #include "driver/spi_master.h"
// #include "esp_timer.h"
// #include "esp_lcd_panel_io.h"
// #include "esp_lcd_panel_vendor.h"
// #include "esp_lcd_panel_ops.h"
// #include "esp_err.h"
// #include "esp_log.h"

// #include "lvgl.h"
// #include "lv_demos.h"
// #include "esp_lcd_sh8601.h"
// #include "esp_lcd_touch_ft5x06.h"
// #include "iot_knob.h"

// static const char *TAG = "bsp";
// /* Assert on error, if selected in menuconfig. Otherwise return error code. */
// #define CONFIG_BSP_ERROR_CHECK 1
// #if CONFIG_BSP_ERROR_CHECK
// #define BSP_ERROR_CHECK_RETURN_ERR(x)    ESP_ERROR_CHECK(x)
// #define BSP_ERROR_CHECK_RETURN_NULL(x)   ESP_ERROR_CHECK(x)
// #define BSP_NULL_CHECK(x, ret)           assert(x)
// #define BSP_NULL_CHECK_GOTO(x, goto_tag) assert(x)
// #else
// #define BSP_ERROR_CHECK_RETURN_ERR(x) do { 
//         esp_err_t err_rc_ = (x);            
//         if (unlikely(err_rc_ != ESP_OK)) {  
//             return err_rc_;                 
//         }                                   
//     } while(0)

// #define BSP_ERROR_CHECK_RETURN_NULL(x)  do { 
//         if (unlikely((x) != ESP_OK)) {      
//             return NULL;                    
//         }                                   
//     } while(0)

// #define BSP_NULL_CHECK(x, ret) do { 
//         if ((x) == NULL) {          
//             return ret;             
//         }                           
//     } while(0)

// #define BSP_NULL_CHECK_GOTO(x, goto_tag) do { 
//         if ((x) == NULL) {      
//             goto goto_tag;      
//         }                       
//     } while(0)
// #endif
// /* Buttons */
// typedef enum {
//     BSP_BTN_PRESS = GPIO_NUM_0,
// } bsp_button_t;

// #define BSP_ENCODER_A         (GPIO_NUM_6)
// #define BSP_ENCODER_B         (GPIO_NUM_5)


// #define CONFIG_BSP_DISPLAY_LVGL_TICK 5
// /* LVGL related parameters */
// #define LVGL_TICK_PERIOD_MS         (CONFIG_BSP_DISPLAY_LVGL_TICK)
// #define LVGL_BUFFER_HEIGHT          (CONFIG_BSP_DISPLAY_LVGL_BUF_HEIGHT)


// esp_err_t bsp_button_init(const bsp_button_t btn)
// {
//     const gpio_config_t button_io_config = {
//         .pin_bit_mask = BIT64(btn),
//         .mode = GPIO_MODE_INPUT,
//         .pull_up_en = GPIO_PULLUP_ENABLE,
//         .pull_down_en = GPIO_PULLDOWN_DISABLE,
//         .intr_type = GPIO_INTR_DISABLE
//     };
//     return gpio_config(&button_io_config);
// }

// bool bsp_button_get(const bsp_button_t btn)
// {
//     return !(bool)gpio_get_level(btn);
// }
// static void encoder_read(lv_indev_drv_t *indev_drv, lv_indev_data_t *data)
// {
//     static int32_t last_v = 0;

//     assert(indev_drv);
//     assert(indev_drv->user_data);
//     knob_handle_t knob_handle = (knob_handle_t)indev_drv->user_data;

//     int32_t invd = iot_knob_get_count_value(knob_handle);//获取旋转编码器的当前值
//     knob_event_t event = iot_knob_get_event(knob_handle);//获取旋转编码器的当前事件（如旋转方向）

//     if (last_v ^ invd) {
//         last_v = invd;
//         data->enc_diff = (KNOB_RIGHT == event) ? (-1) : ((KNOB_LEFT == event) ? (1) : (0));
//     } else {
//         data->enc_diff = 0;
//     }
//     if(data->enc_diff==-1)
//     {
//        ESP_LOGI(TAG, "uichangge T");     
//     }
//     else if(data->enc_diff==1)
//     {
//        ESP_LOGI(TAG, "uichangge R");

//     }
//     data->state = (true == bsp_button_get(BSP_BTN_PRESS)) ? LV_INDEV_STATE_PRESSED : LV_INDEV_STATE_RELEASED;
// }

//  esp_err_t lvgl_port_indev_init(void)
// {
//     static lv_indev_drv_t indev_drv;//配置和注册旋转编码器输入设备。
//     lv_indev_t *indev_encoder;//定义一个指向LVGL输入设备结构体的指针，用于接收注册后的输入设备句柄。
//     knob_handle_t knob_handle;//定义一个句柄，用于表示旋转编码器的实例。

//     BSP_ERROR_CHECK_RETURN_NULL(bsp_button_init(BSP_BTN_PRESS));//按钮
    

//     //配置旋转编码器
//     knob_config_t *cfg = calloc(1, sizeof(knob_config_t));
//     cfg->default_direction = 0;
//     cfg->gpio_encoder_a = BSP_ENCODER_A;
//     cfg->gpio_encoder_b = BSP_ENCODER_B;
//     knob_handle = iot_knob_create(cfg);

//     /* Register a touchpad input device */ //将编码器和按钮配置在LVGL里面
//     lv_indev_drv_init(&indev_drv);
//     indev_drv.type = LV_INDEV_TYPE_ENCODER;
//     indev_drv.read_cb = encoder_read;//回调函数
//     indev_drv.user_data = knob_handle;
//     indev_encoder = lv_indev_drv_register(&indev_drv);
//     BSP_NULL_CHECK(indev_encoder, ESP_ERR_NO_MEM);

//     return ESP_OK;
// }
// static void lvgl_port_tick_increment(void *arg)
// {
//     /* Tell LVGL how many milliseconds have elapsed */
//     lv_tick_inc(LVGL_TICK_PERIOD_MS);
// }

//  esp_err_t lvgl_port_tick_init(void)
// {
//     // Tick interface for LVGL (using esp_timer to generate 2ms periodic event)
//     const esp_timer_create_args_t lvgl_tick_timer_args = {
//         .callback = &lvgl_port_tick_increment,
//         .name = "LVGL tick"
//     };
//     esp_timer_handle_t lvgl_tick_timer = NULL;
//     BSP_ERROR_CHECK_RETURN_ERR(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer));
//     return esp_timer_start_periodic(lvgl_tick_timer, LVGL_TICK_PERIOD_MS * 1000);
// }
// // lv_disp_t *bsp_display_start(void)
// // {
// //     //lv_init();
// //     //lv_disp_t *disp = lvgl_port_display_init();
// //     //BSP_NULL_CHECK(disp, NULL);
// //     BSP_ERROR_CHECK_RETURN_NULL(lvgl_port_tick_init());
// //     BSP_ERROR_CHECK_RETURN_NULL(lvgl_port_indev_init());//初始化输入设备

// //     // lvgl_mux = xSemaphoreCreateRecursiveMutex();
// //     // BSP_NULL_CHECK(lvgl_mux, NULL);
// //     // xTaskCreate(
// //     //     lvgl_port_task, "LVGL task", CONFIG_BSP_DISPLAY_LVGL_TASK_STACK_SIZE * 1024, NULL,
// //     //     CONFIG_BSP_DISPLAY_LVGL_TASK_PRIORITY, &lvgl_task_handle
// //     // );
// // #if CONFIG_BSP_DISPLAY_LVGL_AVOID_TEAR
// //     bsp_lcd_register_trans_done_callback(lvgl_port_lcd_trans_done);
// // #endif

// //     return disp;
// // }

// // void bsp_display_rotate(lv_disp_t *disp, lv_disp_rot_t rotation)
// // {
// //     lv_disp_set_rotation(disp, rotation);
// // }

// // bool bsp_display_lock(uint32_t timeout_ms)
// // {
// //     assert(lvgl_mux && "bsp_display_start must be called first");

// //     const TickType_t timeout_ticks = (timeout_ms == 0) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
// //     return xSemaphoreTake(lvgl_mux, timeout_ticks) == pdTRUE;
// // }

// // void bsp_display_unlock(void)
// // {
// //     assert(lvgl_mux && "bsp_display_start must be called first");
// //     xSemaphoreGive(lvgl_mux);
// // }

// /**************************************************************************************************
//  *
//  * Button Funciton
//  *
//  **************************************************************************************************/


//上面是搬官方的//
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
#include "esp_err.h"
#include "esp_log.h"

#include "lvgl.h"
#include "lv_demos.h"
#include "esp_lcd_sh8601.h"


static const char *TAG = "encoder_button";
//********************************************************************** */
#define HF_encoder_button  1  //使用编码器和按钮的宏开关
#if HF_encoder_button
typedef void (* knob_cb_t)(void *, void *);
typedef void *knob_handle_t;
typedef enum {
    KNOB_LEFT = 0,                     /*!< EVENT: Rotate to the left */
    KNOB_RIGHT,                        /*!< EVENT: Rotate to the right */
    KNOB_H_LIM,                        /*!< EVENT: Count reaches maximum limit */
    KNOB_L_LIM,                        /*!< EVENT: Count reaches the minimum limit */
    KNOB_ZERO,                         /*!< EVENT: Count back to 0 */
    KNOB_EVENT_MAX,                    /*!< EVENT: Number of events */
} knob_event_t;

/**
 * @brief Knob config
 * 
 */
#define KNOB_CHECK(a, str, ret_val)                          \
    if (!(a))                                                     \
    {                                                             \
        ESP_LOGE(TAG, "%s(%d): %s", __FUNCTION__, __LINE__, str); \
        return (ret_val);                                         \
    }

#define KNOB_CHECK_GOTO(a, str, label) if(!(a)) { \
        ESP_LOGE(TAG,"%s:%d (%s):%s", __FILE__, __LINE__, __FUNCTION__, str); \
        goto label; \
    }

#define CALL_EVENT_CB(ev)   if(knob->cb[ev])knob->cb[ev](knob, knob->usr_data[ev])

typedef struct {
    uint8_t default_direction;          /*!< 0:positive increase   1:negative increase */
    uint8_t gpio_encoder_a;             /*!< Encoder Pin A */
    uint8_t gpio_encoder_b;             /*!< Encoder Pin B */
} knob_config_t;
typedef enum {
    KNOB_READY = 0,                     /*!< Knob state: ready*/
    KNOB_PHASE_A,                       /*!< Knob state: phase A arrives first */
    KNOB_PHASE_B,                       /*!< Knob state: phase B arrives first */
} knob_state_t;

typedef struct Knob {
    bool          encoder_a_change;                            /*<! true means Encoder A phase Inverted*/
    bool          encoder_b_change;                            /*<! true means Encoder B phase Inverted*/
    uint8_t       default_direction;                           /*!< 0:positive increase   1:negative increase */
    knob_state_t  state;                                       /*!< knob state machine status */
    uint8_t       debounce_a_cnt;                              /*!< Encoder A phase debounce count */
    uint8_t       debounce_b_cnt;                              /*!< Encoder B phase debounce count */
    uint8_t       encoder_a_level;                             /*!< Encoder A phase current level */
    uint8_t       encoder_b_level;                             /*!< Encoder B phase current Level */
    knob_event_t  event;                                       /*!< Current event */
    uint16_t      ticks;                                       /*!< Timer interrupt count */
    int           count_value;                                 /*!< Knob count */
    uint8_t       (*hal_knob_level)(void *hardware_data);      /*!< Get current level */
    void          *encoder_a;                                  /*!< Encoder A phase gpio number */
    void          *encoder_b;                                  /*!< Encoder B phase gpio number */
    void          *usr_data[KNOB_EVENT_MAX];                   /*!< User data for event */
    knob_cb_t     cb[KNOB_EVENT_MAX];                          /*!< Event callback */
    struct Knob   *next;                                       /*!< Next pointer */
} knob_dev_t;

static knob_dev_t *s_head_handle = NULL;
static esp_timer_handle_t s_knob_timer_handle;
static bool s_is_timer_running = false;

#define CONFIG_KNOB_PERIOD_TIME_MS 3
#define CONFIG_KNOB_HIGH_LIMIT 1000
#define CONFIG_KNOB_LOW_LIMIT -1000
#define CONFIG_KNOB_DEBOUNCE_TICKS 2
#define TICKS_INTERVAL    CONFIG_KNOB_PERIOD_TIME_MS
#define DEBOUNCE_TICKS    CONFIG_KNOB_DEBOUNCE_TICKS
#define HIGH_LIMIT        CONFIG_KNOB_HIGH_LIMIT
#define LOW_LIMIT         CONFIG_KNOB_LOW_LIMIT


#define CONFIG_BSP_ERROR_CHECK 1
#if CONFIG_BSP_ERROR_CHECK
#define BSP_ERROR_CHECK_RETURN_ERR(x)    ESP_ERROR_CHECK(x)
#define BSP_ERROR_CHECK_RETURN_NULL(x)   ESP_ERROR_CHECK(x)
#define BSP_NULL_CHECK(x, ret)           assert(x)
#define BSP_NULL_CHECK_GOTO(x, goto_tag) assert(x)
#endif
/* Buttons */
typedef enum {
    BSP_BTN_PRESS = GPIO_NUM_0,
} bsp_button_t;

#define BSP_ENCODER_A         (GPIO_NUM_6)
#define BSP_ENCODER_B         (GPIO_NUM_5)


esp_err_t bsp_button_init(const bsp_button_t btn)
{
    const gpio_config_t button_io_config = {
        .pin_bit_mask = BIT64(btn),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    return gpio_config(&button_io_config);
}
knob_event_t iot_knob_get_event(knob_handle_t knob_handle)
{
    KNOB_CHECK(NULL != knob_handle, "Pointer of handle is invalid", ESP_ERR_INVALID_ARG);
    knob_dev_t *knob = (knob_dev_t *) knob_handle;
    return knob->event;
}

int iot_knob_get_count_value(knob_handle_t knob_handle)
{
    KNOB_CHECK(NULL != knob_handle, "Pointer of handle is invalid", ESP_ERR_INVALID_ARG);
    knob_dev_t *knob = (knob_dev_t *) knob_handle;
    return knob->count_value;
}

esp_err_t iot_knob_clear_count_value(knob_handle_t knob_handle)
{
    KNOB_CHECK(NULL != knob_handle, "Pointer of handle is invalid", ESP_ERR_INVALID_ARG);
    knob_dev_t *knob = (knob_dev_t *) knob_handle;
    knob->count_value = 0;
    return ESP_OK;
}
bool bsp_button_get(const bsp_button_t btn)
{
    return !(bool)gpio_get_level(btn);
}
static uint8_t _knob_gpio_get_key_level(void *gpio_num)
{
    return (uint8_t)gpio_get_level((uint32_t)gpio_num);
}
static esp_err_t _knob_gpio_init(uint8_t gpio_num)
{
    gpio_config_t gpio_cfg = {
        .pin_bit_mask = (1ULL << gpio_num),
        .mode = GPIO_MODE_INPUT,
        .intr_type = GPIO_INTR_ANYEDGE,
        .pull_up_en = 1,
    };
    esp_err_t ret = gpio_config(&gpio_cfg);

    return ret;
}
static esp_err_t _knob_gpio_deinit(int gpio_num)
{
    /** both disable pullup and pulldown */
    gpio_config_t gpio_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << gpio_num),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
    };
    gpio_config(&gpio_conf);
    return ESP_OK;
}

static void knob_handler(knob_dev_t *knob)
{
    uint8_t pha_value = knob->hal_knob_level(knob->encoder_a);
    uint8_t phb_value = knob->hal_knob_level(knob->encoder_b);

    if ((knob->state) > 0) {
        knob->ticks++;
    }

    if (pha_value != knob->encoder_a_level) {
        if (++(knob->debounce_a_cnt) >= DEBOUNCE_TICKS) {
            knob->encoder_a_change = true;
            knob->encoder_a_level = pha_value;
            knob->debounce_a_cnt = 0;
        }
    } else {
        knob->debounce_a_cnt = 0;
    }

    if (phb_value != knob->encoder_b_level) {
        if (++(knob->debounce_b_cnt) >= DEBOUNCE_TICKS) {
            knob->encoder_b_change = true;
            knob->encoder_b_level = phb_value;
            knob->debounce_b_cnt = 0;
        }
    } else {
        knob->debounce_b_cnt = 0;
    }

    switch (knob->state) {
    case KNOB_READY:
        if (knob->encoder_a_change) {
            knob->encoder_a_change = false;
            knob->ticks = 0;
            knob->state = KNOB_PHASE_A;
        } else if ( knob->encoder_b_change) {
            knob->encoder_b_change = false;
            knob->ticks = 0;
            knob->state = KNOB_PHASE_B;
        }
        break;

    case KNOB_PHASE_A:
        if (knob->encoder_b_change) {
            knob->encoder_b_change = false;
            if (knob->default_direction) {
                knob->count_value--;
                knob->event = KNOB_LEFT;
                CALL_EVENT_CB(KNOB_LEFT);
                if (knob->count_value <= LOW_LIMIT) {
                    knob->event = KNOB_L_LIM;
                    CALL_EVENT_CB(KNOB_L_LIM);
                    knob->count_value = 0;
                } else if (knob->count_value == 0) {
                    knob->event = KNOB_ZERO;
                    CALL_EVENT_CB(KNOB_ZERO);
                }
            } else {
                knob->count_value++;
                knob->event = KNOB_RIGHT;
                CALL_EVENT_CB(KNOB_RIGHT);
                if (knob->count_value >= HIGH_LIMIT) {
                    knob->event = KNOB_H_LIM;
                    CALL_EVENT_CB(KNOB_H_LIM);
                    knob->count_value = 0;
                } else if (knob->count_value == 0) {
                    knob->event = KNOB_ZERO;
                    CALL_EVENT_CB(KNOB_ZERO);
                }
            }
            knob->ticks = 0;
            knob->state = KNOB_READY;
        } else if (knob->encoder_a_change) {
            knob->encoder_a_change = false;
            knob->ticks = 0;
            knob->state = KNOB_READY;
        }
        break;

    case KNOB_PHASE_B:
        if (knob->encoder_a_change) {
            knob->encoder_a_change = false;
            if (knob->default_direction) {
                knob->count_value++;
                knob->event = KNOB_RIGHT;
                CALL_EVENT_CB(KNOB_RIGHT);
                if (knob->count_value >= HIGH_LIMIT) {
                    knob->event = KNOB_H_LIM;
                    CALL_EVENT_CB(KNOB_H_LIM);
                    knob->count_value = 0;
                } else if (knob->count_value == 0) {
                    knob->event = KNOB_ZERO;
                    CALL_EVENT_CB(KNOB_ZERO);
                }
            } else {
                knob->count_value--;
                knob->event = KNOB_LEFT;
                CALL_EVENT_CB(KNOB_LEFT);
                if (knob->count_value <= LOW_LIMIT) {
                    knob->event = KNOB_L_LIM;
                    CALL_EVENT_CB(KNOB_L_LIM);
                    knob->count_value = 0;
                } else if (knob->count_value == 0) {
                    knob->event = KNOB_ZERO;
                    CALL_EVENT_CB(KNOB_ZERO);
                }
            }
            knob->ticks = 0;
            knob->state = KNOB_READY;
        } else if (knob->encoder_b_change) {
            knob->encoder_b_change = false;
            knob->ticks = 0;
            knob->state = KNOB_READY;
        }
        break;
    }
}

static void knob_cb(void *args)
{
    knob_dev_t *target;
    for (target = s_head_handle; target; target = target->next) {
        knob_handler(target);
    }
}

knob_handle_t iot_knob_create(const knob_config_t *config)
{
    KNOB_CHECK(NULL != config, "config pointer can't be NULL!", NULL)
    KNOB_CHECK(config->gpio_encoder_a != config->gpio_encoder_b, "encoder A can't be the same as encoder B", NULL);
    esp_err_t ret = ESP_OK;
    ret = _knob_gpio_init(config->gpio_encoder_a);
    KNOB_CHECK(ESP_OK == ret, "encoder A gpio init failed", NULL);
    ret = _knob_gpio_init(config->gpio_encoder_b);
    KNOB_CHECK_GOTO(ESP_OK == ret, "encoder B gpio init failed", _encoder_a_deinit);

    knob_dev_t *knob = (knob_dev_t *) calloc(1, sizeof(knob_dev_t));
    KNOB_CHECK_GOTO(NULL != knob, "alloc knob failed", _encoder_b_deinit);
    knob->default_direction = config->default_direction;
    knob->hal_knob_level = _knob_gpio_get_key_level;
    knob->encoder_a = (void *)(long)config->gpio_encoder_a;
    knob->encoder_b = (void *)(long)config->gpio_encoder_b;

    knob->encoder_a_level = knob->hal_knob_level(knob->encoder_a);
    knob->encoder_b_level = knob->hal_knob_level(knob->encoder_b);

    knob->next = s_head_handle;
    s_head_handle = knob;

    if (false == s_is_timer_running) {
        esp_timer_create_args_t knob_timer;
        knob_timer.arg = NULL;
        knob_timer.callback = knob_cb;
        knob_timer.dispatch_method = ESP_TIMER_TASK;
        knob_timer.name = "knob_timer";
        esp_timer_create(&knob_timer, &s_knob_timer_handle);
        esp_timer_start_periodic(s_knob_timer_handle, TICKS_INTERVAL * 1000U);
        s_is_timer_running = true;
    }

    //ESP_LOGI(TAG, "Iot Knob Config Succeed, encoder A:%d, encoder B:%d, direction:%d, Version: %d.%d.%d",config->gpio_encoder_a, config->gpio_encoder_b, config->default_direction, KNOB_VER_MAJOR, KNOB_VER_MINOR, KNOB_VER_PATCH);
    return (knob_handle_t)knob;

_encoder_b_deinit:
    _knob_gpio_deinit(config->gpio_encoder_b);
_encoder_a_deinit:
    _knob_gpio_deinit(config->gpio_encoder_a);
    return NULL;
}
volatile int HF_botton=0;
volatile int HF_botton_time=0;
volatile int HF_encoder_num=0;
 void encoder_read(lv_indev_drv_t *indev_drv, lv_indev_data_t *data)
{
    static int32_t last_v = 0;

    assert(indev_drv);
    assert(indev_drv->user_data);
    knob_handle_t knob_handle = (knob_handle_t)indev_drv->user_data;

    int32_t invd = iot_knob_get_count_value(knob_handle);//获取旋转编码器的当前值
    knob_event_t event = iot_knob_get_event(knob_handle);//获取旋转编码器的当前事件（如旋转方向）

    if (last_v ^ invd) {
        last_v = invd;
        data->enc_diff = (KNOB_RIGHT == event) ? (-1) : ((KNOB_LEFT == event) ? (1) : (0));
        if(KNOB_RIGHT == event)
        {
            ESP_LOGI(TAG, "running L\n");
             HF_encoder_num=-1;
        }
       
        else
        {    
             HF_encoder_num=1;
            ESP_LOGI(TAG, "running R\n");
        }
       
    } else {
      
        data->enc_diff = 0;
    }
    data->state = (true == bsp_button_get(BSP_BTN_PRESS)) ? LV_INDEV_STATE_PRESSED : LV_INDEV_STATE_RELEASED;
    if( data->state ==LV_INDEV_STATE_PRESSED)
    {
       
        vTaskDelay(pdMS_TO_TICKS(20));
        if( data->state ==LV_INDEV_STATE_PRESSED)
        {
             if(HF_botton!=-1)
                HF_botton=1;
        }
    }
    else if(data->state ==LV_INDEV_STATE_RELEASED)//抬起
    {
        if(HF_botton==1&&HF_botton!=-1)//按下
          HF_botton=2;//按下抬起
        if(HF_botton==-1)
            HF_botton=0;
       

    }
    if(HF_botton==1)
    {
        HF_botton_time++;
    }
    else
    {
        HF_botton_time=0;    
    }
}
 esp_err_t lvgl_port_indev_init(void)
{
    static lv_indev_drv_t indev_drv;//配置和注册旋转编码器输入设备。
    lv_indev_t *indev_encoder;//定义一个指向LVGL输入设备结构体的指针，用于接收注册后的输入设备句柄。
    knob_handle_t knob_handle;//定义一个句柄，用于表示旋转编码器的实例。

    bsp_button_init(BSP_BTN_PRESS);//按钮
    

    //配置旋转编码器
    knob_config_t *cfg = calloc(1, sizeof(knob_config_t));
    cfg->default_direction = 0;
    cfg->gpio_encoder_a = BSP_ENCODER_A;
    cfg->gpio_encoder_b = BSP_ENCODER_B;
    knob_handle = iot_knob_create(cfg);

    /* Register a touchpad input device */ //将编码器和按钮配置在LVGL里面
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_ENCODER;
    indev_drv.read_cb = encoder_read;//回调函数
    indev_drv.user_data = knob_handle;
    indev_encoder = lv_indev_drv_register(&indev_drv);
    BSP_NULL_CHECK(indev_encoder, ESP_ERR_NO_MEM);

    return ESP_OK;
}
#endif
// /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


/*
 * SPDX-FileCopyrightText: 2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

// #define KEYST_UP    	0
// #define KEYST_DOWN		1
// #define KEYST_KEEP		2
// #define KEYST_KEEP_1S   3
// #define KEYST_KEEP_2S   4
// #define KEYST_KEEP_3S   5
// #define KEYST_KEEP_5S   6
// typedef enum{
//     KEY_LIGHT1= 0,
// 	KEY_SWKEY,
// 	KEY_ENCODE_NEXT,
// 	KEY_ENCODE_PRE,
//     KEY_NOKEY = 0xff
// }E_KEY;
// typedef struct{
//     uint8_t keystatus;
//     uint8_t keyvalue;
// }S_INPUT_KEY;
// void HF_get_encoder()
// {

// }