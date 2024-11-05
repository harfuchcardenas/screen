/* TBD FIXME which do we really need?
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_freertos_hooks.h"
#include "esp_system.h"
#include "driver/gpio.h"
*/
#include <Arduino.h>  // TBD FIXME just for Serial, remove later
#include "freertos/FreeRTOS.h"    // TBD FIXME from ESP32-IDF example
#include "freertos/task.h"
#include "freertos/semphr.h"
// TBD FIXME see: https://circuitdigest.com/microcontroller-projects/arduino-freertos-tutorial-using-semaphore-and-mutex-in-freertos-with-arduino
//#include "<Arduino_FreeRTOS.h>"
//#include "<semphr.h>" 
#include "gui.h"
#include "gui_app.h"

#define CONFIG_LV_TFT_DISPLAY_CONTROLLER_FT81X

namespace gui {

SemaphoreHandle_t xGuiSemaphore;

//static lv_disp_draw_buf_t disp_draw_buf;  // LVGL v8.0.0 		// TBD static: hmm, when using multiple displays we need multiple buffers
static lv_disp_buf_t disp_draw_buf;	            		// TBD static: hmm, when using multiple displays we need multiple buffers
static lv_color_t* disp_draw_buf_1;
static lv_color_t* disp_draw_buf_2;

 // display draw buffer size
#if defined CONFIG_LV_TFT_DISPLAY_CONTROLLER_IL3820         \
    || defined CONFIG_LV_TFT_DISPLAY_CONTROLLER_JD79653A    \
    || defined CONFIG_LV_TFT_DISPLAY_CONTROLLER_UC8151D     \
    || defined CONFIG_LV_TFT_DISPLAY_CONTROLLER_SSD1306

    // actual size in pixels, not bytes
    constexpr uint32_t disp_draw_buf_size_in_px = DISP_BUF_SIZE * 8;
#else
    constexpr uint32_t disp_draw_buf_size_in_px = DISP_BUF_SIZE;
#endif

static void lv_tick_task(void *arg) {
    (void) arg;

    lv_tick_inc(LV_TICK_PERIOD_MS);
}

#if LV_USE_LOG != 0
void lv_log_cb(lv_log_level_t level, const char * file, uint32_t line, const char * function, const char * dsc )
{
    // TBD FIXME NOW, replace with LOGGER_x calls!
  Serial.printf("%s@%d (%s)->%s\r\n", file, line, function, dsc);
  Serial.flush();	// TBD FIXME NOW, add this to the logging app
}
#endif

static void lvgl_init()
{
    /* Initialize LVGL */
    // see: "esp32dev/lvgl/examples/porting/lv_port_disp_template.c" 
    // see: "lv_platformio/hal/stm32f429_disco/tft.c"	// TBD FIXME

//    gpio_set_direction(4, GPIO_MODE_OUTPUT);

//        gpio_set_level(4, 0);
//        //delay(6); /* minimum time for power-down is 5ms */
//        delay(60); /* minimum time for power-down is 5ms */

//        gpio_set_level(4, 1);
//        //delay(21);
//        delay(210);
//
    // init LVGL
    lv_init();

#if LV_USE_LOG != 0
    // register LVGL print callback function for logging
    lv_log_register_print_cb(lv_log_cb);
#endif

    /* Initialize Display Drawing Buffers */
    // see: esp32dev/lvgl/examples/porting/lv_port_disp_template.c 	// TBD checkout variant 3 with full display buffer 
    disp_draw_buf_1 = static_cast<lv_color_t*>(heap_caps_malloc(DISP_BUF_SIZE * sizeof(lv_color_t), MALLOC_CAP_DMA));
    assert(disp_draw_buf_1 != NULL);

#ifdef CONFIG_LV_TFT_DISPLAY_MONOCHROME
    // use only single buffer when working with monochrome displays
    disp_draw_buf_2 = NULL;		// TBD static: hmm, when using multiple displays we need multiple buffers
#else
    // use two buffer when working with color displays
    disp_draw_buf_2 = static_cast<lv_color_t*>(heap_caps_malloc(DISP_BUF_SIZE * sizeof(lv_color_t), MALLOC_CAP_DMA));
    assert(disp_draw_buf_2 != NULL);
#endif

    // initialize the working buffer depending on the selected display
    // disp_draw_buf_2 == NULL (not used) when using monochrome displays.
    //lv_disp_draw_buf_init(&disp_draw_buf, disp_draw_buf_1, disp_draw_buf_2, disp_draw_buf_size_in_px);    // LVGL v8.0.0
    lv_disp_buf_init(&disp_draw_buf, disp_draw_buf_1, disp_draw_buf_2, disp_draw_buf_size_in_px);

    /* Register Display Driver in LVGL */
    lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    // set display resolution
    disp_drv.hor_res = LV_HOR_RES_MAX;
    disp_drv.ver_res = LV_VER_RES_MAX;
    disp_drv.flush_cb = disp_driver_flush;	// TBD FIXME NOW, where is this implemented!?	// TBD FIXME replace those for the K210
    disp_drv.buffer = &disp_draw_buf;
#ifdef CONFIG_LV_TFT_DISPLAY_MONOCHROME
    // register callback functions needed for monochrome displays
    disp_drv.rounder_cb = disp_driver_rounder;	// TBD FIXME NOW, where is this implemented!?	// TBD FIXME replace those for the K210
    disp_drv.set_px_cb = disp_driver_set_px;	// TBD FIXME NOW, where is this implemented!?	// TBD FIXME replace those for the K210
#endif
#if CONFIG_LV_USE_GPU
    // fill a memory array with a color
    disp_drv.gpu_blend_cb = gpu_mem_blend;
    disp_drv.gpu_fill_cb = gpu_mem_fill;
#endif
    // register the display driver
    lv_disp_drv_register(&disp_drv);

#if CONFIG_LV_TOUCH_CONTROLLER != TOUCH_CONTROLLER_NONE
    /* Register Input Device Driver in LVGL */
    lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.read_cb = touch_driver_read;
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    lv_indev_drv_register(&indev_drv);
#endif
}

static void lvgl_finalize(void)
{
    /* LVGL clean up */
    free(disp_draw_buf_1);
#ifndef CONFIG_LV_TFT_DISPLAY_MONOCHROME
    free(disp_draw_buf_2);
#endif
}

static void GUI_task(void *pvParameter)
{
    (void) pvParameter;

    /* Create GUI Semaphore */
    xGuiSemaphore = xSemaphoreCreateMutex();

    /* Initialize the Display and Input Device Drivers (SPI, I2C Bus as needed) */
    lvgl_driver_init();		// TBD FIXME make static	// TBD FIXME replace those for the K210

    /* Initialize LVGL */
    lvgl_init();

    /* Create and start a periodic timer interrupt to call lv_tick_inc */
    const esp_timer_create_args_t periodic_timer_args =
    {
        &lv_tick_task,  // callback
        NULL,           // arg
        ESP_TIMER_TASK, // dispatch_method
        "periodic_gui"  // name
    };
    
    esp_timer_handle_t periodic_timer;
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, LV_TICK_PERIOD_MS * 1000));

    /* Create GUI */
    app::gui::create_GUI();	// TBD FIXME make this a parameter?

    while (1)
    {
        /* Delay 1 tick (assumes FreeRTOS tick is 10ms) */
        vTaskDelay(pdMS_TO_TICKS(10));

        /* Try to take the semaphore, call LVGL related function on success */
        if (pdTRUE == xSemaphoreTake(xGuiSemaphore, portMAX_DELAY)) {
            lv_task_handler();
            xSemaphoreGive(xGuiSemaphore);
       }
    }
    
    /* A task should NEVER return */
    lvgl_finalize();
    vTaskDelete(NULL);
}

void start_GUI_task(void)
{

    /* If you want to use a task to create the graphic, you NEED to create a Pinned task
     * Otherwise there can be problem such as memory corruption and so on.
     * NOTE: When not using Wi-Fi nor Bluetooth you can pin the guiTask to core 0 */
    xTaskCreatePinnedToCore(            // TBD FIXME check return value!
            GUI_task,       // task function 
            "GUI_task",     // task name
            4096 * 2,       // stack size (in words) 
            NULL,           // task input parameter
            5,              // task priority    // TBD FIXME NOW (was 1)
            NULL,           // task handle
            1);             // core
}

}