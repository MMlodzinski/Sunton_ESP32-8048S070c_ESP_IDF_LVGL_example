// This project was created using Espressif example (rgb_panel).
// UI for this project was created with SquareLine Studio.
// It was tested with ESP-IDF v5.1

#include "Sunton_ESP32-8048S070c_board.hpp"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_rgb.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "i2c_cxx.hpp"
#include "lvgl.h"
#include "sdkconfig.h"
#include <stdio.h>
extern "C" {

static const char *TAG = "Sunton_ESP32-8048S070c_example";

#define LVGL_TICK_PERIOD_MS 2

// we use two semaphores to sync the VSYNC event and the LVGL task, to avoid
// potential tearing effect
#if CONFIG_EXAMPLE_AVOID_TEAR_EFFECT_WITH_SEM
SemaphoreHandle_t sem_vsync_end;
SemaphoreHandle_t sem_gui_ready;
#endif

extern void ui_init(lv_disp_t *disp);

static void example_lvgl_flush_cb(lv_disp_drv_t *drv, const lv_area_t *area,
                                  lv_color_t *color_map) {
  esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t)drv->user_data;
  int offsetx1 = area->x1;
  int offsetx2 = area->x2;
  int offsety1 = area->y1;
  int offsety2 = area->y2;
#if CONFIG_EXAMPLE_AVOID_TEAR_EFFECT_WITH_SEM
  xSemaphoreGive(sem_gui_ready);
  xSemaphoreTake(sem_vsync_end, portMAX_DELAY);
#endif
  // pass the draw buffer to the driver
  esp_lcd_panel_draw_bitmap(panel_handle, offsetx1, offsety1, offsetx2 + 1,
                            offsety2 + 1, color_map);
  lv_disp_flush_ready(drv);
}

#if CONFIG_EXAMPLE_LCD_TOUCH_ENABLED
static void example_lvgl_touch_cb(lv_indev_drv_t *drv, lv_indev_data_t *data) {
  uint16_t touchpad_x[1] = {0};
  uint16_t touchpad_y[1] = {0};
  uint8_t touchpad_cnt = 0;

  /* Read touch controller data */
  esp_lcd_touch_read_data((esp_lcd_touch_handle_t)drv->user_data);

  /* Get coordinates */
  bool touchpad_pressed = esp_lcd_touch_get_coordinates(
      (esp_lcd_touch_handle_t)drv->user_data, touchpad_x, touchpad_y, NULL,
      &touchpad_cnt, 1);

  if (touchpad_pressed && touchpad_cnt > 0) {
    data->point.x = touchpad_x[0];
    data->point.y = touchpad_y[0];
    data->state = LV_INDEV_STATE_PRESSED;
  } else {
    data->state = LV_INDEV_STATE_RELEASED;
  }
}
#endif

static void example_increase_lvgl_tick(void *arg) {
  /* Tell LVGL how many milliseconds has elapsed */
  lv_tick_inc(LVGL_TICK_PERIOD_MS);
}

extern "C" void app_main(void) {
  ESP_LOGI(TAG, "Register event callbacks");
  Lcd screen;
  Touch touchpad;
  ESP_LOGI(TAG, "buffers init");
  static lv_disp_draw_buf_t
      disp_buf; // contains internal graphic buffer(s) called draw buffer(s)
  static lv_disp_drv_t disp_drv; // contains callback functions

  ESP_LOGI(TAG, "screen init");
  screen.init(&disp_drv);

#if CONFIG_EXAMPLE_AVOID_TEAR_EFFECT_WITH_SEM
  ESP_LOGI(TAG, "Create semaphores");
  sem_vsync_end = xSemaphoreCreateBinary();
  assert(sem_vsync_end);
  sem_gui_ready = xSemaphoreCreateBinary();
  assert(sem_gui_ready);
#endif

  ESP_LOGI(TAG, "Initialize LVGL library");
  lv_init();
  void *buf1 = NULL;
  void *buf2 = NULL;
#if CONFIG_EXAMPLE_DOUBLE_FB
  ESP_LOGI(TAG, "Use frame buffers as LVGL draw buffers");
  ESP_ERROR_CHECK(
      esp_lcd_rgb_panel_get_frame_buffer(panel_handle, 2, &buf1, &buf2));
  // initialize LVGL draw buffers
  lv_disp_draw_buf_init(&disp_buf, buf1, buf2, 800 * 480);
#else
  ESP_LOGI(TAG, "Allocate separate LVGL draw buffers from PSRAM");
  buf1 = heap_caps_malloc(800 * 100 * sizeof(lv_color_t), MALLOC_CAP_SPIRAM);
  assert(buf1);
  buf2 = heap_caps_malloc(480 * 100 * sizeof(lv_color_t), MALLOC_CAP_SPIRAM);
  assert(buf2);
  // initialize LVGL draw buffers
  lv_disp_draw_buf_init(&disp_buf, buf1, buf2, 800 * 100);
#endif // CONFIG_EXAMPLE_DOUBLE_FB

  ESP_LOGI(TAG, "Register display driver to LVGL");
  lv_disp_drv_init(&disp_drv);
  disp_drv.hor_res = 800;
  disp_drv.ver_res = 480;
  disp_drv.flush_cb = example_lvgl_flush_cb;
  disp_drv.draw_buf = &disp_buf;
  disp_drv.user_data = *(screen.getPanelHandle());
#if CONFIG_EXAMPLE_DOUBLE_FB
  disp_drv.full_refresh = true; // the full_refresh mode can maintain the
                                // synchronization between the two frame
                                // buffers
#endif
  lv_disp_t *disp = lv_disp_drv_register(&disp_drv);

  ESP_LOGI(TAG, "Install LVGL tick timer");
  // Tick interface for LVGL (using esp_timer to generate 2ms periodic event)
  const esp_timer_create_args_t lvgl_tick_timer_args = {
      .callback = &example_increase_lvgl_tick, .name = "lvgl_tick"};
  esp_timer_handle_t lvgl_tick_timer = NULL;
  ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer));
  ESP_ERROR_CHECK(
      esp_timer_start_periodic(lvgl_tick_timer, LVGL_TICK_PERIOD_MS * 1000));

#if CONFIG_EXAMPLE_LCD_TOUCH_ENABLED
  static lv_indev_drv_t indev_drv; // Input device driver (Touch)
  lv_indev_drv_init(&indev_drv);
  indev_drv.type = LV_INDEV_TYPE_POINTER;
  indev_drv.disp = disp;
  indev_drv.read_cb = example_lvgl_touch_cb;
  indev_drv.user_data = *(touchpad.getTouchHandler());

  lv_indev_drv_register(&indev_drv);
#endif

  ESP_LOGI(TAG, "Display simple buttons example");
  ui_init(disp);

  while (1) {
    // raise the task priority of LVGL and/or reduce the handler period can
    // improve the performance
    vTaskDelay(pdMS_TO_TICKS(10));
    // The task running lv_timer_handler should have lower priority than
    // that
    // running `lv_tick_inc`
    lv_timer_handler();
  }
}
}