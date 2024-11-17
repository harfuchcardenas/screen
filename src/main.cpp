#include "TFT_eSPI/TFT_eSPI.h"/home/aharfuch/Dokumente/PlatformIO/Projects/screen/lib/lvgl/src/drivers/display/tft_espi/lv_tft_espi.h
#include <Arduino.h>
#include <lvgl.h>
#include <../lv_conf.h>
#include <examples/get_started/lv_example_get_started.h>

// put function declarations here:
TFT_eSPI tft = TFT_eSPI(); /* TFT instance */
static lv_disp_buf_t disp_buf;
static lv_color_t buf[LV_HOR_RES_MAX * 10];

#if USE_LV_LOG != 0
/* Serial debugging */
void my_print(lv_log_level_t level, const char *file, uint32_t line, const char *dsc)
{
  Serial.printf("%s@%d->%s\r\n", file, line, dsc);
  Serial.flush();
}
#endif

/* Display flushing */
void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p)
{
  uint32_t w = (area->x2 - area->x1 + 1);
  uint32_t h = (area->y2 - area->y1 + 1);
  tft.startWrite();
  tft.setAddrWindow(area->x1, area->y1, w, h);
  tft.pushColors(&color_p->full, w * h, true);
  tft.endWrite();
  lv_disp_flush_ready(disp);
}

void setup() {
  // put your setup code here, to run once:
  lv_tick_inc(10);
  lv_init();
  lv_example_get_started_1();
}

void loop() {
  // put your main code here, to run repeatedly:
}

// put function definitions here:
