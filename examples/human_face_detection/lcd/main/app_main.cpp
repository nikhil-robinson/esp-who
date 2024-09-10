#include "who_camera.h"
#include "who_human_face_detection.hpp"
#include "who_lcd.h"
#include "VL53L0X.h"
#include "fb_gfx.h"
#include "bsp/esp-bsp.h"
#define TAG "WHO"

static QueueHandle_t xQueueAIFrame = NULL;
static QueueHandle_t xQueueLIDARFrame = NULL;
static QueueHandle_t xQueueLCDFrame = NULL;

VL53L0X vl(I2C_NUM_1);

#define RGB565_MASK_RED 0xF800
#define RGB565_MASK_GREEN 0x07E0
#define RGB565_MASK_BLUE 0x001F

static void rgb_print(camera_fb_t *fb, uint32_t color, const char *str)
{
    fb_gfx_print(fb, (fb->width - (strlen(str) * 14)) / 2, 10, color, str);
}


static int rgb_printf(camera_fb_t *fb, uint32_t color, const char *format, ...)
{
    char loc_buf[64];
    char *temp = loc_buf;
    int len;
    va_list arg;
    va_list copy;
    va_start(arg, format);
    va_copy(copy, arg);
    len = vsnprintf(loc_buf, sizeof(loc_buf), format, arg);
    va_end(copy);
    if (len >= sizeof(loc_buf))
    {
        temp = (char *)malloc(len + 1);
        if (temp == NULL)
        {
            return 0;
        }
    }
    vsnprintf(temp, len + 1, format, arg);
    va_end(arg);
    rgb_print(fb, color, temp);
    if (len > 64)
    {
        free(temp);
    }
    return len;
}


void lidar_task(void *pvParameters)
{
    camera_fb_t *frame = NULL;
    if (!vl.init())
    {
        ESP_LOGE(TAG, "Failed to initialize VL53L0X :(");
        // vTaskDelay(portMAX_DELAY);
    }
    while (1)
    {

        xQueueReceive(xQueueLIDARFrame, &frame, portMAX_DELAY);
        /* measurement */
        uint16_t result_mm = 0;
        TickType_t tick_start = xTaskGetTickCount();
        bool res = vl.read(&result_mm);
        TickType_t tick_end = xTaskGetTickCount();
        int took_ms = ((int)tick_end - tick_start) / portTICK_PERIOD_MS;
        if (res)
        {
            ESP_LOGI(TAG, "Range: %d [mm] took %d [ms]", (int)result_mm, took_ms);
            rgb_printf(frame, RGB565_MASK_GREEN, "Range: %d mm", (int)result_mm);
        }
        else
        {
            ESP_LOGE(TAG, "Failed to measure :(");
            rgb_printf(frame, RGB565_MASK_RED, "Out of range", (int)result_mm);
        }

        xQueueSend(xQueueLCDFrame, &frame, portMAX_DELAY);
    }
}

extern "C" void app_main()
{
    xQueueAIFrame = xQueueCreate(2, sizeof(camera_fb_t *));
    xQueueLCDFrame = xQueueCreate(2, sizeof(camera_fb_t *));
    xQueueLIDARFrame = xQueueCreate(2, sizeof(camera_fb_t *));

    bsp_sdcard_mount();

    register_camera(PIXFORMAT_RGB565, FRAMESIZE_240X240, 2, xQueueAIFrame);
    register_human_face_detection(xQueueAIFrame, NULL, NULL, xQueueLIDARFrame, false);
    xTaskCreatePinnedToCore(lidar_task, TAG, 4 * 1024, NULL, 5, NULL, 1);
    register_lcd(xQueueLCDFrame, NULL, true);

    esp_rom_gpio_pad_select_gpio(GPIO_NUM_46);
    gpio_set_direction(GPIO_NUM_46, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_NUM_46, 1);

    sensor_t *s = esp_camera_sensor_get();

    s->set_vflip(s, 0); // flip it back
    s->set_hmirror(s, 1);


    
}
