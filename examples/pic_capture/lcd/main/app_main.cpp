#include "who_camera.h"
#include "who_human_face_detection.hpp"
#include "who_lcd.h"
#include "VL53L0X.h"
#include "fb_gfx.h"

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
    while (1)
    {
        size_t _jpg_buf_len;
        uint8_t * _jpg_buf;

        xQueueReceive(xQueueLIDARFrame, &frame, portMAX_DELAY);

        bool jpeg_converted = frame2jpg(frame, 80, &_jpg_buf, &_jpg_buf_len);
        if(!jpeg_converted){
            ESP_LOGE(TAG, "JPEG compression failed");
        }
        else
        {
            /* code */
        }
        
        if (_jpg_buf != NULL)
        {
            free(_jpg_buf);
            _jpg_buf =NULL;
        }
        

        xQueueSend(xQueueLCDFrame, &frame, portMAX_DELAY);
    }
}

extern "C" void app_main()
{
    xQueueAIFrame = xQueueCreate(2, sizeof(camera_fb_t *));
    xQueueLCDFrame = xQueueCreate(2, sizeof(camera_fb_t *));
    xQueueLIDARFrame = xQueueCreate(2, sizeof(camera_fb_t *));

    register_camera(PIXFORMAT_RGB565, FRAMESIZE_240X240, 2, xQueueLIDARFrame);
    // register_human_face_detection(xQueueAIFrame, NULL, NULL, xQueueLIDARFrame, false);
    // xTaskCreatePinnedToCore(lidar_task, TAG, 4 * 1024, NULL, 5, NULL, 1);
    register_lcd(xQueueLIDARFrame, NULL, true);

    esp_rom_gpio_pad_select_gpio(GPIO_NUM_46);
    gpio_set_direction(GPIO_NUM_46, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_NUM_46, 1);

    sensor_t *s = esp_camera_sensor_get();

    s->set_vflip(s, 0); // flip it back
    s->set_hmirror(s, 1);


    
}
