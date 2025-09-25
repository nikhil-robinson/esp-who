#include "who_camera.h"
#include "who_human_face_detection.hpp"
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_camera.h"
#include "usb_device_uvc.h"
#include "uvc_frame_config.h"

static QueueHandle_t xQueueAIFrame = NULL;
static QueueHandle_t xQueueUSBFrame = NULL;

typedef struct
{
    camera_fb_t *cam_fb_p;
    uvc_fb_t uvc_fb;
} fb_t;

static fb_t s_fb;

#define CAMERA_XCLK_FREQ CONFIG_CAMERA_XCLK_FREQ
#define CAMERA_FB_COUNT 4

#if CONFIG_IDF_TARGET_ESP32S3
#define UVC_MAX_FRAMESIZE_SIZE (75 * 1024)
#else
#define UVC_MAX_FRAMESIZE_SIZE (60 * 1024)
#endif

static const char *TAG = "usb_webcam";

static void camera_stop_cb(void *cb_ctx)
{
    (void)cb_ctx;
    ESP_LOGI(TAG, "Camera Stop");
}

static esp_err_t camera_start_cb(uvc_format_t format, int width, int height, int rate, void *cb_ctx)
{
    (void)cb_ctx;
    ESP_LOGI(TAG, "Camera Start");
    return ESP_OK;
}
uint8_t *jpeg_out = NULL;
static uvc_fb_t *camera_fb_get_cb(void *cb_ctx)
{
    (void)cb_ctx;
    camera_fb_t *frame = NULL;
    if (xQueueReceive(xQueueUSBFrame, &frame, portMAX_DELAY) == pdTRUE)
    {
        s_fb.cam_fb_p = frame;
    }
    else
    {
        s_fb.cam_fb_p = NULL;
    }
    if (!s_fb.cam_fb_p)
    {
        return NULL;
    }

    size_t out_len;
    if (frame2jpg(s_fb.cam_fb_p, 65, &jpeg_out, &out_len))
    {
        if ((out_len == 0) && (jpeg_out == NULL)  )
        {
            esp_camera_fb_return(s_fb.cam_fb_p);
            if (jpeg_out)
            {
                free(jpeg_out);
                jpeg_out = NULL;
            }
            return NULL;
        }
        
        s_fb.uvc_fb.buf = jpeg_out;
        s_fb.uvc_fb.len = out_len;
        s_fb.uvc_fb.width = s_fb.cam_fb_p->width;
        s_fb.uvc_fb.height = s_fb.cam_fb_p->height;
        s_fb.uvc_fb.format = UVC_FORMAT_JPEG;
        s_fb.uvc_fb.timestamp = s_fb.cam_fb_p->timestamp;
    }
    else
    {
        esp_camera_fb_return(s_fb.cam_fb_p);
        if (jpeg_out)
        {
            free(jpeg_out);
            jpeg_out = NULL;
        }
        return NULL;
    }

    if (s_fb.uvc_fb.len > UVC_MAX_FRAMESIZE_SIZE)
    {
        esp_camera_fb_return(s_fb.cam_fb_p);
        if (jpeg_out)
        {
            free(jpeg_out);
            jpeg_out = NULL;
        }
        return NULL;
    }
    return &s_fb.uvc_fb;
}

static void camera_fb_return_cb(uvc_fb_t *fb, void *cb_ctx)
{
    (void)cb_ctx;
    assert(fb == &s_fb.uvc_fb);
    esp_camera_fb_return(s_fb.cam_fb_p);
    if (jpeg_out)
    {
        free(jpeg_out);
        jpeg_out = NULL;
    }
}

extern "C" void app_main()
{
    xQueueAIFrame = xQueueCreate(2, sizeof(camera_fb_t *));
    xQueueUSBFrame = xQueueCreate(2, sizeof(camera_fb_t *));

    register_camera(PIXFORMAT_RGB565, FRAMESIZE_QVGA, CAMERA_FB_COUNT, xQueueAIFrame);
    register_human_face_detection(xQueueAIFrame, NULL, NULL, xQueueUSBFrame, false);

    ESP_LOGI(TAG, "Selected Camera Board %s", CAMERA_MODULE_NAME);
    uint8_t *uvc_buffer = (uint8_t *)malloc(UVC_MAX_FRAMESIZE_SIZE);
    if (uvc_buffer == NULL)
    {
        ESP_LOGE(TAG, "malloc frame buffer fail");
        return;
    }

    uvc_device_config_t config = {
        .uvc_buffer = uvc_buffer,
        .uvc_buffer_size = UVC_MAX_FRAMESIZE_SIZE,
        .start_cb = camera_start_cb,
        .fb_get_cb = camera_fb_get_cb,
        .fb_return_cb = camera_fb_return_cb,
        .stop_cb = camera_stop_cb,
    };

    ESP_LOGI(TAG, "Format List");
    ESP_LOGI(TAG, "\tFormat(1) = %s", "MJPEG");
    ESP_LOGI(TAG, "Frame List");
    ESP_LOGI(TAG, "\tFrame(1) = %d * %d @%dfps", UVC_FRAMES_INFO[0][0].width, UVC_FRAMES_INFO[0][0].height, UVC_FRAMES_INFO[0][0].rate);

    ESP_ERROR_CHECK(uvc_device_config(0, &config));
    ESP_ERROR_CHECK(uvc_device_init());
}
