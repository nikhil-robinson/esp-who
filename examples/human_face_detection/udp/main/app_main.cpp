#include "who_camera.h"
#include "who_human_face_detection.hpp"
#include "config.h"
#include "wifi_manager.h"
#include "udp_streamer.h"
#include "frame_manager.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

static const char *TAG = "app_main";

extern "C" void app_main()
{
    esp_err_t ret;

    // Initialize NVS
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Initialize WiFi
    ESP_ERROR_CHECK(wifi_manager_init());

    // Create queues
    QueueHandle_t queue_ai_frame = xQueueCreate(QUEUE_SIZE, sizeof(camera_fb_t *));
    QueueHandle_t queue_udp_frame = xQueueCreate(QUEUE_SIZE, sizeof(camera_fb_t *));
    QueueHandle_t queue_frame_out = xQueueCreate(QUEUE_SIZE, sizeof(camera_fb_t *));

    if (!queue_ai_frame || !queue_udp_frame || !queue_frame_out)
    {
        ESP_LOGE(TAG, "Failed to create queues");
        return;
    }

    // Register camera and face detection
    register_camera(PIXFORMAT_RGB565, FRAMESIZE_240X240, 4, queue_ai_frame);
    register_human_face_detection(queue_ai_frame, NULL, NULL, queue_frame_out, false);

    // Initialize modules
    ESP_ERROR_CHECK(frame_manager_init(queue_frame_out, queue_udp_frame));
    ESP_ERROR_CHECK(udp_streamer_init(queue_udp_frame));

    ESP_LOGI(TAG, "System initialized successfully");
}
