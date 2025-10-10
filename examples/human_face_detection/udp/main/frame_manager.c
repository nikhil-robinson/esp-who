#include "frame_manager.h"
#include "wifi_manager.h"
#include "config.h"
#include "esp_log.h"
#include "esp_camera.h"

static const char *TAG = "frame_manager";
static QueueHandle_t input_queue = NULL;
static QueueHandle_t output_queue = NULL;

static void frame_parser_task(void *arg)
{
    while (true)
    {
        camera_fb_t *frame = NULL;
        
        if (xQueueReceive(input_queue, &frame, portMAX_DELAY) != pdTRUE)
        {
            continue;
        }

        if (!frame)
        {
            continue;
        }

        // Drop frame if not connected
        if (!wifi_manager_is_connected())
        {
            esp_camera_fb_return(frame);
            continue;
        }

        // Try to send frame to output queue
        if (xQueueSend(output_queue, &frame, 0) != pdTRUE)
        {
            ESP_LOGW(TAG, "Output queue full, dropping frame");
            esp_camera_fb_return(frame);
        }
    }
}

esp_err_t frame_manager_init(QueueHandle_t in_queue, QueueHandle_t out_queue)
{
    if (!in_queue || !out_queue)
    {
        ESP_LOGE(TAG, "Invalid queue handles");
        return ESP_ERR_INVALID_ARG;
    }

    input_queue = in_queue;
    output_queue = out_queue;

    BaseType_t ret = xTaskCreate(frame_parser_task, "frame_parser",
                                 TASK_STACK_SIZE, NULL, TASK_PRIORITY, NULL);
    
    if (ret != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to create frame parser task");
        return ESP_FAIL;
    }

    return ESP_OK;
}
