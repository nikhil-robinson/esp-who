#ifndef FRAME_MANAGER_H
#define FRAME_MANAGER_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize and start the frame manager
 * @param input_queue Queue to receive frames from AI detection
 * @param output_queue Queue to send frames to UDP streamer
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t frame_manager_init(QueueHandle_t input_queue, QueueHandle_t output_queue);

#ifdef __cplusplus
}
#endif

#endif // FRAME_MANAGER_H
