#ifndef UDP_STREAMER_H
#define UDP_STREAMER_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize and start the UDP streamer
 * @param frame_queue Queue to receive frames from
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t udp_streamer_init(QueueHandle_t frame_queue);

#ifdef __cplusplus
}
#endif

#endif // UDP_STREAMER_H
