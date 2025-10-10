#include "udp_streamer.h"
#include "wifi_manager.h"
#include "config.h"
#include "esp_log.h"
#include "esp_camera.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>
#include <string.h>
#include <errno.h>

static const char *TAG = "udp_streamer";
static QueueHandle_t frame_queue = NULL;

typedef struct {
    uint8_t marker[5];
    uint16_t frame_len;
} __attribute__((packed)) frame_header_t;

static inline bool send_data_with_retry(int sock, const void *data, size_t len,
                                       const struct sockaddr *addr, socklen_t addrlen)
{
    for (uint8_t attempt = 0; attempt <= MAX_SEND_RETRIES; attempt++)
    {
        int sent = sendto(sock, data, len, 0, addr, addrlen);
        if (sent >= 0)
        {
            return true;
        }

        if (attempt < MAX_SEND_RETRIES)
        {
            ESP_LOGW(TAG, "Send retry %u/%u (errno %d)", 
                     attempt + 1, MAX_SEND_RETRIES, errno);
            vTaskDelay(pdMS_TO_TICKS(SEND_RETRY_BACKOFF_MS));
        }
    }

    ESP_LOGE(TAG, "Send failed after %u attempts (errno %d)", 
             MAX_SEND_RETRIES + 1, errno);
    return false;
}

static bool send_frame_marker(int sock, bool is_start, uint16_t frame_len,
                              const struct sockaddr *addr, socklen_t addrlen)
{
    uint8_t marker[5];
    
    if (is_start)
    {
        marker[0] = FRAME_START_MARKER_0;
        marker[1] = FRAME_START_MARKER_1;
        marker[2] = (uint8_t)(frame_len >> 8);
        marker[3] = (uint8_t)(frame_len & 0xFF);
        marker[4] = FRAME_END_MARKER_4;
    }
    else
    {
        marker[0] = FRAME_END_MARKER_0;
        marker[1] = FRAME_END_MARKER_1;
        marker[2] = FRAME_END_MARKER_2;
        marker[3] = FRAME_END_MARKER_3;
        marker[4] = FRAME_END_MARKER_4;
    }

    return send_data_with_retry(sock, marker, sizeof(marker), addr, addrlen);
}

static bool send_frame(int sock, camera_fb_t *frame, 
                      const struct sockaddr *addr, socklen_t addrlen)
{
    if (!frame || !frame->buf)
    {
        return false;
    }

    // Handle frame length overflow
    const uint16_t truncated_len = frame->len > UINT16_MAX ? 
                                   UINT16_MAX : (uint16_t)frame->len;
    
    if (frame->len > UINT16_MAX)
    {
        ESP_LOGW(TAG, "Frame length %zu truncated to %u", frame->len, truncated_len);
    }

    // Send start marker
    if (!send_frame_marker(sock, true, truncated_len, addr, addrlen))
    {
        return false;
    }

    // Send frame data in chunks
    size_t offset = 0;
    uint32_t chunks = 0;

    while (offset < frame->len)
    {
        size_t to_send = frame->len - offset;
        if (to_send > MAX_UDP_CHUNK)
        {
            to_send = MAX_UDP_CHUNK;
        }

        if (!send_data_with_retry(sock, frame->buf + offset, to_send, addr, addrlen))
        {
            return false;
        }

        offset += to_send;
        chunks++;
    }

    // Send end marker
    if (!send_frame_marker(sock, false, 0, addr, addrlen))
    {
        return false;
    }

    ESP_LOGI(TAG, "Frame sent: %lu chunks, %zu bytes", (unsigned long)chunks, frame->len);
    return true;
}

static void udp_send_task(void *arg)
{
    char rx_buffer[UDP_RX_BUFFER_SIZE];
    char addr_str[UDP_RX_BUFFER_SIZE];
    
    // Setup socket
    struct sockaddr_in dest_addr = {
        .sin_addr.s_addr = htonl(INADDR_ANY),
        .sin_family = AF_INET,
        .sin_port = htons(UDP_PORT)
    };

    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0)
    {
        ESP_LOGE(TAG, "Socket creation failed (errno %d)", errno);
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "Socket created");

    // Set receive timeout
    struct timeval timeout = {
        .tv_sec = UDP_TIMEOUT_SEC,
        .tv_usec = 0
    };
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

    // Bind socket
    if (bind(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr)) < 0)
    {
        ESP_LOGE(TAG, "Socket bind failed (errno %d)", errno);
        close(sock);
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "Socket bound to port %d", UDP_PORT);

    struct sockaddr_storage source_addr;
    socklen_t socklen = sizeof(source_addr);

    // Wait for client connection
    while (true)
    {
        int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0,
                          (struct sockaddr *)&source_addr, &socklen);
        
        if (len < 0)
        {
            if (errno != EWOULDBLOCK && errno != EAGAIN)
            {
                ESP_LOGE(TAG, "recvfrom failed (errno %d)", errno);
            }
            continue;
        }

        inet_ntoa_r(((struct sockaddr_in *)&source_addr)->sin_addr, 
                   addr_str, sizeof(addr_str) - 1);
        ESP_LOGI(TAG, "Streaming to %s:%d", addr_str, UDP_PORT);

        // Stream frames while connected
        while (wifi_manager_is_connected())
        {
            camera_fb_t *frame = NULL;
            
            if (xQueueReceive(frame_queue, &frame, portMAX_DELAY) != pdTRUE)
            {
                continue;
            }

            if (!frame)
            {
                continue;
            }

            if (!send_frame(sock, frame, (struct sockaddr *)&source_addr, socklen))
            {
                ESP_LOGW(TAG, "Frame dropped due to send failure");
            }

            esp_camera_fb_return(frame);
        }
    }

    close(sock);
    vTaskDelete(NULL);
}

esp_err_t udp_streamer_init(QueueHandle_t queue)
{
    if (!queue)
    {
        ESP_LOGE(TAG, "Invalid frame queue");
        return ESP_ERR_INVALID_ARG;
    }

    frame_queue = queue;

    BaseType_t ret = xTaskCreate(udp_send_task, "udp_send", 
                                 TASK_STACK_SIZE, NULL, TASK_PRIORITY, NULL);
    
    if (ret != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to create UDP send task");
        return ESP_FAIL;
    }

    return ESP_OK;
}
