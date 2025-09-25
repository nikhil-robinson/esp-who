#include "who_camera.h"
#include "who_human_face_detection.hpp"
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_camera.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>
#include "esp_mac.h"

static QueueHandle_t xQueueAIFrame = NULL;
static QueueHandle_t xQueueUDPFrame = NULL;

static const char *TAG = "udp_camera";

#define PORT 3333
#define EXAMPLE_ESP_WIFI_SSID "AI-0-SERVER"
#define EXAMPLE_ESP_WIFI_PASS "12345678"
#define EXAMPLE_ESP_WIFI_CHANNEL 2
#define EXAMPLE_MAX_STA_CONN 1

static volatile bool is_connected = false;

#define MAX_UDP_CHUNK 60000 // safe chunk size

typedef struct
{
    uint16_t frame_id;  // frame number
    uint16_t offset;    // offset of chunk in frame
    uint32_t frame_len; // total frame size (only in first chunk)
} __attribute__((packed)) chunk_header_t;

int send_frame_in_chunks(int sock, struct sockaddr_in *dest_addr,
                         const uint8_t *frame_buf, size_t frame_len,
                         uint16_t frame_id)
{
    size_t offset = 0;
    while (offset < frame_len)
    {
        size_t payload_size = frame_len - offset;
        if (offset == 0)
        {
            // first chunk includes frame_len
            if (payload_size > MAX_UDP_CHUNK - sizeof(chunk_header_t))
                payload_size = MAX_UDP_CHUNK - sizeof(chunk_header_t);
        }
        else
        {
            if (payload_size > MAX_UDP_CHUNK - sizeof(chunk_header_t) + sizeof(uint32_t))
                payload_size = MAX_UDP_CHUNK - sizeof(chunk_header_t) + sizeof(uint32_t);
        }

        uint8_t packet[MAX_UDP_CHUNK];
        chunk_header_t header = {
            .frame_id = htons(frame_id),
            .offset = htons(offset),
            .frame_len = htonl(offset == 0 ? frame_len : 0) // only first chunk has frame_len
        };

        memcpy(packet, &header, sizeof(header));
        memcpy(packet + sizeof(header), frame_buf + offset, payload_size);

        int err = sendto(sock, packet, payload_size + sizeof(header), 0, (struct sockaddr *)dest_addr, sizeof(*dest_addr));
        if (err < 0)
        {
            ESP_LOGE(TAG, "Error sending chunk: errno %d", errno);
            return err;
        }

        offset += payload_size;
    }
    return 0;
}
static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    if (event_id == WIFI_EVENT_AP_STACONNECTED)
    {
        wifi_event_ap_staconnected_t *event = (wifi_event_ap_staconnected_t *)event_data;
        ESP_LOGI(TAG, "station " MACSTR " join, AID=%d", MAC2STR(event->mac), event->aid);
        is_connected = true;
    }
    else if (event_id == WIFI_EVENT_AP_STADISCONNECTED)
    {
        wifi_event_ap_stadisconnected_t *event = (wifi_event_ap_stadisconnected_t *)event_data;
        ESP_LOGI(TAG, "station " MACSTR " leave, AID=%d", MAC2STR(event->mac), event->aid);
        is_connected = false;
    }
}

static void camer_read_task(void *arg)
{
    char rx_buffer[128];
    char addr_str[128];
    int addr_family = AF_INET;
    int ip_protocol = 0;
    struct sockaddr_in6 dest_addr;

    struct sockaddr_in *dest_addr_ip4 = (struct sockaddr_in *)&dest_addr;
    dest_addr_ip4->sin_addr.s_addr = htonl(INADDR_ANY);
    dest_addr_ip4->sin_family = AF_INET;
    dest_addr_ip4->sin_port = htons(PORT);
    ip_protocol = IPPROTO_IP;

    int sock = socket(AF_INET, SOCK_DGRAM, ip_protocol);
    if (sock < 0)
    {
        ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
        vTaskDelete(NULL);
    }
    ESP_LOGI(TAG, "Socket created");

    struct timeval timeout;
    timeout.tv_sec = 10;
    timeout.tv_usec = 0;
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof timeout);

    int err = bind(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    if (err < 0)
    {
        ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
    }
    ESP_LOGI(TAG, "Socket bound, port %d", PORT);

    struct sockaddr_storage source_addr; // Large enough for both IPv4 or IPv6
    socklen_t socklen = sizeof(source_addr);

    while (true)
    {

        int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0, (struct sockaddr *)&source_addr, &socklen);
        if (len < 0)
        {
            ESP_LOGE(TAG, "recvfrom failed: errno %d", errno);
            continue;
        }
        
        inet_ntoa_r(((struct sockaddr_in *)&source_addr)->sin_addr, addr_str, sizeof(addr_str) - 1);

        while (1)
        {
            if (!is_connected)
            {
                break;
            }
            
            camera_fb_t *frame = NULL;
            if (xQueueReceive(xQueueUDPFrame, &frame, portMAX_DELAY) == pdTRUE)
            {
                if (is_connected)
                {
                    size_t offset = 0;
                    uint32_t chunks = 0;
                    while (offset < frame->len)
                    {
                        size_t to_send = frame->len - offset;
                        if (to_send > MAX_UDP_CHUNK)
                            to_send = MAX_UDP_CHUNK;

                        int err = sendto(sock, frame->buf + offset, to_send, 0, (struct sockaddr *)&source_addr, sizeof(source_addr));
                        if (err < 0)
                        {
                            ESP_LOGE(TAG, "Error sending chunk %u: errno %d", (unsigned)chunks, errno);
                            break;
                        }

                        offset += to_send;
                        ++chunks;
                    }
                    if (offset == frame->len)
                    {
                        ESP_LOGI(TAG, "Frame sent in %u chunks, size: %zu bytes", (unsigned)chunks, frame->len);
                    }
                }
                esp_camera_fb_return(frame); // return after sending
            }
        }
    }
}

void wifi_init_softap(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL));

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = EXAMPLE_ESP_WIFI_SSID,
            .password = EXAMPLE_ESP_WIFI_PASS,
            .ssid_len = strlen(EXAMPLE_ESP_WIFI_SSID),
            .channel = EXAMPLE_ESP_WIFI_CHANNEL,
            .authmode = WIFI_AUTH_WPA2_PSK,
            .max_connection = EXAMPLE_MAX_STA_CONN,
            .pmf_cfg = {
                .required = true,
            },
        },
    };
    if (strlen(EXAMPLE_ESP_WIFI_PASS) == 0)
    {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_softap finished. SSID:%s password:%s channel:%d", EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS, EXAMPLE_ESP_WIFI_CHANNEL);
}

extern "C" void app_main()
{
    ESP_ERROR_CHECK(nvs_flash_init());
    wifi_init_softap();
    xQueueAIFrame = xQueueCreate(2, sizeof(camera_fb_t *));
    xQueueUDPFrame = xQueueCreate(2, sizeof(camera_fb_t *));

    register_camera(PIXFORMAT_RGB565, FRAMESIZE_240X240, 4, xQueueAIFrame);
    register_human_face_detection(xQueueAIFrame, NULL, NULL, xQueueUDPFrame, false);
    xTaskCreate(camer_read_task, "camer_read_task", 8 * 1024, NULL, 5, NULL);
}
