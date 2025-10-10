#ifndef CONFIG_H
#define CONFIG_H

// WiFi Configuration
#define WIFI_SSID "AI-0-SERVER"
#define WIFI_PASS "12345678"
#define WIFI_CHANNEL 2
#define MAX_STA_CONN 1

// UDP Configuration
#define UDP_PORT 3333
#define UDP_TIMEOUT_SEC 10
#define UDP_RX_BUFFER_SIZE 128

// Frame Configuration
#define MAX_UDP_CHUNK 40000
#define MAX_SEND_RETRIES 3
#define SEND_RETRY_BACKOFF_MS 20

// Queue Configuration
#define QUEUE_SIZE 2
#define TASK_STACK_SIZE (8 * 1024)
#define TASK_PRIORITY 5

// Frame Markers
#define FRAME_START_MARKER_0 0xFF
#define FRAME_START_MARKER_1 0xF5
#define FRAME_END_MARKER_0 0xFF
#define FRAME_END_MARKER_1 0xF5
#define FRAME_END_MARKER_2 'E'
#define FRAME_END_MARKER_3 'D'
#define FRAME_END_MARKER_4 0xF9

#endif // CONFIG_H
