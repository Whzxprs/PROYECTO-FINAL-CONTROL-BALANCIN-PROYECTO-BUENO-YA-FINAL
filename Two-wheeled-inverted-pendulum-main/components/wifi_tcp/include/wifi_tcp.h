#pragma once

#include "esp_mac.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>
#include <string.h>

#define WIFI_SSID "balancin"
#define WIFI_PASS "1q2w3e4r"

#define BUF_SIZE 256
#define PACKET_SIZE 9

#define PORT                        4545
#define KEEPALIVE_IDLE              5
#define KEEPALIVE_INTERVAL          5
#define KEEPALIVE_COUNT             3

extern const char TAG_WIFI[];

extern const uint8_t wifi_ssid[];
extern const uint8_t wifi_pass[];
extern const int  wifi_chan;
extern const int  wifi_conn;

extern void init_wifi();

extern void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);
extern void tcp_server_task(void *pvParameters);
extern void TCP_receive(const int sock);