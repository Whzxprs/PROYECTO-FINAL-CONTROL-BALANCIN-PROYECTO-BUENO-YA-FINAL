#include "wifi_tcp.h"

#include <errno.h>
#include <stdio.h>
#include <string.h>

#include "board_pins.h"
#include "esp_check.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/inet.h"
#include "lwip/netdb.h"
#include "lwip/sockets.h"
#include "lwip/tcp.h"
#include "nvs_flash.h"

static const char *TAG = "wifi_tcp";

static volatile int client_sock = -1;

static void wifi_event_handler(void *arg,
                               esp_event_base_t event_base,
                               int32_t event_id,
                               void *event_data)
{
    (void)arg;
    (void)event_base;

    if (event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t *event = event_data;
        ESP_LOGI(TAG, "station " MACSTR " joined, aid=%d",
                 MAC2STR(event->mac), event->aid);
    } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t *event = event_data;
        ESP_LOGI(TAG, "station " MACSTR " left, aid=%d",
                 MAC2STR(event->mac), event->aid);
    }
}

static void tcp_server_task(void *arg)
{
    (void)arg;

    struct sockaddr_in listen_addr = {
        .sin_addr.s_addr = htonl(INADDR_ANY),
        .sin_family = AF_INET,
        .sin_port = htons(BALANCIN_TCP_PORT),
    };

    int listen_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if (listen_sock < 0) {
        ESP_LOGE(TAG, "socket failed errno=%d", errno);
        vTaskDelete(NULL);
        return;
    }

    int opt = 1;
    setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    if (bind(listen_sock, (struct sockaddr *)&listen_addr, sizeof(listen_addr)) != 0) {
        ESP_LOGE(TAG, "bind failed errno=%d", errno);
        close(listen_sock);
        vTaskDelete(NULL);
        return;
    }

    if (listen(listen_sock, 1) != 0) {
        ESP_LOGE(TAG, "listen failed errno=%d", errno);
        close(listen_sock);
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "tcp server listening on port %d", BALANCIN_TCP_PORT);

    for (;;) {
        struct sockaddr_storage source_addr;
        socklen_t addr_len = sizeof(source_addr);
        int sock = accept(listen_sock, (struct sockaddr *)&source_addr, &addr_len);
        if (sock < 0) {
            ESP_LOGE(TAG, "accept failed errno=%d", errno);
            continue;
        }

        int nodelay = 1;
        setsockopt(sock, IPPROTO_TCP, TCP_NODELAY, &nodelay, sizeof(nodelay));

        struct timeval timeout = {
            .tv_sec = 1,
            .tv_usec = 0,
        };
        setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

        client_sock = sock;
        ESP_LOGI(TAG, "tcp client connected");

        uint8_t rx[64];
        while (client_sock == sock) {
            int len = recv(sock, rx, sizeof(rx), 0);
            if (len == 0) {
                ESP_LOGW(TAG, "tcp client closed");
                break;
            }
            if (len < 0 && errno != EWOULDBLOCK && errno != EAGAIN) {
                ESP_LOGW(TAG, "recv failed errno=%d", errno);
                break;
            }
        }

        client_sock = -1;
        shutdown(sock, 0);
        close(sock);
        ESP_LOGI(TAG, "tcp client disconnected");
    }
}

esp_err_t wifi_tcp_start(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_RETURN_ON_ERROR(nvs_flash_erase(), TAG, "nvs erase failed");
        ret = nvs_flash_init();
    }
    ESP_RETURN_ON_ERROR(ret, TAG, "nvs init failed");
    ESP_RETURN_ON_ERROR(esp_netif_init(), TAG, "netif init failed");
    ESP_RETURN_ON_ERROR(esp_event_loop_create_default(), TAG, "event loop failed");
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_RETURN_ON_ERROR(esp_wifi_init(&cfg), TAG, "wifi init failed");
    ESP_RETURN_ON_ERROR(esp_event_handler_instance_register(WIFI_EVENT,
                                                            ESP_EVENT_ANY_ID,
                                                            wifi_event_handler,
                                                            NULL,
                                                            NULL),
                        TAG, "event handler failed");

    wifi_config_t wifi_config = {0};
    snprintf((char *)wifi_config.ap.ssid,
             sizeof(wifi_config.ap.ssid),
             "%s",
             BALANCIN_WIFI_SSID);
    snprintf((char *)wifi_config.ap.password,
             sizeof(wifi_config.ap.password),
             "%s",
             BALANCIN_WIFI_PASS);
    wifi_config.ap.ssid_len = strlen(BALANCIN_WIFI_SSID);
    wifi_config.ap.channel = 1;
    wifi_config.ap.max_connection = 2;
    wifi_config.ap.authmode = strlen(BALANCIN_WIFI_PASS) == 0 ?
                              WIFI_AUTH_OPEN :
                              WIFI_AUTH_WPA2_PSK;

    ESP_RETURN_ON_ERROR(esp_wifi_set_mode(WIFI_MODE_AP), TAG, "set mode failed");
    ESP_RETURN_ON_ERROR(esp_wifi_set_config(WIFI_IF_AP, &wifi_config),
                        TAG, "set config failed");
    ESP_RETURN_ON_ERROR(esp_wifi_start(), TAG, "wifi start failed");

    xTaskCreate(tcp_server_task, "tcp_server", 4096, NULL, 8, NULL);

    ESP_LOGI(TAG, "softAP started ssid=%s port=%d",
             BALANCIN_WIFI_SSID, BALANCIN_TCP_PORT);
    return ESP_OK;
}

bool wifi_tcp_is_connected(void)
{
    return client_sock >= 0;
}

void wifi_tcp_send(const uint8_t *data, size_t len)
{
    int sock = client_sock;
    if (sock < 0) {
        return;
    }

    size_t sent = 0;
    while (sent < len) {
        int written = send(sock, data + sent, len - sent, 0);
        if (written <= 0) {
            return;
        }
        sent += (size_t)written;
    }
}
