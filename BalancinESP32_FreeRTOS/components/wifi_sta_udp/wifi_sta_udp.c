#include "wifi_sta_udp.h"

#include <errno.h>
#include <fcntl.h>
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
#include "freertos/event_groups.h"
#include "lwip/inet.h"
#include "lwip/sockets.h"
#include "nvs_flash.h"

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1
#define WIFI_MAX_RETRY     10

static const char *TAG = "wifi_sta_udp";

static int udp_sock = -1;
static int udp_cmd_sock = -1;
static struct sockaddr_in udp_peer_addr;
static EventGroupHandle_t wifi_event_group;
static int wifi_retry_count;

static void wifi_event_handler(void *arg,
                               esp_event_base_t event_base,
                               int32_t event_id,
                               void *event_data)
{
    (void)arg;

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (wifi_retry_count < WIFI_MAX_RETRY) {
            wifi_retry_count++;
            ESP_LOGW(TAG, "wifi disconnected, retry %d/%d",
                     wifi_retry_count, WIFI_MAX_RETRY);
            esp_wifi_connect();
        } else {
            xEventGroupSetBits(wifi_event_group, WIFI_FAIL_BIT);
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = event_data;
        wifi_retry_count = 0;
        ESP_LOGI(TAG, "connected, esp32 ip=" IPSTR, IP2STR(&event->ip_info.ip));
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static void udp_telemetry_start(void)
{
    udp_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (udp_sock < 0) {
        ESP_LOGW(TAG, "udp socket failed errno=%d", errno);
        return;
    }

    int broadcast = 1;
    if (setsockopt(udp_sock, SOL_SOCKET, SO_BROADCAST, &broadcast, sizeof(broadcast)) != 0) {
        ESP_LOGW(TAG, "udp broadcast option failed errno=%d", errno);
        close(udp_sock);
        udp_sock = -1;
        return;
    }

    memset(&udp_peer_addr, 0, sizeof(udp_peer_addr));
    udp_peer_addr.sin_family = AF_INET;
    udp_peer_addr.sin_port = htons(BALANCIN_UDP_TELEMETRY_PORT);

    if (strcmp(BALANCIN_UDP_TELEMETRY_HOST, "255.255.255.255") == 0) {
        udp_peer_addr.sin_addr.s_addr = htonl(INADDR_BROADCAST);
    } else if (inet_pton(AF_INET, BALANCIN_UDP_TELEMETRY_HOST, &udp_peer_addr.sin_addr) != 1) {
        ESP_LOGW(TAG, "invalid udp telemetry host=%s", BALANCIN_UDP_TELEMETRY_HOST);
        close(udp_sock);
        udp_sock = -1;
        return;
    }

    ESP_LOGI(TAG, "udp telemetry target %s:%d",
             BALANCIN_UDP_TELEMETRY_HOST,
             BALANCIN_UDP_TELEMETRY_PORT);
}

static void udp_command_start(void)
{
    udp_cmd_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (udp_cmd_sock < 0) {
        ESP_LOGW(TAG, "udp command socket failed errno=%d", errno);
        return;
    }

    int reuse = 1;
    setsockopt(udp_cmd_sock, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    struct sockaddr_in listen_addr = {0};
    listen_addr.sin_family = AF_INET;
    listen_addr.sin_port = htons(BALANCIN_UDP_COMMAND_PORT);
    listen_addr.sin_addr.s_addr = htonl(INADDR_ANY);

    if (bind(udp_cmd_sock, (struct sockaddr *)&listen_addr, sizeof(listen_addr)) != 0) {
        ESP_LOGW(TAG, "udp command bind failed errno=%d", errno);
        close(udp_cmd_sock);
        udp_cmd_sock = -1;
        return;
    }

    int flags = fcntl(udp_cmd_sock, F_GETFL, 0);
    if (flags >= 0) {
        fcntl(udp_cmd_sock, F_SETFL, flags | O_NONBLOCK);
    }

    ESP_LOGI(TAG, "udp command listener port %d", BALANCIN_UDP_COMMAND_PORT);
}

esp_err_t wifi_sta_udp_start(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_RETURN_ON_ERROR(nvs_flash_erase(), TAG, "nvs erase failed");
        ret = nvs_flash_init();
    }
    ESP_RETURN_ON_ERROR(ret, TAG, "nvs init failed");
    ESP_RETURN_ON_ERROR(esp_netif_init(), TAG, "netif init failed");
    ESP_RETURN_ON_ERROR(esp_event_loop_create_default(), TAG, "event loop failed");
    esp_netif_create_default_wifi_sta();

    wifi_event_group = xEventGroupCreate();
    ESP_RETURN_ON_FALSE(wifi_event_group != NULL,
                        ESP_ERR_NO_MEM,
                        TAG,
                        "event group failed");

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_RETURN_ON_ERROR(esp_wifi_init(&cfg), TAG, "wifi init failed");
    ESP_RETURN_ON_ERROR(esp_event_handler_instance_register(WIFI_EVENT,
                                                            ESP_EVENT_ANY_ID,
                                                            wifi_event_handler,
                                                            NULL,
                                                            NULL),
                        TAG, "event handler failed");
    ESP_RETURN_ON_ERROR(esp_event_handler_instance_register(IP_EVENT,
                                                            IP_EVENT_STA_GOT_IP,
                                                            wifi_event_handler,
                                                            NULL,
                                                            NULL),
                        TAG, "ip event handler failed");

    wifi_config_t wifi_config = {0};
    snprintf((char *)wifi_config.sta.ssid,
             sizeof(wifi_config.sta.ssid),
             "%s",
             BALANCIN_WIFI_SSID);
    snprintf((char *)wifi_config.sta.password,
             sizeof(wifi_config.sta.password),
             "%s",
             BALANCIN_WIFI_PASS);
    if (strlen(BALANCIN_WIFI_PASS) == 0) {
        wifi_config.sta.threshold.authmode = WIFI_AUTH_OPEN;
    } else {
        wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    }

    ESP_RETURN_ON_ERROR(esp_wifi_set_mode(WIFI_MODE_STA), TAG, "set mode failed");
    ESP_RETURN_ON_ERROR(esp_wifi_set_config(WIFI_IF_STA, &wifi_config),
                        TAG, "set config failed");
    ESP_RETURN_ON_ERROR(esp_wifi_start(), TAG, "wifi start failed");

    ESP_LOGI(TAG, "connecting to wifi ssid=%s", BALANCIN_WIFI_SSID);
    EventBits_t bits = xEventGroupWaitBits(wifi_event_group,
                                           WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                           pdFALSE,
                                           pdFALSE,
                                           pdMS_TO_TICKS(15000));
    ESP_RETURN_ON_FALSE((bits & WIFI_CONNECTED_BIT) != 0,
                        ESP_ERR_TIMEOUT,
                        TAG,
                        "wifi connection failed");

    udp_telemetry_start();
    udp_command_start();

    ESP_LOGI(TAG, "wifi STA ready udp_target=%s:%d",
             BALANCIN_UDP_TELEMETRY_HOST,
             BALANCIN_UDP_TELEMETRY_PORT);
    return ESP_OK;
}

void wifi_udp_send(const uint8_t *data, size_t len)
{
    int sock = udp_sock;
    if (sock < 0) {
        return;
    }

    int written = sendto(sock,
                         data,
                         len,
                         0,
                         (struct sockaddr *)&udp_peer_addr,
                         sizeof(udp_peer_addr));
    if (written < 0) {
        ESP_LOGD(TAG, "udp send failed errno=%d", errno);
    }
}

bool wifi_udp_receive_command(uint8_t *cmd_id, float *value)
{
    if (udp_cmd_sock < 0 || cmd_id == NULL || value == NULL) {
        return false;
    }

    uint8_t buf[16] = {0};
    int len = recvfrom(udp_cmd_sock, buf, sizeof(buf), 0, NULL, NULL);
    if (len < 0) {
        if (errno != EWOULDBLOCK && errno != EAGAIN) {
            ESP_LOGD(TAG, "udp command recv failed errno=%d", errno);
        }
        return false;
    }

    if (len < 6 || buf[0] != 0xBB) {
        ESP_LOGD(TAG, "invalid udp command len=%d header=0x%02x", len, buf[0]);
        return false;
    }

    *cmd_id = buf[1];
    memcpy(value, &buf[2], sizeof(*value));
    return true;
}
