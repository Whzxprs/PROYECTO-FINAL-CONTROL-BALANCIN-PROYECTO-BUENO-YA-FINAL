idf_component_register(
    SRCS "wifi_sta_udp.c"
    INCLUDE_DIRS "include"
    REQUIRES board esp_wifi esp_event esp_netif nvs_flash
)
