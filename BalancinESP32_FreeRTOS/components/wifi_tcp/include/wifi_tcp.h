#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

esp_err_t wifi_tcp_start(void);
bool wifi_tcp_is_connected(void);
void wifi_tcp_send(const uint8_t *data, size_t len);
