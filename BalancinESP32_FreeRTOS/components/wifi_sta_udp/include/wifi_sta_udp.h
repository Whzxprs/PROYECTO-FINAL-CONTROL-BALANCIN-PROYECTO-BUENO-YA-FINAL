#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

esp_err_t wifi_sta_udp_start(void);
void wifi_udp_send(const uint8_t *data, size_t len);
bool wifi_udp_receive_command(uint8_t *cmd_id, float *value);
