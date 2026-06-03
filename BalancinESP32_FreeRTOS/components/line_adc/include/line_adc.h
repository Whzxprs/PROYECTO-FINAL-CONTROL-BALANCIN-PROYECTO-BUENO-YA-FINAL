#pragma once

#include <stdint.h>

#include "esp_err.h"

esp_err_t line_adc_init(void);
esp_err_t line_adc_read(uint16_t *sl_raw,
                        uint16_t *sr_raw,
                        uint8_t *sl_8,
                        uint8_t *sr_8);
