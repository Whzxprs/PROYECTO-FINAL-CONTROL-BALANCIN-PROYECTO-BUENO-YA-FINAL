#pragma once

#include "esp_err.h"

esp_err_t encoder_quad_init(void);
void encoder_quad_read(int *right_delta, int *left_delta);
