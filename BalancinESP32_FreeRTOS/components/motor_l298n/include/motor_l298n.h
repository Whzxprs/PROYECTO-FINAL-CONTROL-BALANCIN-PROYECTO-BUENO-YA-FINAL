#pragma once

#include "esp_err.h"

esp_err_t motor_l298n_init(void);
void motor_l298n_set(int left_pwm, int right_pwm);
void motor_l298n_stop(void);
