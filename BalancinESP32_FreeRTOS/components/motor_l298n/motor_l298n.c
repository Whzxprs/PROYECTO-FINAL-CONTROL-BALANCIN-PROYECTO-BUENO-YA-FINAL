#include "motor_l298n.h"

#include <stdlib.h>

#include "board_pins.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_check.h"

#define MOTOR_PWM_FREQ_HZ 20000
#define MOTOR_PWM_RES LEDC_TIMER_8_BIT
#define MOTOR_PWM_TIMER LEDC_TIMER_0
#define MOTOR_PWM_MODE LEDC_LOW_SPEED_MODE
#define MOTOR_LEFT_CHANNEL LEDC_CHANNEL_0
#define MOTOR_RIGHT_CHANNEL LEDC_CHANNEL_1
#define MOTOR_PWM_MAX_DUTY 255

static const char *TAG = "motor_l298n";

static void set_direction(gpio_num_t in1, gpio_num_t in2, int pwm, int invert)
{
    if (pwm == 0) {
        gpio_set_level(in1, 0);
        gpio_set_level(in2, 0);
        return;
    }

    int positive = pwm > 0;
    if (invert) {
        positive = !positive;
    }

    if (positive) {
        gpio_set_level(in1, 0);
        gpio_set_level(in2, 1);
    } else {
        gpio_set_level(in1, 1);
        gpio_set_level(in2, 0);
    }
}

static uint32_t duty_from_pwm(int pwm)
{
    int duty = abs(pwm);
    if (duty > MOTOR_PWM_MAX_DUTY) {
        duty = MOTOR_PWM_MAX_DUTY;
    }
    return (uint32_t)duty;
}

esp_err_t motor_l298n_init(void)
{
    gpio_config_t gpio = {
        .pin_bit_mask = (1ULL << BALANCIN_MOTOR_LEFT_IN1_GPIO) |
                        (1ULL << BALANCIN_MOTOR_LEFT_IN2_GPIO) |
                        (1ULL << BALANCIN_MOTOR_RIGHT_IN1_GPIO) |
                        (1ULL << BALANCIN_MOTOR_RIGHT_IN2_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&gpio), TAG, "gpio_config failed");

    ledc_timer_config_t timer = {
        .speed_mode = MOTOR_PWM_MODE,
        .duty_resolution = MOTOR_PWM_RES,
        .timer_num = MOTOR_PWM_TIMER,
        .freq_hz = MOTOR_PWM_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_RETURN_ON_ERROR(ledc_timer_config(&timer), TAG, "ledc_timer_config failed");

    ledc_channel_config_t left = {
        .speed_mode = MOTOR_PWM_MODE,
        .channel = MOTOR_LEFT_CHANNEL,
        .timer_sel = MOTOR_PWM_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = BALANCIN_MOTOR_LEFT_PWM_GPIO,
        .duty = 0,
        .hpoint = 0,
    };
    ESP_RETURN_ON_ERROR(ledc_channel_config(&left), TAG, "left channel failed");

    ledc_channel_config_t right = {
        .speed_mode = MOTOR_PWM_MODE,
        .channel = MOTOR_RIGHT_CHANNEL,
        .timer_sel = MOTOR_PWM_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = BALANCIN_MOTOR_RIGHT_PWM_GPIO,
        .duty = 0,
        .hpoint = 0,
    };
    ESP_RETURN_ON_ERROR(ledc_channel_config(&right), TAG, "right channel failed");

    motor_l298n_stop();
    return ESP_OK;
}

void motor_l298n_set(int left_pwm, int right_pwm)
{
    set_direction(BALANCIN_MOTOR_LEFT_IN1_GPIO,
                  BALANCIN_MOTOR_LEFT_IN2_GPIO,
                  left_pwm,
                  BALANCIN_MOTOR_LEFT_INVERT);
    set_direction(BALANCIN_MOTOR_RIGHT_IN1_GPIO,
                  BALANCIN_MOTOR_RIGHT_IN2_GPIO,
                  right_pwm,
                  BALANCIN_MOTOR_RIGHT_INVERT);

    ledc_set_duty(MOTOR_PWM_MODE, MOTOR_LEFT_CHANNEL, duty_from_pwm(left_pwm));
    ledc_update_duty(MOTOR_PWM_MODE, MOTOR_LEFT_CHANNEL);
    ledc_set_duty(MOTOR_PWM_MODE, MOTOR_RIGHT_CHANNEL, duty_from_pwm(right_pwm));
    ledc_update_duty(MOTOR_PWM_MODE, MOTOR_RIGHT_CHANNEL);
}

void motor_l298n_stop(void)
{
    motor_l298n_set(0, 0);
}
