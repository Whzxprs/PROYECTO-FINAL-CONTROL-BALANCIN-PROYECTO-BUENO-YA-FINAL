#include "encoder_quad.h"

#include "board_pins.h"
#include "driver/gpio.h"
#include "driver/pulse_cnt.h"
#include "esp_check.h"

#define PCNT_HIGH_LIMIT 1000
#define PCNT_LOW_LIMIT  -1000

static const char *TAG = "encoder_quad";

static pcnt_unit_handle_t right_unit;
static pcnt_unit_handle_t left_unit;

static esp_err_t configure_unit(pcnt_unit_handle_t *unit,
                                gpio_num_t phase_a,
                                gpio_num_t phase_b)
{
    gpio_set_pull_mode(phase_a, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(phase_b, GPIO_PULLUP_ONLY);

    pcnt_unit_config_t unit_config = {
        .high_limit = PCNT_HIGH_LIMIT,
        .low_limit = PCNT_LOW_LIMIT,
    };
    ESP_RETURN_ON_ERROR(pcnt_new_unit(&unit_config, unit), TAG, "pcnt_new_unit failed");

    pcnt_glitch_filter_config_t filter = {
        .max_glitch_ns = 1000,
    };
    ESP_RETURN_ON_ERROR(pcnt_unit_set_glitch_filter(*unit, &filter),
                        TAG, "pcnt_unit_set_glitch_filter failed");

    pcnt_channel_handle_t chan_a = NULL;
    pcnt_channel_handle_t chan_b = NULL;

    pcnt_chan_config_t chan_a_config = {
        .edge_gpio_num = phase_a,
        .level_gpio_num = phase_b,
    };
    pcnt_chan_config_t chan_b_config = {
        .edge_gpio_num = phase_b,
        .level_gpio_num = phase_a,
    };

    ESP_RETURN_ON_ERROR(pcnt_new_channel(*unit, &chan_a_config, &chan_a),
                        TAG, "pcnt_new_channel A failed");
    ESP_RETURN_ON_ERROR(pcnt_new_channel(*unit, &chan_b_config, &chan_b),
                        TAG, "pcnt_new_channel B failed");

    ESP_RETURN_ON_ERROR(pcnt_channel_set_edge_action(chan_a,
                                                     PCNT_CHANNEL_EDGE_ACTION_DECREASE,
                                                     PCNT_CHANNEL_EDGE_ACTION_INCREASE),
                        TAG, "pcnt edge A failed");
    ESP_RETURN_ON_ERROR(pcnt_channel_set_level_action(chan_a,
                                                      PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                                                      PCNT_CHANNEL_LEVEL_ACTION_INVERSE),
                        TAG, "pcnt level A failed");
    ESP_RETURN_ON_ERROR(pcnt_channel_set_edge_action(chan_b,
                                                     PCNT_CHANNEL_EDGE_ACTION_INCREASE,
                                                     PCNT_CHANNEL_EDGE_ACTION_DECREASE),
                        TAG, "pcnt edge B failed");
    ESP_RETURN_ON_ERROR(pcnt_channel_set_level_action(chan_b,
                                                      PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                                                      PCNT_CHANNEL_LEVEL_ACTION_INVERSE),
                        TAG, "pcnt level B failed");

    ESP_RETURN_ON_ERROR(pcnt_unit_enable(*unit), TAG, "pcnt enable failed");
    ESP_RETURN_ON_ERROR(pcnt_unit_clear_count(*unit), TAG, "pcnt clear failed");
    ESP_RETURN_ON_ERROR(pcnt_unit_start(*unit), TAG, "pcnt start failed");

    return ESP_OK;
}

esp_err_t encoder_quad_init(void)
{
    ESP_RETURN_ON_ERROR(configure_unit(&right_unit,
                                       BALANCIN_ENCODER_RIGHT_A_GPIO,
                                       BALANCIN_ENCODER_RIGHT_B_GPIO),
                        TAG, "right encoder failed");
    ESP_RETURN_ON_ERROR(configure_unit(&left_unit,
                                       BALANCIN_ENCODER_LEFT_A_GPIO,
                                       BALANCIN_ENCODER_LEFT_B_GPIO),
                        TAG, "left encoder failed");
    return ESP_OK;
}

void encoder_quad_read(int *right_delta, int *left_delta)
{
    int right = 0;
    int left = 0;

    pcnt_unit_get_count(right_unit, &right);
    pcnt_unit_clear_count(right_unit);
    pcnt_unit_get_count(left_unit, &left);
    pcnt_unit_clear_count(left_unit);

    if (BALANCIN_ENCODER_RIGHT_INVERT) {
        right = -right;
    }
    if (BALANCIN_ENCODER_LEFT_INVERT) {
        left = -left;
    }

    *right_delta = right;
    *left_delta = left;
}
