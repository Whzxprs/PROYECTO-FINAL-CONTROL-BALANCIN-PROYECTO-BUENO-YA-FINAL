#include "line_adc.h"

#include "board_pins.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_check.h"

static const char *TAG = "line_adc";

static adc_oneshot_unit_handle_t adc_handle;
static adc_channel_t left_channel;
static adc_channel_t right_channel;

esp_err_t line_adc_init(void)
{
    adc_oneshot_unit_init_cfg_t unit_config = {
        .unit_id = ADC_UNIT_1,
        .clk_src = ADC_RTC_CLK_SRC_DEFAULT,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_RETURN_ON_ERROR(adc_oneshot_new_unit(&unit_config, &adc_handle),
                        TAG, "adc unit failed");

    adc_unit_t unit;
    ESP_RETURN_ON_ERROR(adc_oneshot_io_to_channel(BALANCIN_LINE_LEFT_GPIO,
                                                  &unit,
                                                  &left_channel),
                        TAG, "left gpio to adc channel failed");
    ESP_RETURN_ON_ERROR(adc_oneshot_io_to_channel(BALANCIN_LINE_RIGHT_GPIO,
                                                  &unit,
                                                  &right_channel),
                        TAG, "right gpio to adc channel failed");

    adc_oneshot_chan_cfg_t channel_config = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };
    ESP_RETURN_ON_ERROR(adc_oneshot_config_channel(adc_handle,
                                                   left_channel,
                                                   &channel_config),
                        TAG, "left adc config failed");
    ESP_RETURN_ON_ERROR(adc_oneshot_config_channel(adc_handle,
                                                   right_channel,
                                                   &channel_config),
                        TAG, "right adc config failed");

    return ESP_OK;
}

esp_err_t line_adc_read(uint16_t *sl_raw,
                        uint16_t *sr_raw,
                        uint8_t *sl_8,
                        uint8_t *sr_8)
{
    int left = 0;
    int right = 0;

    ESP_RETURN_ON_ERROR(adc_oneshot_read(adc_handle, left_channel, &left),
                        TAG, "left adc read failed");
    ESP_RETURN_ON_ERROR(adc_oneshot_read(adc_handle, right_channel, &right),
                        TAG, "right adc read failed");

    if (left < 0) {
        left = 0;
    } else if (left > 4095) {
        left = 4095;
    }

    if (right < 0) {
        right = 0;
    } else if (right > 4095) {
        right = 4095;
    }

    *sl_raw = (uint16_t)left;
    *sr_raw = (uint16_t)right;
    *sl_8 = (uint8_t)(left >> 4);
    *sr_8 = (uint8_t)(right >> 4);

    return ESP_OK;
}
