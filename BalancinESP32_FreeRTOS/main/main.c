#include <stdint.h>

#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "balancin_control.h"
#include "encoder_quad.h"
#include "line_adc.h"
#include "motor_l298n.h"
#include "mpu6050.h"
#include "wifi_tcp.h"

static const char *TAG = "balancin";

static balancin_control_t ctrl;

static void control_task(void *arg)
{
    (void)arg;

    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(BALANCIN_CONTROL_PERIOD_MS);

    for (;;) {
        uint16_t sl_raw = 0;
        uint16_t sr_raw = 0;
        uint8_t sl_8 = 0;
        uint8_t sr_8 = 0;
        int16_t ax = 0;
        int16_t gy = 0;
        int right_delta = 0;
        int left_delta = 0;

        esp_err_t adc_err = line_adc_read(&sl_raw, &sr_raw, &sl_8, &sr_8);
        esp_err_t mpu_err = mpu6050_read_raw(&ax, &gy);
        encoder_quad_read(&right_delta, &left_delta);

        if (adc_err != ESP_OK || mpu_err != ESP_OK) {
            motor_l298n_stop();
            ESP_LOGW(TAG, "sensor read failed adc=%s mpu=%s",
                     esp_err_to_name(adc_err), esp_err_to_name(mpu_err));
            vTaskDelayUntil(&last_wake, period);
            continue;
        }

        balancin_control_set_inputs(&ctrl,
                                    (float)sl_8,
                                    (float)sr_8,
                                    (float)ax,
                                    (float)gy,
                                    (float)right_delta,
                                    (float)left_delta);
        balancin_control_step(&ctrl);

        motor_l298n_set(ctrl.left_pwm, ctrl.right_pwm);

        uint8_t packet[BALANCIN_LEGACY_PACKET_SIZE];
        balancin_control_make_legacy_packet(&ctrl, packet);
        wifi_tcp_send(packet, sizeof(packet));

        vTaskDelayUntil(&last_wake, period);
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "starting ESP32 DevKit V1 balancin firmware");

    balancin_control_init(&ctrl);
    ESP_ERROR_CHECK(line_adc_init());
    ESP_ERROR_CHECK(encoder_quad_init());
    ESP_ERROR_CHECK(motor_l298n_init());
    ESP_ERROR_CHECK(mpu6050_init());
    ESP_ERROR_CHECK(wifi_tcp_start());

    ESP_LOGI(TAG, "control period: %d ms", BALANCIN_CONTROL_PERIOD_MS);
    xTaskCreatePinnedToCore(control_task, "control", 4096, NULL, 15, NULL, 1);
}
