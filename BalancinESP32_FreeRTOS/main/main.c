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
#include "wifi_sta_udp.h"

static const char *TAG = "balancin";

#define UDP_TELEMETRY_DIVIDER 20

static balancin_control_t ctrl;
static uint16_t udp_telem_seq;

static void process_udp_commands(void)
{
    uint8_t cmd_id = 0;
    float value = 0.0f;

    for (int i = 0; i < 16 && wifi_udp_receive_command(&cmd_id, &value); i++) {
        if (balancin_control_apply_command(&ctrl, cmd_id, value)) {
            ESP_LOGI(TAG, "udp cmd id=%u value=%.4f", cmd_id, value);
        } else {
            ESP_LOGW(TAG, "udp cmd rejected id=%u value=%.4f", cmd_id, value);
        }
    }
}

static void control_task(void *arg)
{
    (void)arg;

    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(BALANCIN_CONTROL_PERIOD_MS);

    for (;;) {
        process_udp_commands();

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

        static uint8_t udp_divider;
        udp_divider++;
        if (udp_divider >= UDP_TELEMETRY_DIVIDER) {
            udp_divider = 0;
            uint8_t udp_packet[BALANCIN_TELEMETRY_PACKET_V2_SIZE];
            balancin_control_make_telemetry_v2_packet(&ctrl,
                                                      sl_raw,
                                                      sr_raw,
                                                      udp_telem_seq++,
                                                      0,
                                                      udp_packet);
            wifi_udp_send(udp_packet, sizeof(udp_packet));
        }

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
    ESP_ERROR_CHECK(wifi_sta_udp_start());

    ESP_LOGI(TAG, "control period: %d ms", BALANCIN_CONTROL_PERIOD_MS);
    xTaskCreatePinnedToCore(control_task, "control", 4096, NULL, 15, NULL, 1);
}
