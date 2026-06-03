#include <ctype.h>
#include <inttypes.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/ledc.h"
#include "driver/pulse_cnt.h"
#include "driver/uart.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/*
 * Mapeo base tomado de _TEORIA/MAPEO DE PINES.xlsx y corregido
 * con pruebas fisicas de sensores/encoders.
 * ESP32 DevKit V1 30 pins.
 */
#define PIN_I2C_SDA GPIO_NUM_21
#define PIN_I2C_SCL GPIO_NUM_22

#define PIN_LINE_LEFT  GPIO_NUM_35
#define PIN_LINE_RIGHT GPIO_NUM_34

#define PIN_ENCODER_RIGHT_A GPIO_NUM_33
#define PIN_ENCODER_RIGHT_B GPIO_NUM_32
#define PIN_ENCODER_LEFT_A  GPIO_NUM_18
#define PIN_ENCODER_LEFT_B  GPIO_NUM_19

/* L298N connector order: ENA, IN1, IN2, IN3, IN4, ENB. */
#define PIN_MOTOR_LEFT_PWM  GPIO_NUM_14
#define PIN_MOTOR_LEFT_IN1  GPIO_NUM_5
#define PIN_MOTOR_LEFT_IN2  GPIO_NUM_4
#define PIN_MOTOR_RIGHT_IN1 GPIO_NUM_2
#define PIN_MOTOR_RIGHT_IN2 GPIO_NUM_15
#define PIN_MOTOR_RIGHT_PWM GPIO_NUM_27

#define DEFAULT_LEFT_MOTOR_INVERT   1
#define DEFAULT_RIGHT_MOTOR_INVERT  0
#define DEFAULT_LEFT_ENCODER_INVERT 0
#define DEFAULT_RIGHT_ENCODER_INVERT 1

#define SERIAL_BAUD_RATE 115200
#define UART_RX_BUF_SIZE 512
#define COMMAND_LINE_SIZE 128

#define MOTOR_PWM_FREQ_HZ 20000
#define MOTOR_PWM_RES LEDC_TIMER_8_BIT
#define MOTOR_PWM_TIMER LEDC_TIMER_0
#define MOTOR_PWM_MODE LEDC_LOW_SPEED_MODE
#define MOTOR_LEFT_CHANNEL LEDC_CHANNEL_0
#define MOTOR_RIGHT_CHANNEL LEDC_CHANNEL_1
#define MOTOR_PWM_MAX 255
#define MOTOR_SUPPLY_VOLTAGE 11.0f
#define MOTOR_DEFAULT_TIMEOUT_MS 2500

#define PCNT_HIGH_LIMIT 1000
#define PCNT_LOW_LIMIT  -1000

#define MPU6050_ADDR       0x68
#define MPU6050_REG_WHOAMI 0x75
#define MPU6050_REG_CONFIG 0x1A
#define MPU6050_REG_GYRO   0x1B
#define MPU6050_REG_ACCEL  0x1C
#define MPU6050_REG_PWR1   0x6B
#define MPU6050_REG_PWR2   0x6C
#define MPU6050_REG_DATA   0x3B

#define ACCEL_LSB_PER_G 16384.0f
#define GYRO_LSB_PER_DPS 131.0f
#define RAD_TO_DEG 57.2957795f

static const char *TAG = "pruebas_balancin";

typedef enum {
    SIDE_LEFT,
    SIDE_RIGHT,
    SIDE_BOTH,
} side_t;

typedef struct {
    bool motor_ready;
    bool line_ready;
    bool encoder_ready;
    bool mpu_ready;

    bool left_motor_invert;
    bool right_motor_invert;
    bool left_encoder_invert;
    bool right_encoder_invert;

    int left_pwm;
    int right_pwm;
    int64_t motor_stop_at_us;
    int motor_timeout_ms;

    bool stream_enabled;
    uint32_t stream_period_ms;

    bool have_white_cal;
    bool have_black_cal;
    uint16_t white_left;
    uint16_t white_right;
    uint16_t black_left;
    uint16_t black_right;
} test_state_t;

typedef struct {
    int16_t ax_raw;
    int16_t ay_raw;
    int16_t az_raw;
    int16_t temp_raw;
    int16_t gx_raw;
    int16_t gy_raw;
    int16_t gz_raw;
    float ax_g;
    float ay_g;
    float az_g;
    float gx_dps;
    float gy_dps;
    float gz_dps;
    float temp_c;
    float accel_roll_deg;
    float accel_pitch_deg;
    float gyro_roll_deg;
    float gyro_pitch_deg;
    float gyro_yaw_deg;
    float comp_roll_deg;
    float comp_pitch_deg;
} mpu_sample_t;

static test_state_t state = {
    .left_motor_invert = DEFAULT_LEFT_MOTOR_INVERT,
    .right_motor_invert = DEFAULT_RIGHT_MOTOR_INVERT,
    .left_encoder_invert = DEFAULT_LEFT_ENCODER_INVERT,
    .right_encoder_invert = DEFAULT_RIGHT_ENCODER_INVERT,
    .motor_timeout_ms = MOTOR_DEFAULT_TIMEOUT_MS,
    .stream_enabled = true,
    .stream_period_ms = 250,
};

static adc_oneshot_unit_handle_t adc_handle;
static adc_channel_t line_left_channel;
static adc_channel_t line_right_channel;

static pcnt_unit_handle_t pcnt_right_unit;
static pcnt_unit_handle_t pcnt_left_unit;

static i2c_master_bus_handle_t i2c_bus;
static i2c_master_dev_handle_t mpu_dev;

static float gyro_offset_x_dps;
static float gyro_offset_y_dps;
static float gyro_offset_z_dps;
static float gyro_roll_deg;
static float gyro_pitch_deg;
static float gyro_yaw_deg;
static float comp_roll_deg;
static float comp_pitch_deg;
static int64_t mpu_last_us;

static int clamp_int(int value, int lo, int hi)
{
    if (value < lo) {
        return lo;
    }
    if (value > hi) {
        return hi;
    }
    return value;
}

static float clamp_float(float value, float lo, float hi)
{
    if (value < lo) {
        return lo;
    }
    if (value > hi) {
        return hi;
    }
    return value;
}

static int pwm_from_voltage(float volts)
{
    float limited = clamp_float(volts, -MOTOR_SUPPLY_VOLTAGE, MOTOR_SUPPLY_VOLTAGE);
    int pwm = (int)lrintf((limited / MOTOR_SUPPLY_VOLTAGE) * MOTOR_PWM_MAX);
    return clamp_int(pwm, -MOTOR_PWM_MAX, MOTOR_PWM_MAX);
}

static const char *side_name(side_t side)
{
    if (side == SIDE_LEFT) {
        return "izquierdo";
    }
    if (side == SIDE_RIGHT) {
        return "derecho";
    }
    return "ambos";
}

static bool parse_side(const char *text, side_t *side)
{
    if (text == NULL) {
        return false;
    }
    if (strcasecmp(text, "l") == 0 || strcasecmp(text, "left") == 0 ||
        strcasecmp(text, "izq") == 0 || strcasecmp(text, "izquierdo") == 0) {
        *side = SIDE_LEFT;
        return true;
    }
    if (strcasecmp(text, "r") == 0 || strcasecmp(text, "right") == 0 ||
        strcasecmp(text, "der") == 0 || strcasecmp(text, "derecho") == 0) {
        *side = SIDE_RIGHT;
        return true;
    }
    if (strcasecmp(text, "b") == 0 || strcasecmp(text, "both") == 0 ||
        strcasecmp(text, "ambos") == 0 || strcasecmp(text, "all") == 0) {
        *side = SIDE_BOTH;
        return true;
    }
    return false;
}

static esp_err_t motor_init(void)
{
    gpio_config_t gpio = {
        .pin_bit_mask = (1ULL << PIN_MOTOR_LEFT_IN1) |
                        (1ULL << PIN_MOTOR_LEFT_IN2) |
                        (1ULL << PIN_MOTOR_RIGHT_IN1) |
                        (1ULL << PIN_MOTOR_RIGHT_IN2),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&gpio), TAG, "motor GPIO config failed");

    ledc_timer_config_t timer = {
        .speed_mode = MOTOR_PWM_MODE,
        .duty_resolution = MOTOR_PWM_RES,
        .timer_num = MOTOR_PWM_TIMER,
        .freq_hz = MOTOR_PWM_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_RETURN_ON_ERROR(ledc_timer_config(&timer), TAG, "LEDC timer failed");

    ledc_channel_config_t left = {
        .speed_mode = MOTOR_PWM_MODE,
        .channel = MOTOR_LEFT_CHANNEL,
        .timer_sel = MOTOR_PWM_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = PIN_MOTOR_LEFT_PWM,
        .duty = 0,
        .hpoint = 0,
    };
    ESP_RETURN_ON_ERROR(ledc_channel_config(&left), TAG, "left LEDC failed");

    ledc_channel_config_t right = {
        .speed_mode = MOTOR_PWM_MODE,
        .channel = MOTOR_RIGHT_CHANNEL,
        .timer_sel = MOTOR_PWM_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = PIN_MOTOR_RIGHT_PWM,
        .duty = 0,
        .hpoint = 0,
    };
    ESP_RETURN_ON_ERROR(ledc_channel_config(&right), TAG, "right LEDC failed");

    state.motor_ready = true;
    return ESP_OK;
}

static void motor_write_one(gpio_num_t in1,
                            gpio_num_t in2,
                            ledc_channel_t channel,
                            int pwm,
                            bool invert)
{
    int command = clamp_int(pwm, -MOTOR_PWM_MAX, MOTOR_PWM_MAX);
    bool positive = command > 0;
    if (invert) {
        positive = !positive;
    }

    if (command == 0) {
        gpio_set_level(in1, 0);
        gpio_set_level(in2, 0);
    } else if (positive) {
        gpio_set_level(in1, 0);
        gpio_set_level(in2, 1);
    } else {
        gpio_set_level(in1, 1);
        gpio_set_level(in2, 0);
    }

    ledc_set_duty(MOTOR_PWM_MODE, channel, (uint32_t)abs(command));
    ledc_update_duty(MOTOR_PWM_MODE, channel);
}

static void motor_apply_pwm(int left_pwm, int right_pwm, bool timed)
{
    if (!state.motor_ready) {
        printf("ERR,motores no inicializados\n");
        return;
    }

    state.left_pwm = clamp_int(left_pwm, -MOTOR_PWM_MAX, MOTOR_PWM_MAX);
    state.right_pwm = clamp_int(right_pwm, -MOTOR_PWM_MAX, MOTOR_PWM_MAX);

    motor_write_one(PIN_MOTOR_LEFT_IN1,
                    PIN_MOTOR_LEFT_IN2,
                    MOTOR_LEFT_CHANNEL,
                    state.left_pwm,
                    state.left_motor_invert);
    motor_write_one(PIN_MOTOR_RIGHT_IN1,
                    PIN_MOTOR_RIGHT_IN2,
                    MOTOR_RIGHT_CHANNEL,
                    state.right_pwm,
                    state.right_motor_invert);

    if (timed && (state.left_pwm != 0 || state.right_pwm != 0)) {
        state.motor_stop_at_us = esp_timer_get_time() +
                                 ((int64_t)state.motor_timeout_ms * 1000);
    } else {
        state.motor_stop_at_us = 0;
    }
}

static void motor_stop(void)
{
    motor_apply_pwm(0, 0, false);
}

static void motor_check_timeout(void)
{
    if (state.motor_stop_at_us > 0 && esp_timer_get_time() >= state.motor_stop_at_us) {
        motor_stop();
        printf("MOTOR,auto_stop,timeout_ms=%d\n", state.motor_timeout_ms);
    }
}

static esp_err_t line_adc_init(void)
{
    adc_oneshot_unit_init_cfg_t unit_config = {
        .unit_id = ADC_UNIT_1,
        .clk_src = ADC_RTC_CLK_SRC_DEFAULT,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_RETURN_ON_ERROR(adc_oneshot_new_unit(&unit_config, &adc_handle),
                        TAG, "ADC unit failed");

    adc_unit_t unit_left;
    adc_unit_t unit_right;
    ESP_RETURN_ON_ERROR(adc_oneshot_io_to_channel(PIN_LINE_LEFT,
                                                  &unit_left,
                                                  &line_left_channel),
                        TAG, "left line GPIO to ADC failed");
    ESP_RETURN_ON_ERROR(adc_oneshot_io_to_channel(PIN_LINE_RIGHT,
                                                  &unit_right,
                                                  &line_right_channel),
                        TAG, "right line GPIO to ADC failed");

    if (unit_left != ADC_UNIT_1 || unit_right != ADC_UNIT_1) {
        return ESP_ERR_INVALID_ARG;
    }

    adc_oneshot_chan_cfg_t channel_config = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };
    ESP_RETURN_ON_ERROR(adc_oneshot_config_channel(adc_handle,
                                                   line_left_channel,
                                                   &channel_config),
                        TAG, "left line ADC config failed");
    ESP_RETURN_ON_ERROR(adc_oneshot_config_channel(adc_handle,
                                                   line_right_channel,
                                                   &channel_config),
                        TAG, "right line ADC config failed");

    state.line_ready = true;
    return ESP_OK;
}

static esp_err_t line_adc_read(uint16_t *left_raw, uint16_t *right_raw)
{
    int left = 0;
    int right = 0;

    if (!state.line_ready) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_RETURN_ON_ERROR(adc_oneshot_read(adc_handle, line_left_channel, &left),
                        TAG, "left ADC read failed");
    ESP_RETURN_ON_ERROR(adc_oneshot_read(adc_handle, line_right_channel, &right),
                        TAG, "right ADC read failed");

    *left_raw = (uint16_t)clamp_int(left, 0, 4095);
    *right_raw = (uint16_t)clamp_int(right, 0, 4095);
    return ESP_OK;
}

static esp_err_t encoder_configure_unit(pcnt_unit_handle_t *unit,
                                        gpio_num_t phase_a,
                                        gpio_num_t phase_b)
{
    gpio_set_pull_mode(phase_a, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(phase_b, GPIO_PULLUP_ONLY);

    pcnt_unit_config_t unit_config = {
        .high_limit = PCNT_HIGH_LIMIT,
        .low_limit = PCNT_LOW_LIMIT,
    };
    ESP_RETURN_ON_ERROR(pcnt_new_unit(&unit_config, unit), TAG, "PCNT unit failed");

    pcnt_glitch_filter_config_t filter = {
        .max_glitch_ns = 1000,
    };
    ESP_RETURN_ON_ERROR(pcnt_unit_set_glitch_filter(*unit, &filter),
                        TAG, "PCNT filter failed");

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
                        TAG, "PCNT channel A failed");
    ESP_RETURN_ON_ERROR(pcnt_new_channel(*unit, &chan_b_config, &chan_b),
                        TAG, "PCNT channel B failed");

    ESP_RETURN_ON_ERROR(pcnt_channel_set_edge_action(chan_a,
                                                     PCNT_CHANNEL_EDGE_ACTION_DECREASE,
                                                     PCNT_CHANNEL_EDGE_ACTION_INCREASE),
                        TAG, "PCNT edge A failed");
    ESP_RETURN_ON_ERROR(pcnt_channel_set_level_action(chan_a,
                                                      PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                                                      PCNT_CHANNEL_LEVEL_ACTION_INVERSE),
                        TAG, "PCNT level A failed");
    ESP_RETURN_ON_ERROR(pcnt_channel_set_edge_action(chan_b,
                                                     PCNT_CHANNEL_EDGE_ACTION_INCREASE,
                                                     PCNT_CHANNEL_EDGE_ACTION_DECREASE),
                        TAG, "PCNT edge B failed");
    ESP_RETURN_ON_ERROR(pcnt_channel_set_level_action(chan_b,
                                                      PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                                                      PCNT_CHANNEL_LEVEL_ACTION_INVERSE),
                        TAG, "PCNT level B failed");

    ESP_RETURN_ON_ERROR(pcnt_unit_enable(*unit), TAG, "PCNT enable failed");
    ESP_RETURN_ON_ERROR(pcnt_unit_clear_count(*unit), TAG, "PCNT clear failed");
    ESP_RETURN_ON_ERROR(pcnt_unit_start(*unit), TAG, "PCNT start failed");

    return ESP_OK;
}

static esp_err_t encoder_init(void)
{
    ESP_RETURN_ON_ERROR(encoder_configure_unit(&pcnt_right_unit,
                                               PIN_ENCODER_RIGHT_A,
                                               PIN_ENCODER_RIGHT_B),
                        TAG, "right encoder failed");
    ESP_RETURN_ON_ERROR(encoder_configure_unit(&pcnt_left_unit,
                                               PIN_ENCODER_LEFT_A,
                                               PIN_ENCODER_LEFT_B),
                        TAG, "left encoder failed");

    state.encoder_ready = true;
    return ESP_OK;
}

static esp_err_t encoder_read_delta(int *right_delta, int *left_delta)
{
    if (!state.encoder_ready) {
        return ESP_ERR_INVALID_STATE;
    }

    int right = 0;
    int left = 0;
    ESP_RETURN_ON_ERROR(pcnt_unit_get_count(pcnt_right_unit, &right),
                        TAG, "right PCNT read failed");
    ESP_RETURN_ON_ERROR(pcnt_unit_get_count(pcnt_left_unit, &left),
                        TAG, "left PCNT read failed");
    ESP_RETURN_ON_ERROR(pcnt_unit_clear_count(pcnt_right_unit),
                        TAG, "right PCNT clear failed");
    ESP_RETURN_ON_ERROR(pcnt_unit_clear_count(pcnt_left_unit),
                        TAG, "left PCNT clear failed");

    if (state.right_encoder_invert) {
        right = -right;
    }
    if (state.left_encoder_invert) {
        left = -left;
    }

    *right_delta = right;
    *left_delta = left;
    return ESP_OK;
}

static esp_err_t mpu_write_reg(uint8_t reg, uint8_t value)
{
    uint8_t data[2] = {reg, value};
    return i2c_master_transmit(mpu_dev, data, sizeof(data), 100);
}

static esp_err_t mpu_read_reg(uint8_t reg, uint8_t *value)
{
    return i2c_master_transmit_receive(mpu_dev, &reg, 1, value, 1, 100);
}

static esp_err_t mpu_read_bytes(uint8_t reg, uint8_t *data, size_t len)
{
    return i2c_master_transmit_receive(mpu_dev, &reg, 1, data, len, 100);
}

static esp_err_t mpu_init(void)
{
    i2c_master_bus_config_t bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_0,
        .scl_io_num = PIN_I2C_SCL,
        .sda_io_num = PIN_I2C_SDA,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_RETURN_ON_ERROR(i2c_new_master_bus(&bus_config, &i2c_bus),
                        TAG, "I2C bus failed");

    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = MPU6050_ADDR,
        .scl_speed_hz = 400000,
    };
    ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(i2c_bus, &dev_config, &mpu_dev),
                        TAG, "MPU I2C device failed");

    uint8_t whoami = 0;
    ESP_RETURN_ON_ERROR(mpu_read_reg(MPU6050_REG_WHOAMI, &whoami),
                        TAG, "MPU WHO_AM_I failed");

    ESP_RETURN_ON_ERROR(mpu_write_reg(MPU6050_REG_PWR1, 0x80),
                        TAG, "MPU reset failed");
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_RETURN_ON_ERROR(mpu_write_reg(MPU6050_REG_PWR1, 0x00),
                        TAG, "MPU wake failed");
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_RETURN_ON_ERROR(mpu_write_reg(MPU6050_REG_PWR2, 0x00),
                        TAG, "MPU pwr2 failed");
    ESP_RETURN_ON_ERROR(mpu_write_reg(MPU6050_REG_CONFIG, 0x03),
                        TAG, "MPU DLPF failed");
    ESP_RETURN_ON_ERROR(mpu_write_reg(MPU6050_REG_GYRO, 0x00),
                        TAG, "MPU gyro config failed");
    ESP_RETURN_ON_ERROR(mpu_write_reg(MPU6050_REG_ACCEL, 0x00),
                        TAG, "MPU accel config failed");

    mpu_last_us = esp_timer_get_time();
    state.mpu_ready = true;
    printf("MPU,WHO_AM_I,0x%02X\n", whoami);
    return ESP_OK;
}

static esp_err_t mpu_read_sample(mpu_sample_t *sample)
{
    uint8_t data[14] = {0};

    if (!state.mpu_ready) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_RETURN_ON_ERROR(mpu_read_bytes(MPU6050_REG_DATA, data, sizeof(data)),
                        TAG, "MPU data read failed");

    sample->ax_raw = (int16_t)((data[0] << 8) | data[1]);
    sample->ay_raw = (int16_t)((data[2] << 8) | data[3]);
    sample->az_raw = (int16_t)((data[4] << 8) | data[5]);
    sample->temp_raw = (int16_t)((data[6] << 8) | data[7]);
    sample->gx_raw = (int16_t)((data[8] << 8) | data[9]);
    sample->gy_raw = (int16_t)((data[10] << 8) | data[11]);
    sample->gz_raw = (int16_t)((data[12] << 8) | data[13]);

    sample->ax_g = sample->ax_raw / ACCEL_LSB_PER_G;
    sample->ay_g = sample->ay_raw / ACCEL_LSB_PER_G;
    sample->az_g = sample->az_raw / ACCEL_LSB_PER_G;
    sample->gx_dps = (sample->gx_raw / GYRO_LSB_PER_DPS) - gyro_offset_x_dps;
    sample->gy_dps = (sample->gy_raw / GYRO_LSB_PER_DPS) - gyro_offset_y_dps;
    sample->gz_dps = (sample->gz_raw / GYRO_LSB_PER_DPS) - gyro_offset_z_dps;
    sample->temp_c = (sample->temp_raw / 340.0f) + 36.53f;

    sample->accel_roll_deg = atan2f(sample->ay_g, sample->az_g) * RAD_TO_DEG;
    sample->accel_pitch_deg = atan2f(-sample->ax_g,
                                     sqrtf(sample->ay_g * sample->ay_g +
                                           sample->az_g * sample->az_g)) *
                              RAD_TO_DEG;

    int64_t now_us = esp_timer_get_time();
    float dt = (now_us - mpu_last_us) / 1000000.0f;
    if (dt <= 0.0f || dt > 1.0f) {
        dt = 0.01f;
    }
    mpu_last_us = now_us;

    gyro_roll_deg += sample->gx_dps * dt;
    gyro_pitch_deg += sample->gy_dps * dt;
    gyro_yaw_deg += sample->gz_dps * dt;
    comp_roll_deg = 0.98f * (comp_roll_deg + sample->gx_dps * dt) +
                    0.02f * sample->accel_roll_deg;
    comp_pitch_deg = 0.98f * (comp_pitch_deg + sample->gy_dps * dt) +
                     0.02f * sample->accel_pitch_deg;

    sample->gyro_roll_deg = gyro_roll_deg;
    sample->gyro_pitch_deg = gyro_pitch_deg;
    sample->gyro_yaw_deg = gyro_yaw_deg;
    sample->comp_roll_deg = comp_roll_deg;
    sample->comp_pitch_deg = comp_pitch_deg;

    return ESP_OK;
}

static esp_err_t mpu_calibrate(uint32_t samples)
{
    if (!state.mpu_ready) {
        return ESP_ERR_INVALID_STATE;
    }

    if (samples < 100) {
        samples = 100;
    }
    if (samples > 2000) {
        samples = 2000;
    }

    int64_t sum_gx = 0;
    int64_t sum_gy = 0;
    int64_t sum_gz = 0;
    mpu_sample_t sample = {0};

    printf("MPU,CALIBRANDO,muestras=%" PRIu32 ",mantener quieto\n", samples);
    for (uint32_t i = 0; i < samples; i++) {
        uint8_t data[14] = {0};
        esp_err_t err = mpu_read_bytes(MPU6050_REG_DATA, data, sizeof(data));
        if (err != ESP_OK) {
            return err;
        }
        int16_t gx = (int16_t)((data[8] << 8) | data[9]);
        int16_t gy = (int16_t)((data[10] << 8) | data[11]);
        int16_t gz = (int16_t)((data[12] << 8) | data[13]);
        sum_gx += gx;
        sum_gy += gy;
        sum_gz += gz;
        vTaskDelay(pdMS_TO_TICKS(3));
    }

    gyro_offset_x_dps = ((float)sum_gx / (float)samples) / GYRO_LSB_PER_DPS;
    gyro_offset_y_dps = ((float)sum_gy / (float)samples) / GYRO_LSB_PER_DPS;
    gyro_offset_z_dps = ((float)sum_gz / (float)samples) / GYRO_LSB_PER_DPS;

    gyro_roll_deg = 0.0f;
    gyro_pitch_deg = 0.0f;
    gyro_yaw_deg = 0.0f;
    mpu_last_us = esp_timer_get_time();
    ESP_RETURN_ON_ERROR(mpu_read_sample(&sample), TAG, "MPU read after cal failed");
    comp_roll_deg = sample.accel_roll_deg;
    comp_pitch_deg = sample.accel_pitch_deg;

    printf("MPU,CAL_OK,off_gx=%.4f,off_gy=%.4f,off_gz=%.4f,acc_roll=%.2f,acc_pitch=%.2f\n",
           gyro_offset_x_dps,
           gyro_offset_y_dps,
           gyro_offset_z_dps,
           sample.accel_roll_deg,
           sample.accel_pitch_deg);
    return ESP_OK;
}

static void print_pin_map(void)
{
    printf("\nPINMAP,_TEORIA/MAPEO_DE_PINES.xlsx\n");
    printf("PINMAP,MPU_SDA,D21,GPIO%d\n", PIN_I2C_SDA);
    printf("PINMAP,MPU_SCL,D22,GPIO%d\n", PIN_I2C_SCL);
    printf("PINMAP,TCRT5000_L_A0,D35,GPIO%d\n", PIN_LINE_LEFT);
    printf("PINMAP,TCRT5000_R_A0,D34,GPIO%d\n", PIN_LINE_RIGHT);
    printf("PINMAP,ENCODER_R_A,D33,GPIO%d\n", PIN_ENCODER_RIGHT_A);
    printf("PINMAP,ENCODER_R_B,D32,GPIO%d\n", PIN_ENCODER_RIGHT_B);
    printf("PINMAP,ENCODER_L_A,D18,GPIO%d\n", PIN_ENCODER_LEFT_A);
    printf("PINMAP,ENCODER_L_B,D19,GPIO%d\n", PIN_ENCODER_LEFT_B);
    printf("PINMAP,L298N_ENA_PWM_L,D14,GPIO%d\n", PIN_MOTOR_LEFT_PWM);
    printf("PINMAP,L298N_IN1_L,D5,GPIO%d\n", PIN_MOTOR_LEFT_IN1);
    printf("PINMAP,L298N_IN2_L,D4,GPIO%d\n", PIN_MOTOR_LEFT_IN2);
    printf("PINMAP,L298N_IN3_R,D2,GPIO%d\n", PIN_MOTOR_RIGHT_IN1);
    printf("PINMAP,L298N_IN4_R,D15,GPIO%d\n", PIN_MOTOR_RIGHT_IN2);
    printf("PINMAP,L298N_ENB_PWM_R,D27,GPIO%d\n", PIN_MOTOR_RIGHT_PWM);
    printf("PINMAP,NOTA,GPIO2 GPIO5 GPIO15 son pines de arranque del ESP32\n\n");
}

static void print_help(void)
{
    printf("\n");
    printf("=== MONITOR DE PRUEBAS BALANCIN ===\n");
    printf("help                         muestra esta ayuda\n");
    printf("pins                         muestra el mapeo de pines de _TEORIA\n");
    printf("status                       muestra estado de drivers e inversiones\n");
    printf("stream on|off                activa/desactiva telemetria CSV\n");
    printf("rate <ms>                    periodo de telemetria, ejemplo: rate 100\n");
    printf("stop                         apaga ambos motores\n");
    printf("timeout <ms>                 auto-stop de motores, ejemplo: timeout 1500\n");
    printf("motor <l|r|b> <V> [V_der]    aplica voltaje PWM equivalente\n");
    printf("pwm <l|r|b> <pwm> [pwm_der]  aplica PWM signed -255..255\n");
    printf("pulse <l|r|b> <V> <ms>       pulso seguro y luego stop\n");
    printf("testmotor <l|r> <V> <ms>     pulso + lectura de encoder\n");
    printf("inv motor <l|r> <0|1>        cambia inversion de motor en RAM\n");
    printf("inv enc <l|r> <0|1>          cambia inversion de encoder en RAM\n");
    printf("ir                           lee sensores infrarrojos una vez\n");
    printf("ircal blanco <ms>            promedio/min/max sobre linea blanca\n");
    printf("ircal negro <ms>             promedio/min/max sobre pista negra\n");
    printf("threshold                    sugiere umbrales blanco/negro guardados\n");
    printf("enc [ms]                     delta de encoders durante el intervalo\n");
    printf("mpu                          lee angulos y crudos del MPU una vez\n");
    printf("mpu zero [muestras]          calibra offsets del giroscopio quieto\n");
    printf("\n");
    printf("Convencion: comando positivo debe ser 'adelante' para cada rueda.\n");
    printf("Si una rueda gira al reves, prueba: inv motor l 1 o inv motor r 0.\n");
    printf("Si encoder positivo no coincide con motor positivo, usa inv enc l/r.\n\n");
}

static void print_status(void)
{
    printf("STATUS,motor=%d,linea=%d,encoder=%d,mpu=%d\n",
           state.motor_ready,
           state.line_ready,
           state.encoder_ready,
           state.mpu_ready);
    printf("STATUS,inv_motor_l=%d,inv_motor_r=%d,inv_enc_l=%d,inv_enc_r=%d\n",
           state.left_motor_invert,
           state.right_motor_invert,
           state.left_encoder_invert,
           state.right_encoder_invert);
    printf("STATUS,pwm_l=%d,pwm_r=%d,timeout_ms=%d,stream=%d,rate_ms=%" PRIu32 "\n",
           state.left_pwm,
           state.right_pwm,
           state.motor_timeout_ms,
           state.stream_enabled,
           state.stream_period_ms);
}

static void print_line_once(void)
{
    uint16_t left = 0;
    uint16_t right = 0;
    esp_err_t err = line_adc_read(&left, &right);
    if (err != ESP_OK) {
        printf("IR,ERR,%s\n", esp_err_to_name(err));
        return;
    }

    printf("IR,raw_l=%u,raw_r=%u,adc8_l=%u,adc8_r=%u\n",
           left,
           right,
           left >> 4,
           right >> 4);
}

static void line_calibrate(const char *label, uint32_t duration_ms)
{
    if (duration_ms < 200) {
        duration_ms = 200;
    }
    if (duration_ms > 10000) {
        duration_ms = 10000;
    }

    uint16_t min_l = 4095;
    uint16_t min_r = 4095;
    uint16_t max_l = 0;
    uint16_t max_r = 0;
    uint64_t sum_l = 0;
    uint64_t sum_r = 0;
    uint32_t count = 0;

    int64_t end_us = esp_timer_get_time() + ((int64_t)duration_ms * 1000);
    printf("IRCAL,%s,inicio_ms=%" PRIu32 "\n", label, duration_ms);
    while (esp_timer_get_time() < end_us) {
        uint16_t left = 0;
        uint16_t right = 0;
        if (line_adc_read(&left, &right) == ESP_OK) {
            if (left < min_l) {
                min_l = left;
            }
            if (left > max_l) {
                max_l = left;
            }
            if (right < min_r) {
                min_r = right;
            }
            if (right > max_r) {
                max_r = right;
            }
            sum_l += left;
            sum_r += right;
            count++;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    if (count == 0) {
        printf("IRCAL,%s,ERR,sin_muestras\n", label);
        return;
    }

    uint16_t avg_l = (uint16_t)(sum_l / count);
    uint16_t avg_r = (uint16_t)(sum_r / count);
    printf("IRCAL,%s,count=%" PRIu32 ",L_min=%u,L_avg=%u,L_max=%u,R_min=%u,R_avg=%u,R_max=%u\n",
           label,
           count,
           min_l,
           avg_l,
           max_l,
           min_r,
           avg_r,
           max_r);

    if (strcasecmp(label, "blanco") == 0 || strcasecmp(label, "white") == 0) {
        state.have_white_cal = true;
        state.white_left = avg_l;
        state.white_right = avg_r;
    } else if (strcasecmp(label, "negro") == 0 || strcasecmp(label, "black") == 0) {
        state.have_black_cal = true;
        state.black_left = avg_l;
        state.black_right = avg_r;
    }
}

static void print_thresholds(void)
{
    if (!state.have_white_cal || !state.have_black_cal) {
        printf("THRESHOLD,ERR,primero usa ircal blanco <ms> e ircal negro <ms>\n");
        return;
    }

    uint16_t left = (uint16_t)(((uint32_t)state.white_left + state.black_left) / 2);
    uint16_t right = (uint16_t)(((uint32_t)state.white_right + state.black_right) / 2);
    printf("THRESHOLD,L=%u,R=%u,white_L=%u,black_L=%u,white_R=%u,black_R=%u\n",
           left,
           right,
           state.white_left,
           state.black_left,
           state.white_right,
           state.black_right);
    printf("THRESHOLD,nota,si negro da mayor lectura que blanco usa raw>umbral para negro; si da menor usa raw<umbral\n");
}

static void print_encoder_delta(uint32_t duration_ms)
{
    if (duration_ms > 0) {
        int dummy_r = 0;
        int dummy_l = 0;
        encoder_read_delta(&dummy_r, &dummy_l);
        vTaskDelay(pdMS_TO_TICKS(duration_ms));
    }

    int right = 0;
    int left = 0;
    esp_err_t err = encoder_read_delta(&right, &left);
    if (err != ESP_OK) {
        printf("ENC,ERR,%s\n", esp_err_to_name(err));
        return;
    }

    printf("ENC,dt_ms=%" PRIu32 ",delta_l=%d,delta_r=%d\n",
           duration_ms,
           left,
           right);
}

static void print_mpu_once(void)
{
    mpu_sample_t sample = {0};
    esp_err_t err = mpu_read_sample(&sample);
    if (err != ESP_OK) {
        printf("MPU,ERR,%s\n", esp_err_to_name(err));
        return;
    }

    printf("MPU,raw,ax=%d,ay=%d,az=%d,gx=%d,gy=%d,gz=%d,temp_raw=%d\n",
           sample.ax_raw,
           sample.ay_raw,
           sample.az_raw,
           sample.gx_raw,
           sample.gy_raw,
           sample.gz_raw,
           sample.temp_raw);
    printf("MPU,unit,ax_g=%.4f,ay_g=%.4f,az_g=%.4f,gx_dps=%.3f,gy_dps=%.3f,gz_dps=%.3f,temp_c=%.2f\n",
           sample.ax_g,
           sample.ay_g,
           sample.az_g,
           sample.gx_dps,
           sample.gy_dps,
           sample.gz_dps,
           sample.temp_c);
    printf("MPU,angle,acc_roll=%.2f,acc_pitch=%.2f,gyro_roll=%.2f,gyro_pitch=%.2f,gyro_yaw=%.2f,comp_roll=%.2f,comp_pitch=%.2f\n",
           sample.accel_roll_deg,
           sample.accel_pitch_deg,
           sample.gyro_roll_deg,
           sample.gyro_pitch_deg,
           sample.gyro_yaw_deg,
           sample.comp_roll_deg,
           sample.comp_pitch_deg);
}

static void run_motor_encoder_test(side_t side, float volts, uint32_t duration_ms)
{
    if (side == SIDE_BOTH) {
        printf("TESTMOTOR,ERR,usa l o r para aislar cada rueda\n");
        return;
    }
    if (duration_ms < 100) {
        duration_ms = 100;
    }
    if (duration_ms > 5000) {
        duration_ms = 5000;
    }

    int pwm = pwm_from_voltage(volts);
    int clear_r = 0;
    int clear_l = 0;
    encoder_read_delta(&clear_r, &clear_l);

    if (side == SIDE_LEFT) {
        motor_apply_pwm(pwm, 0, false);
    } else {
        motor_apply_pwm(0, pwm, false);
    }

    vTaskDelay(pdMS_TO_TICKS(duration_ms));
    motor_stop();
    vTaskDelay(pdMS_TO_TICKS(50));

    int right = 0;
    int left = 0;
    esp_err_t err = encoder_read_delta(&right, &left);
    if (err != ESP_OK) {
        printf("TESTMOTOR,%s,ERR,%s\n", side_name(side), esp_err_to_name(err));
        return;
    }

    printf("TESTMOTOR,%s,V=%.2f,pwm=%d,dt_ms=%" PRIu32 ",delta_l=%d,delta_r=%d\n",
           side_name(side),
           volts,
           pwm,
           duration_ms,
           left,
           right);
    printf("TESTMOTOR,nota,si la rueda giro adelante pero su delta salio negativo invierte ese encoder\n");
}

static void process_command(char *line)
{
    char *saveptr = NULL;
    char *cmd = strtok_r(line, " \t\r\n", &saveptr);
    if (cmd == NULL) {
        return;
    }

    if (strcasecmp(cmd, "help") == 0 || strcmp(cmd, "?") == 0) {
        print_help();
    } else if (strcasecmp(cmd, "pins") == 0) {
        print_pin_map();
    } else if (strcasecmp(cmd, "status") == 0) {
        print_status();
    } else if (strcasecmp(cmd, "stream") == 0) {
        char *arg = strtok_r(NULL, " \t\r\n", &saveptr);
        if (arg == NULL) {
            printf("STREAM,%d\n", state.stream_enabled);
        } else if (strcasecmp(arg, "on") == 0 || strcmp(arg, "1") == 0) {
            state.stream_enabled = true;
            printf("STREAM,on\n");
        } else if (strcasecmp(arg, "off") == 0 || strcmp(arg, "0") == 0) {
            state.stream_enabled = false;
            printf("STREAM,off\n");
        } else {
            printf("ERR,uso: stream on|off\n");
        }
    } else if (strcasecmp(cmd, "rate") == 0) {
        char *arg = strtok_r(NULL, " \t\r\n", &saveptr);
        if (arg == NULL) {
            printf("RATE,%" PRIu32 "\n", state.stream_period_ms);
        } else {
            uint32_t ms = (uint32_t)strtoul(arg, NULL, 10);
            state.stream_period_ms = clamp_int((int)ms, 50, 5000);
            printf("RATE,%" PRIu32 "\n", state.stream_period_ms);
        }
    } else if (strcasecmp(cmd, "stop") == 0) {
        motor_stop();
        printf("MOTOR,stop\n");
    } else if (strcasecmp(cmd, "timeout") == 0) {
        char *arg = strtok_r(NULL, " \t\r\n", &saveptr);
        if (arg == NULL) {
            printf("TIMEOUT,%d\n", state.motor_timeout_ms);
        } else {
            int ms = atoi(arg);
            state.motor_timeout_ms = clamp_int(ms, 100, 10000);
            printf("TIMEOUT,%d\n", state.motor_timeout_ms);
        }
    } else if (strcasecmp(cmd, "motor") == 0 || strcasecmp(cmd, "m") == 0) {
        char *side_text = strtok_r(NULL, " \t\r\n", &saveptr);
        char *v1_text = strtok_r(NULL, " \t\r\n", &saveptr);
        char *v2_text = strtok_r(NULL, " \t\r\n", &saveptr);
        side_t side = SIDE_BOTH;
        if (!parse_side(side_text, &side) || v1_text == NULL) {
            printf("ERR,uso: motor <l|r|b> <V> [V_der]\n");
            return;
        }
        float v1 = strtof(v1_text, NULL);
        float v2 = v2_text ? strtof(v2_text, NULL) : v1;
        int left = state.left_pwm;
        int right = state.right_pwm;
        if (side == SIDE_LEFT) {
            left = pwm_from_voltage(v1);
            right = 0;
        } else if (side == SIDE_RIGHT) {
            left = 0;
            right = pwm_from_voltage(v1);
        } else {
            left = pwm_from_voltage(v1);
            right = pwm_from_voltage(v2);
        }
        motor_apply_pwm(left, right, true);
        printf("MOTOR,V,l=%.2f,r=%.2f,pwm_l=%d,pwm_r=%d,auto_stop_ms=%d\n",
               (left * MOTOR_SUPPLY_VOLTAGE) / MOTOR_PWM_MAX,
               (right * MOTOR_SUPPLY_VOLTAGE) / MOTOR_PWM_MAX,
               left,
               right,
               state.motor_timeout_ms);
    } else if (strcasecmp(cmd, "pwm") == 0) {
        char *side_text = strtok_r(NULL, " \t\r\n", &saveptr);
        char *p1_text = strtok_r(NULL, " \t\r\n", &saveptr);
        char *p2_text = strtok_r(NULL, " \t\r\n", &saveptr);
        side_t side = SIDE_BOTH;
        if (!parse_side(side_text, &side) || p1_text == NULL) {
            printf("ERR,uso: pwm <l|r|b> <pwm> [pwm_der]\n");
            return;
        }
        int p1 = atoi(p1_text);
        int p2 = p2_text ? atoi(p2_text) : p1;
        int left = 0;
        int right = 0;
        if (side == SIDE_LEFT) {
            left = p1;
        } else if (side == SIDE_RIGHT) {
            right = p1;
        } else {
            left = p1;
            right = p2;
        }
        motor_apply_pwm(left, right, true);
        printf("MOTOR,PWM,l=%d,r=%d,auto_stop_ms=%d\n",
               state.left_pwm,
               state.right_pwm,
               state.motor_timeout_ms);
    } else if (strcasecmp(cmd, "pulse") == 0) {
        char *side_text = strtok_r(NULL, " \t\r\n", &saveptr);
        char *v_text = strtok_r(NULL, " \t\r\n", &saveptr);
        char *ms_text = strtok_r(NULL, " \t\r\n", &saveptr);
        side_t side = SIDE_BOTH;
        if (!parse_side(side_text, &side) || v_text == NULL || ms_text == NULL) {
            printf("ERR,uso: pulse <l|r|b> <V> <ms>\n");
            return;
        }
        float volts = strtof(v_text, NULL);
        uint32_t duration_ms = clamp_int(atoi(ms_text), 50, 5000);
        int pwm = pwm_from_voltage(volts);
        int left = (side == SIDE_RIGHT) ? 0 : pwm;
        int right = (side == SIDE_LEFT) ? 0 : pwm;
        printf("PULSE,%s,V=%.2f,pwm=%d,dt_ms=%" PRIu32 "\n",
               side_name(side),
               volts,
               pwm,
               duration_ms);
        motor_apply_pwm(left, right, false);
        vTaskDelay(pdMS_TO_TICKS(duration_ms));
        motor_stop();
        printf("PULSE,stop\n");
    } else if (strcasecmp(cmd, "testmotor") == 0) {
        char *side_text = strtok_r(NULL, " \t\r\n", &saveptr);
        char *v_text = strtok_r(NULL, " \t\r\n", &saveptr);
        char *ms_text = strtok_r(NULL, " \t\r\n", &saveptr);
        side_t side = SIDE_BOTH;
        if (!parse_side(side_text, &side) || v_text == NULL || ms_text == NULL) {
            printf("ERR,uso: testmotor <l|r> <V> <ms>\n");
            return;
        }
        run_motor_encoder_test(side,
                               strtof(v_text, NULL),
                               (uint32_t)atoi(ms_text));
    } else if (strcasecmp(cmd, "inv") == 0) {
        char *what = strtok_r(NULL, " \t\r\n", &saveptr);
        char *side_text = strtok_r(NULL, " \t\r\n", &saveptr);
        char *value_text = strtok_r(NULL, " \t\r\n", &saveptr);
        side_t side = SIDE_BOTH;
        if (what == NULL || !parse_side(side_text, &side) ||
            value_text == NULL || side == SIDE_BOTH) {
            printf("ERR,uso: inv motor|enc <l|r> <0|1>\n");
            return;
        }
        bool value = atoi(value_text) != 0;
        if (strcasecmp(what, "motor") == 0 || strcasecmp(what, "m") == 0) {
            if (side == SIDE_LEFT) {
                state.left_motor_invert = value;
            } else {
                state.right_motor_invert = value;
            }
            printf("INV,motor_%s=%d\n", side_name(side), value);
        } else if (strcasecmp(what, "enc") == 0 || strcasecmp(what, "encoder") == 0) {
            if (side == SIDE_LEFT) {
                state.left_encoder_invert = value;
            } else {
                state.right_encoder_invert = value;
            }
            printf("INV,enc_%s=%d\n", side_name(side), value);
        } else {
            printf("ERR,uso: inv motor|enc <l|r> <0|1>\n");
        }
    } else if (strcasecmp(cmd, "ir") == 0) {
        print_line_once();
    } else if (strcasecmp(cmd, "ircal") == 0) {
        char *label = strtok_r(NULL, " \t\r\n", &saveptr);
        char *ms_text = strtok_r(NULL, " \t\r\n", &saveptr);
        if (label == NULL) {
            printf("ERR,uso: ircal blanco|negro <ms>\n");
            return;
        }
        uint32_t duration_ms = ms_text ? (uint32_t)atoi(ms_text) : 3000;
        line_calibrate(label, duration_ms);
    } else if (strcasecmp(cmd, "threshold") == 0 ||
               strcasecmp(cmd, "umbrales") == 0) {
        print_thresholds();
    } else if (strcasecmp(cmd, "enc") == 0) {
        char *ms_text = strtok_r(NULL, " \t\r\n", &saveptr);
        uint32_t duration_ms = ms_text ? (uint32_t)atoi(ms_text) : 0;
        print_encoder_delta(duration_ms);
    } else if (strcasecmp(cmd, "mpu") == 0) {
        char *arg = strtok_r(NULL, " \t\r\n", &saveptr);
        if (arg != NULL && (strcasecmp(arg, "zero") == 0 ||
                            strcasecmp(arg, "cal") == 0)) {
            char *samples_text = strtok_r(NULL, " \t\r\n", &saveptr);
            uint32_t samples = samples_text ? (uint32_t)atoi(samples_text) : 500;
            esp_err_t err = mpu_calibrate(samples);
            if (err != ESP_OK) {
                printf("MPU,CAL_ERR,%s\n", esp_err_to_name(err));
            }
        } else {
            print_mpu_once();
        }
    } else {
        printf("ERR,comando desconocido: %s. Usa help\n", cmd);
    }
}

static void command_task(void *arg)
{
    (void)arg;

    char line[COMMAND_LINE_SIZE] = {0};
    size_t len = 0;
    uint8_t ch = 0;

    print_help();
    printf("> ");
    fflush(stdout);

    for (;;) {
        int n = uart_read_bytes(UART_NUM_0, &ch, 1, pdMS_TO_TICKS(100));
        if (n <= 0) {
            continue;
        }

        if (ch == '\r' || ch == '\n') {
            if (len > 0) {
                line[len] = '\0';
                printf("\n");
                process_command(line);
                len = 0;
                memset(line, 0, sizeof(line));
            }
            printf("> ");
            fflush(stdout);
        } else if (ch == 0x08 || ch == 0x7F) {
            if (len > 0) {
                len--;
                printf("\b \b");
                fflush(stdout);
            }
        } else if (isprint((int)ch) && len < sizeof(line) - 1) {
            line[len++] = (char)ch;
            printf("%c", ch);
            fflush(stdout);
        }
    }
}

static void telemetry_task(void *arg)
{
    (void)arg;

    printf("CSV_HEADER,t_ms,ir_l,ir_r,ir8_l,ir8_r,enc_l,enc_r,ax_raw,ay_raw,az_raw,gx_raw,gy_raw,gz_raw,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps,acc_roll,acc_pitch,comp_roll,comp_pitch,pwm_l,pwm_r\n");

    for (;;) {
        motor_check_timeout();

        if (state.stream_enabled) {
            uint16_t ir_l = 0;
            uint16_t ir_r = 0;
            int enc_l = 0;
            int enc_r = 0;
            mpu_sample_t mpu = {0};
            bool line_ok = line_adc_read(&ir_l, &ir_r) == ESP_OK;
            bool enc_ok = encoder_read_delta(&enc_r, &enc_l) == ESP_OK;
            bool mpu_ok = mpu_read_sample(&mpu) == ESP_OK;

            printf("CSV,%" PRIu64 ",",
                   (uint64_t)(esp_timer_get_time() / 1000));
            printf("%u,%u,%u,%u,",
                   line_ok ? ir_l : 0,
                   line_ok ? ir_r : 0,
                   line_ok ? (ir_l >> 4) : 0,
                   line_ok ? (ir_r >> 4) : 0);
            printf("%d,%d,",
                   enc_ok ? enc_l : 0,
                   enc_ok ? enc_r : 0);
            printf("%d,%d,%d,%d,%d,%d,",
                   mpu_ok ? mpu.ax_raw : 0,
                   mpu_ok ? mpu.ay_raw : 0,
                   mpu_ok ? mpu.az_raw : 0,
                   mpu_ok ? mpu.gx_raw : 0,
                   mpu_ok ? mpu.gy_raw : 0,
                   mpu_ok ? mpu.gz_raw : 0);
            printf("%.4f,%.4f,%.4f,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,%.2f,%d,%d\n",
                   mpu_ok ? mpu.ax_g : 0.0f,
                   mpu_ok ? mpu.ay_g : 0.0f,
                   mpu_ok ? mpu.az_g : 0.0f,
                   mpu_ok ? mpu.gx_dps : 0.0f,
                   mpu_ok ? mpu.gy_dps : 0.0f,
                   mpu_ok ? mpu.gz_dps : 0.0f,
                   mpu_ok ? mpu.accel_roll_deg : 0.0f,
                   mpu_ok ? mpu.accel_pitch_deg : 0.0f,
                   mpu_ok ? mpu.comp_roll_deg : 0.0f,
                   mpu_ok ? mpu.comp_pitch_deg : 0.0f,
                   state.left_pwm,
                   state.right_pwm);
        }

        vTaskDelay(pdMS_TO_TICKS(state.stream_period_ms));
    }
}

static void serial_init(void)
{
    setvbuf(stdin, NULL, _IONBF, 0);
    setvbuf(stdout, NULL, _IONBF, 0);

    uart_config_t uart_config = {
        .baud_rate = SERIAL_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t err = uart_driver_install(UART_NUM_0,
                                        UART_RX_BUF_SIZE,
                                        0,
                                        0,
                                        NULL,
                                        0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "uart_driver_install: %s", esp_err_to_name(err));
    }
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_0, &uart_config));
}

void app_main(void)
{
    serial_init();

    printf("\n\n");
    printf("Balancin ESP32 - firmware de pruebas\n");
    printf("Baud: %d\n", SERIAL_BAUD_RATE);
    print_pin_map();

    esp_err_t err = motor_init();
    printf("INIT,motor,%s\n", esp_err_to_name(err));
    if (err == ESP_OK) {
        motor_stop();
    }

    err = line_adc_init();
    printf("INIT,linea,%s\n", esp_err_to_name(err));

    err = encoder_init();
    printf("INIT,encoder,%s\n", esp_err_to_name(err));

    err = mpu_init();
    printf("INIT,mpu,%s\n", esp_err_to_name(err));
    if (err == ESP_OK) {
        err = mpu_calibrate(300);
        if (err != ESP_OK) {
            printf("INIT,mpu_cal,%s\n", esp_err_to_name(err));
        }
    }

    xTaskCreatePinnedToCore(command_task, "cmd", 4096, NULL, 8, NULL, 0);
    xTaskCreatePinnedToCore(telemetry_task, "telemetry", 4096, NULL, 5, NULL, 1);
}
