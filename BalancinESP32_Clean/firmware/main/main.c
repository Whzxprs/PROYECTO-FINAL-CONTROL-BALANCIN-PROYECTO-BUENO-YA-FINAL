/*
 * Balancin ESP32 Clean - Etapa 2
 *
 * Comandos: help | pins | status | stream on|off | rate <ms> | stop
 *           zero [n] | mpu | ir | enc [ms]
 *           pwm/motor/pulse <l|r|b> ... | balance on|off
 *           alphad <deg> | kp <val> | kd <val>
 */

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
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"

static const char *TAG = "bal_clean";

/* ============================================================================
   1. Mapeo de pines probado
   ============================================================================ */

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

/* Signos probados: positivo debe avanzar. */
#define MOTOR_LEFT_INVERT    1
#define MOTOR_RIGHT_INVERT   0
#define ENCODER_LEFT_INVERT  0
#define ENCODER_RIGHT_INVERT 1

/* Ejes del MPU: 1 = invertir, 0 = normal.
   Verifica con 'mpu': inclinar el robot hacia adelante debe dar comp_pitch positivo.
   Configuracion actual: MPU montado de cabeza (giro 180 grados sobre el eje Y). */
#define MPU_INVERT_AX  1
#define MPU_INVERT_AY  0
#define MPU_INVERT_AZ  1
#define MPU_INVERT_GX  1
#define MPU_INVERT_GY  1
#define MPU_INVERT_GZ  1

/* ============================================================================
   2. Constantes
   ============================================================================ */

/* Parametros fisicos del robot */
#define ROBOT_Ra        3.0f        /* resistencia de armadura [Ohm]      */
#define ROBOT_NR        34.014f     /* relacion de transmision             */
#define ROBOT_km        0.0008f     /* constante de par motor [N·m/A]     */
#define ROBOT_R         0.035f      /* radio de rueda [m]                  */
#define ROBOT_b         0.09f       /* semidistancia entre huellas [m]    */
#define ROBOT_tauM      0.1654f     /* par maximo en rueda [N·m]          */
#define ROBOT_uM        11.0f       /* voltaje maximo [V]                  */
#define ROBOT_ppr       12.0f       /* pulsos por rev del encoder          */

/* Derivados del modelo de motor */
#define ROBOT_Rasnkm    (ROBOT_Ra / (ROBOT_NR * ROBOT_km))  /* ≈ 110.25 V/(N·m) */
#define ROBOT_nkm       (ROBOT_NR * ROBOT_km)               /* ≈ 0.0272 N·m/rpm */
#define ROBOT_v2tauM    (2.0f * ROBOT_tauM)                 /* limite torque [N·m] */

/* Balance PD */
#define BALANCE_KP_DEFAULT      5.4567f
#define BALANCE_KD_DEFAULT      0.6877f
#define BALANCE_ALPHAD_DEFAULT  0.1030f  /* calpha en radianes */
#define BALANCE_FALL_RAD        1.3090f  /* 75 grados en radianes */
#define BALANCE_PERIOD_MS       10
#define BALANCE_WARMUP_STEPS    80
#define COMP_FILTER_ALPHA       0.98f

#define WIFI_AP_SSID     "Balancin"
#define WIFI_AP_PASS     "12345678"
#define WIFI_AP_CHANNEL  1
#define WIFI_AP_MAX_CONN 4

#define SERIAL_BAUD_RATE 115200
#define UART_RX_BUF_SIZE 512
#define COMMAND_LINE_SIZE 128

#define MOTOR_SUPPLY_VOLTAGE 11.0f
#define MOTOR_PWM_MAX 255
#define MOTOR_DEFAULT_TIMEOUT_MS 2500
#define MOTOR_PWM_FREQ_HZ 20000
#define MOTOR_PWM_RES LEDC_TIMER_8_BIT
#define MOTOR_PWM_TIMER LEDC_TIMER_0
#define MOTOR_PWM_MODE LEDC_LOW_SPEED_MODE
#define MOTOR_LEFT_CHANNEL LEDC_CHANNEL_0
#define MOTOR_RIGHT_CHANNEL LEDC_CHANNEL_1

#define PCNT_HIGH_LIMIT 1000
#define PCNT_LOW_LIMIT  -1000

#define MPU6050_ADDR        0x68
#define MPU6050_REG_CONFIG  0x1A
#define MPU6050_REG_GYRO    0x1B
#define MPU6050_REG_ACCEL   0x1C
#define MPU6050_REG_PWR1    0x6B
#define MPU6050_REG_PWR2    0x6C
#define MPU6050_REG_DATA    0x3B
#define MPU6050_REG_WHOAMI  0x75

#define ACCEL_LSB_PER_G 16384.0f
#define GYRO_LSB_PER_DPS 131.0f
#define RAD_TO_DEG 57.2957795f
#define DEG_TO_RAD ((float)M_PI / 180.0f)

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

    int left_pwm;
    int right_pwm;
    int64_t motor_stop_at_us;
    int motor_timeout_ms;

    bool stream_enabled;
    uint32_t stream_period_ms;

    bool balance_enabled;
    float alphad;
    float kp_balance;
    float kd_balance;
    float last_u_balance;
} app_state_t;

typedef struct {
    int16_t ax_raw;
    int16_t ay_raw;
    int16_t az_raw;
    int16_t gx_raw;
    int16_t gy_raw;
    int16_t gz_raw;

    float ax_g;
    float ay_g;
    float az_g;
    float gy_rads;          /* velocidad angular pitch [rad/s] */

    float accel_pitch_rad;  /* angulo pitch por acelerometro  */
    float comp_pitch_rad;   /* angulo pitch filtro complementario (alpha) */
} mpu_sample_t;

static app_state_t state = {
    .motor_timeout_ms = MOTOR_DEFAULT_TIMEOUT_MS,
    .stream_enabled = false,
    .stream_period_ms = 250,
    .kp_balance = BALANCE_KP_DEFAULT,
    .kd_balance = BALANCE_KD_DEFAULT,
    .alphad = BALANCE_ALPHAD_DEFAULT,
};

static adc_oneshot_unit_handle_t adc_handle;
static adc_channel_t line_left_channel;
static adc_channel_t line_right_channel;

static pcnt_unit_handle_t pcnt_right_unit;
static pcnt_unit_handle_t pcnt_left_unit;

static i2c_master_bus_handle_t i2c_bus;
static i2c_master_dev_handle_t mpu_dev;

static float gyro_offset_y_dps;
static float comp_pitch_rad;
static int64_t mpu_last_us;
static mpu_sample_t last_mpu_sample;

/* ============================================================================
   3. Utilidades pequenas
   ============================================================================ */

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
    float limited = clamp_float(volts,
                                -MOTOR_SUPPLY_VOLTAGE,
                                MOTOR_SUPPLY_VOLTAGE);
    int pwm = (int)lrintf((limited / MOTOR_SUPPLY_VOLTAGE) * MOTOR_PWM_MAX);
    return clamp_int(pwm, -MOTOR_PWM_MAX, MOTOR_PWM_MAX);
}

static const char *side_name(side_t side)
{
    if (side == SIDE_LEFT) {
        return "left";
    }
    if (side == SIDE_RIGHT) {
        return "right";
    }
    return "both";
}

static bool parse_side(const char *text, side_t *side)
{
    if (text == NULL) {
        return false;
    }
    if (strcasecmp(text, "l") == 0 || strcasecmp(text, "left") == 0 ||
        strcasecmp(text, "izq") == 0) {
        *side = SIDE_LEFT;
        return true;
    }
    if (strcasecmp(text, "r") == 0 || strcasecmp(text, "right") == 0 ||
        strcasecmp(text, "der") == 0) {
        *side = SIDE_RIGHT;
        return true;
    }
    if (strcasecmp(text, "b") == 0 || strcasecmp(text, "both") == 0 ||
        strcasecmp(text, "ambos") == 0) {
        *side = SIDE_BOTH;
        return true;
    }
    return false;
}

/* ============================================================================
   4. Motores L298N
   ============================================================================ */

static void motor_set_direction(gpio_num_t in1, gpio_num_t in2, int pwm, int invert)
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

    gpio_set_level(in1, positive ? 0 : 1);
    gpio_set_level(in2, positive ? 1 : 0);
}

static void motor_apply_pwm(int left_pwm, int right_pwm, bool timed)
{
    if (!state.motor_ready) {
        printf("ERR,motor,no_init\n");
        return;
    }

    left_pwm = clamp_int(left_pwm, -MOTOR_PWM_MAX, MOTOR_PWM_MAX);
    right_pwm = clamp_int(right_pwm, -MOTOR_PWM_MAX, MOTOR_PWM_MAX);

    motor_set_direction(PIN_MOTOR_LEFT_IN1,
                        PIN_MOTOR_LEFT_IN2,
                        left_pwm,
                        MOTOR_LEFT_INVERT);
    motor_set_direction(PIN_MOTOR_RIGHT_IN1,
                        PIN_MOTOR_RIGHT_IN2,
                        right_pwm,
                        MOTOR_RIGHT_INVERT);

    ledc_set_duty(MOTOR_PWM_MODE, MOTOR_LEFT_CHANNEL, (uint32_t)abs(left_pwm));
    ledc_update_duty(MOTOR_PWM_MODE, MOTOR_LEFT_CHANNEL);
    ledc_set_duty(MOTOR_PWM_MODE, MOTOR_RIGHT_CHANNEL, (uint32_t)abs(right_pwm));
    ledc_update_duty(MOTOR_PWM_MODE, MOTOR_RIGHT_CHANNEL);

    state.left_pwm = left_pwm;
    state.right_pwm = right_pwm;
    if (timed && (left_pwm != 0 || right_pwm != 0)) {
        state.motor_stop_at_us = esp_timer_get_time() +
                                 ((int64_t)state.motor_timeout_ms * 1000);
    } else if (left_pwm == 0 && right_pwm == 0) {
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
    ESP_RETURN_ON_ERROR(gpio_config(&gpio), TAG, "motor gpio failed");

    ledc_timer_config_t timer = {
        .speed_mode = MOTOR_PWM_MODE,
        .duty_resolution = MOTOR_PWM_RES,
        .timer_num = MOTOR_PWM_TIMER,
        .freq_hz = MOTOR_PWM_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_RETURN_ON_ERROR(ledc_timer_config(&timer), TAG, "motor pwm timer failed");

    ledc_channel_config_t channel = {
        .speed_mode = MOTOR_PWM_MODE,
        .timer_sel = MOTOR_PWM_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .duty = 0,
        .hpoint = 0,
    };

    channel.gpio_num = PIN_MOTOR_LEFT_PWM;
    channel.channel = MOTOR_LEFT_CHANNEL;
    ESP_RETURN_ON_ERROR(ledc_channel_config(&channel), TAG, "left pwm failed");

    channel.gpio_num = PIN_MOTOR_RIGHT_PWM;
    channel.channel = MOTOR_RIGHT_CHANNEL;
    ESP_RETURN_ON_ERROR(ledc_channel_config(&channel), TAG, "right pwm failed");

    state.motor_ready = true;
    motor_stop();
    return ESP_OK;
}

/* ============================================================================
   5. Sensores IR / ADC
   ============================================================================ */

static esp_err_t line_init(void)
{
    adc_oneshot_unit_init_cfg_t unit_config = {
        .unit_id = ADC_UNIT_1,
        .clk_src = ADC_RTC_CLK_SRC_DEFAULT,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_RETURN_ON_ERROR(adc_oneshot_new_unit(&unit_config, &adc_handle),
                        TAG, "adc unit failed");

    adc_unit_t unit;
    ESP_RETURN_ON_ERROR(adc_oneshot_io_to_channel(PIN_LINE_LEFT,
                                                  &unit,
                                                  &line_left_channel),
                        TAG, "left adc channel failed");
    ESP_RETURN_ON_ERROR(adc_oneshot_io_to_channel(PIN_LINE_RIGHT,
                                                  &unit,
                                                  &line_right_channel),
                        TAG, "right adc channel failed");

    adc_oneshot_chan_cfg_t channel_config = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };
    ESP_RETURN_ON_ERROR(adc_oneshot_config_channel(adc_handle,
                                                   line_left_channel,
                                                   &channel_config),
                        TAG, "left adc config failed");
    ESP_RETURN_ON_ERROR(adc_oneshot_config_channel(adc_handle,
                                                   line_right_channel,
                                                   &channel_config),
                        TAG, "right adc config failed");

    state.line_ready = true;
    return ESP_OK;
}

static esp_err_t line_read(uint16_t *left_raw, uint16_t *right_raw)
{
    if (!state.line_ready) {
        return ESP_ERR_INVALID_STATE;
    }

    int left = 0;
    int right = 0;
    ESP_RETURN_ON_ERROR(adc_oneshot_read(adc_handle, line_left_channel, &left),
                        TAG, "left adc read failed");
    ESP_RETURN_ON_ERROR(adc_oneshot_read(adc_handle, line_right_channel, &right),
                        TAG, "right adc read failed");

    *left_raw = (uint16_t)clamp_int(left, 0, 4095);
    *right_raw = (uint16_t)clamp_int(right, 0, 4095);
    return ESP_OK;
}

/* ============================================================================
   6. Encoders por PCNT
   ============================================================================ */

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
    ESP_RETURN_ON_ERROR(pcnt_new_unit(&unit_config, unit), TAG, "pcnt unit failed");

    pcnt_glitch_filter_config_t filter = {
        .max_glitch_ns = 1000,
    };
    ESP_RETURN_ON_ERROR(pcnt_unit_set_glitch_filter(*unit, &filter),
                        TAG, "pcnt filter failed");

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
                        TAG, "pcnt channel A failed");
    ESP_RETURN_ON_ERROR(pcnt_new_channel(*unit, &chan_b_config, &chan_b),
                        TAG, "pcnt channel B failed");

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
    pcnt_unit_get_count(pcnt_right_unit, &right);
    pcnt_unit_clear_count(pcnt_right_unit);
    pcnt_unit_get_count(pcnt_left_unit, &left);
    pcnt_unit_clear_count(pcnt_left_unit);

    if (ENCODER_RIGHT_INVERT) {
        right = -right;
    }
    if (ENCODER_LEFT_INVERT) {
        left = -left;
    }

    *right_delta = right;
    *left_delta = left;
    return ESP_OK;
}

/* ============================================================================
   7. MPU6050
   ============================================================================ */

static esp_err_t mpu_write_reg(uint8_t reg, uint8_t value)
{
    uint8_t data[2] = {reg, value};
    return i2c_master_transmit(mpu_dev, data, sizeof(data), 100);
}

static esp_err_t mpu_read_regs(uint8_t reg, uint8_t *data, size_t len)
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
                        TAG, "i2c bus failed");

    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = MPU6050_ADDR,
        .scl_speed_hz = 400000,
    };
    ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(i2c_bus, &dev_config, &mpu_dev),
                        TAG, "mpu device failed");

    uint8_t whoami = 0;
    if (mpu_read_regs(MPU6050_REG_WHOAMI, &whoami, 1) == ESP_OK) {
        printf("MPU,WHOAMI,0x%02X\n", whoami);
    }

    ESP_RETURN_ON_ERROR(mpu_write_reg(MPU6050_REG_PWR1, 0x80),
                        TAG, "mpu reset failed");
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_RETURN_ON_ERROR(mpu_write_reg(MPU6050_REG_PWR1, 0x00),
                        TAG, "mpu wake failed");
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_RETURN_ON_ERROR(mpu_write_reg(MPU6050_REG_PWR2, 0x00),
                        TAG, "mpu pwr2 failed");
    ESP_RETURN_ON_ERROR(mpu_write_reg(MPU6050_REG_CONFIG, 0x01),
                        TAG, "mpu config failed");
    ESP_RETURN_ON_ERROR(mpu_write_reg(MPU6050_REG_GYRO, 0x00),
                        TAG, "mpu gyro failed");
    ESP_RETURN_ON_ERROR(mpu_write_reg(MPU6050_REG_ACCEL, 0x00),
                        TAG, "mpu accel failed");

    mpu_last_us = esp_timer_get_time();
    state.mpu_ready = true;
    return ESP_OK;
}

static esp_err_t mpu_read_sample(mpu_sample_t *sample)
{
    if (!state.mpu_ready) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t data[14] = {0};
    ESP_RETURN_ON_ERROR(mpu_read_regs(MPU6050_REG_DATA, data, sizeof(data)),
                        TAG, "mpu read failed");

    sample->ax_raw = (int16_t)((data[0] << 8) | data[1]);
    sample->ay_raw = (int16_t)((data[2] << 8) | data[3]);
    sample->az_raw = (int16_t)((data[4] << 8) | data[5]);
    sample->gx_raw = (int16_t)((data[8] << 8) | data[9]);
    sample->gy_raw = (int16_t)((data[10] << 8) | data[11]);
    sample->gz_raw = (int16_t)((data[12] << 8) | data[13]);

    sample->ax_g = (MPU_INVERT_AX ? -1.0f : 1.0f) * (sample->ax_raw / ACCEL_LSB_PER_G);
    sample->ay_g = (MPU_INVERT_AY ? -1.0f : 1.0f) * (sample->ay_raw / ACCEL_LSB_PER_G);
    sample->az_g = (MPU_INVERT_AZ ? -1.0f : 1.0f) * (sample->az_raw / ACCEL_LSB_PER_G);
    sample->gy_rads = (MPU_INVERT_GY ? -1.0f : 1.0f) *
                      ((sample->gy_raw / GYRO_LSB_PER_DPS) - gyro_offset_y_dps) * DEG_TO_RAD;

    sample->accel_pitch_rad = atan2f(-sample->ax_g,
                                     sqrtf(sample->ay_g * sample->ay_g +
                                           sample->az_g * sample->az_g));

    int64_t now_us = esp_timer_get_time();
    float dt = (now_us - mpu_last_us) / 1000000.0f;
    if (dt <= 0.0f || dt > 0.2f) {
        dt = 0.01f;
    }
    mpu_last_us = now_us;

    comp_pitch_rad = COMP_FILTER_ALPHA * (comp_pitch_rad + sample->gy_rads * dt) +
                     (1.0f - COMP_FILTER_ALPHA) * sample->accel_pitch_rad;
    sample->comp_pitch_rad = comp_pitch_rad;
    return ESP_OK;
}

static esp_err_t mpu_zero(uint32_t samples)
{
    if (!state.mpu_ready) {
        return ESP_ERR_INVALID_STATE;
    }
    if (samples < 20) {
        samples = 20;
    }
    if (samples > 3000) {
        samples = 3000;
    }

    printf("MPU,ZERO,start,samples=%" PRIu32 "\n", samples);

    int64_t sum_gy = 0;
    for (uint32_t i = 0; i < samples; i++) {
        uint8_t data[14] = {0};
        ESP_RETURN_ON_ERROR(mpu_read_regs(MPU6050_REG_DATA, data, sizeof(data)),
                            TAG, "mpu zero read failed");
        sum_gy += (int16_t)((data[10] << 8) | data[11]);
        vTaskDelay(pdMS_TO_TICKS(2));
    }

    gyro_offset_y_dps = ((float)sum_gy / (float)samples) / GYRO_LSB_PER_DPS;
    comp_pitch_rad = 0.0f;
    mpu_last_us = esp_timer_get_time();

    printf("MPU,ZERO,ok,off_gy=%.4f\n", gyro_offset_y_dps);
    return ESP_OK;
}

/* ============================================================================
   8. Comandos seriales
   ============================================================================ */

static void print_pins(void)
{
    printf("\nPINMAP,current\n");
    printf("PIN,MPU_SDA,GPIO%d\n", PIN_I2C_SDA);
    printf("PIN,MPU_SCL,GPIO%d\n", PIN_I2C_SCL);
    printf("PIN,IR_LEFT,GPIO%d\n", PIN_LINE_LEFT);
    printf("PIN,IR_RIGHT,GPIO%d\n", PIN_LINE_RIGHT);
    printf("PIN,ENC_RIGHT_A,GPIO%d\n", PIN_ENCODER_RIGHT_A);
    printf("PIN,ENC_RIGHT_B,GPIO%d\n", PIN_ENCODER_RIGHT_B);
    printf("PIN,ENC_LEFT_A,GPIO%d\n", PIN_ENCODER_LEFT_A);
    printf("PIN,ENC_LEFT_B,GPIO%d\n", PIN_ENCODER_LEFT_B);
    printf("PIN,MOTOR_LEFT_PWM,GPIO%d\n", PIN_MOTOR_LEFT_PWM);
    printf("PIN,MOTOR_LEFT_IN1,GPIO%d\n", PIN_MOTOR_LEFT_IN1);
    printf("PIN,MOTOR_LEFT_IN2,GPIO%d\n", PIN_MOTOR_LEFT_IN2);
    printf("PIN,MOTOR_RIGHT_IN1,GPIO%d\n", PIN_MOTOR_RIGHT_IN1);
    printf("PIN,MOTOR_RIGHT_IN2,GPIO%d\n", PIN_MOTOR_RIGHT_IN2);
    printf("PIN,MOTOR_RIGHT_PWM,GPIO%d\n\n", PIN_MOTOR_RIGHT_PWM);
}

static void print_help(void)
{
    printf("\n=== Balancin Clean - Etapa 2 ===\n");
    printf("help                         muestra ayuda\n");
    printf("pins                         muestra pines actuales\n");
    printf("status                       muestra drivers y salidas\n");
    printf("stream on|off                activa/desactiva CSV\n");
    printf("rate <ms>                    periodo CSV, min 50 ms\n");
    printf("stop                         apaga motores (desactiva balance)\n");
    printf("timeout <ms>                 auto-stop motores (solo sin balance)\n");
    printf("pwm <l|r|b> <pwm> [pwm_r]    PWM manual (solo sin balance)\n");
    printf("motor <l|r|b> <V> [V_r]      voltaje manual (solo sin balance)\n");
    printf("pulse <l|r|b> <V> <ms>       pulso seguro (solo sin balance)\n");
    printf("enc [ms]                     delta encoders\n");
    printf("ir                           lee sensores IR\n");
    printf("mpu                          lee MPU / ultimo sample balance\n");
    printf("zero [samples]               calibra gyro (solo sin balance)\n");
    printf("--- Etapa 2 ---\n");
    printf("balance on|off               activa/desactiva control PD\n");
    printf("alphad <deg>                 angulo deseado (ajusta si deriva)\n");
    printf("kp <val>                     ganancia proporcional (default %.2f)\n", BALANCE_KP_DEFAULT);
    printf("kd <val>                     ganancia derivativa  (default %.2f)\n", BALANCE_KD_DEFAULT);
    printf("\nEtapa 1: zero -> mpu -> pulse l/r -> enc.\n");
    printf("Etapa 2: zero 200 -> balance on -> ajustar alphad / kp / kd.\n\n");
}

static void print_status(void)
{
    printf("STATUS,motor=%d,line=%d,encoder=%d,mpu=%d\n",
           state.motor_ready,
           state.line_ready,
           state.encoder_ready,
           state.mpu_ready);
    printf("STATUS,pwm_l=%d,pwm_r=%d,timeout_ms=%d,stream=%d,rate_ms=%" PRIu32 "\n",
           state.left_pwm,
           state.right_pwm,
           state.motor_timeout_ms,
           state.stream_enabled,
           state.stream_period_ms);
}

static void print_ir_once(void)
{
    uint16_t left = 0;
    uint16_t right = 0;
    esp_err_t err = line_read(&left, &right);
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

static void print_encoder_delta(uint32_t duration_ms)
{
    int right = 0;
    int left = 0;

    if (duration_ms > 0) {
        encoder_read_delta(&right, &left);
        vTaskDelay(pdMS_TO_TICKS(clamp_int((int)duration_ms, 1, 10000)));
    }

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

static void print_mpu_data(const mpu_sample_t *mpu)
{
    printf("MPU,raw,ax=%d,ay=%d,az=%d,gx=%d,gy=%d,gz=%d\n",
           mpu->ax_raw, mpu->ay_raw, mpu->az_raw,
           mpu->gx_raw, mpu->gy_raw, mpu->gz_raw);
    printf("MPU,unit,ax_g=%.4f,ay_g=%.4f,az_g=%.4f,gy_dps=%.3f\n",
           mpu->ax_g, mpu->ay_g, mpu->az_g,
           mpu->gy_rads * RAD_TO_DEG);
    printf("MPU,pitch_deg,acc=%.2f,comp=%.2f\n",
           mpu->accel_pitch_rad * RAD_TO_DEG,
           mpu->comp_pitch_rad  * RAD_TO_DEG);
}

static void print_mpu_once(void)
{
    if (state.balance_enabled) {
        print_mpu_data(&last_mpu_sample);
        return;
    }
    mpu_sample_t mpu = {0};
    esp_err_t err = mpu_read_sample(&mpu);
    if (err != ESP_OK) {
        printf("MPU,ERR,%s\n", esp_err_to_name(err));
        return;
    }
    print_mpu_data(&mpu);
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
        print_pins();
    } else if (strcasecmp(cmd, "status") == 0) {
        print_status();
    } else if (strcasecmp(cmd, "stream") == 0) {
        char *arg = strtok_r(NULL, " \t\r\n", &saveptr);
        if (arg != NULL && strcasecmp(arg, "on") == 0) {
            state.stream_enabled = true;
        } else if (arg != NULL && strcasecmp(arg, "off") == 0) {
            state.stream_enabled = false;
        }
        printf("STREAM,%d\n", state.stream_enabled);
    } else if (strcasecmp(cmd, "rate") == 0) {
        char *arg = strtok_r(NULL, " \t\r\n", &saveptr);
        if (arg != NULL) {
            state.stream_period_ms = (uint32_t)clamp_int(atoi(arg), 50, 5000);
        }
        printf("RATE,%" PRIu32 "\n", state.stream_period_ms);
    } else if (strcasecmp(cmd, "stop") == 0) {
        state.balance_enabled = false;
        motor_stop();
        printf("MOTOR,stop\n");
    } else if (strcasecmp(cmd, "timeout") == 0) {
        char *arg = strtok_r(NULL, " \t\r\n", &saveptr);
        if (arg != NULL) {
            state.motor_timeout_ms = clamp_int(atoi(arg), 100, 10000);
        }
        printf("TIMEOUT,%d\n", state.motor_timeout_ms);
    } else if (strcasecmp(cmd, "pwm") == 0) {
        if (state.balance_enabled) { printf("ERR,balance_on,usa 'stop' primero\n"); return; }
        char *side_text = strtok_r(NULL, " \t\r\n", &saveptr);
        char *p1_text   = strtok_r(NULL, " \t\r\n", &saveptr);
        char *p2_text   = strtok_r(NULL, " \t\r\n", &saveptr);
        side_t side = SIDE_BOTH;
        if (!parse_side(side_text, &side) || p1_text == NULL) {
            printf("ERR,uso:pwm <l|r|b> <pwm> [pwm_r]\n");
            return;
        }
        int p1 = atoi(p1_text);
        int p2 = p2_text ? atoi(p2_text) : p1;
        int left  = (side == SIDE_RIGHT) ? 0 : p1;
        int right = (side == SIDE_LEFT)  ? 0 : p2;
        motor_apply_pwm(left, right, true);
        printf("MOTOR,PWM,l=%d,r=%d,auto_stop_ms=%d\n",
               state.left_pwm, state.right_pwm, state.motor_timeout_ms);
    } else if (strcasecmp(cmd, "motor") == 0 || strcasecmp(cmd, "m") == 0) {
        if (state.balance_enabled) { printf("ERR,balance_on,usa 'stop' primero\n"); return; }
        char *side_text = strtok_r(NULL, " \t\r\n", &saveptr);
        char *v1_text   = strtok_r(NULL, " \t\r\n", &saveptr);
        char *v2_text   = strtok_r(NULL, " \t\r\n", &saveptr);
        side_t side = SIDE_BOTH;
        if (!parse_side(side_text, &side) || v1_text == NULL) {
            printf("ERR,uso:motor <l|r|b> <V> [V_r]\n");
            return;
        }
        float v1 = strtof(v1_text, NULL);
        float v2 = v2_text ? strtof(v2_text, NULL) : v1;
        int left  = (side == SIDE_RIGHT) ? 0 : pwm_from_voltage(v1);
        int right = (side == SIDE_LEFT)  ? 0 : pwm_from_voltage(v2);
        motor_apply_pwm(left, right, true);
        printf("MOTOR,V,l=%.2f,r=%.2f,pwm_l=%d,pwm_r=%d\n",
               (state.left_pwm  * MOTOR_SUPPLY_VOLTAGE) / MOTOR_PWM_MAX,
               (state.right_pwm * MOTOR_SUPPLY_VOLTAGE) / MOTOR_PWM_MAX,
               state.left_pwm, state.right_pwm);
    } else if (strcasecmp(cmd, "pulse") == 0) {
        if (state.balance_enabled) { printf("ERR,balance_on,usa 'stop' primero\n"); return; }
        char *side_text = strtok_r(NULL, " \t\r\n", &saveptr);
        char *v_text    = strtok_r(NULL, " \t\r\n", &saveptr);
        char *ms_text   = strtok_r(NULL, " \t\r\n", &saveptr);
        side_t side = SIDE_BOTH;
        if (!parse_side(side_text, &side) || v_text == NULL || ms_text == NULL) {
            printf("ERR,uso:pulse <l|r|b> <V> <ms>\n");
            return;
        }
        int pwm_val = pwm_from_voltage(strtof(v_text, NULL));
        uint32_t ms = (uint32_t)clamp_int(atoi(ms_text), 50, 5000);
        int left  = (side == SIDE_RIGHT) ? 0 : pwm_val;
        int right = (side == SIDE_LEFT)  ? 0 : pwm_val;
        printf("PULSE,%s,pwm=%d,ms=%" PRIu32 "\n", side_name(side), pwm_val, ms);
        motor_apply_pwm(left, right, false);
        vTaskDelay(pdMS_TO_TICKS(ms));
        motor_stop();
        printf("PULSE,stop\n");
    } else if (strcasecmp(cmd, "enc") == 0) {
        char *ms_text = strtok_r(NULL, " \t\r\n", &saveptr);
        uint32_t ms = ms_text ? (uint32_t)atoi(ms_text) : 0;
        print_encoder_delta(ms);
    } else if (strcasecmp(cmd, "ir") == 0) {
        print_ir_once();
    } else if (strcasecmp(cmd, "mpu") == 0) {
        print_mpu_once();
    } else if (strcasecmp(cmd, "zero") == 0) {
        if (state.balance_enabled) { printf("ERR,balance_on,usa 'stop' primero\n"); return; }
        char *samples_text = strtok_r(NULL, " \t\r\n", &saveptr);
        uint32_t samples = samples_text ? (uint32_t)atoi(samples_text) : 500;
        esp_err_t err = mpu_zero(samples);
        if (err != ESP_OK) {
            printf("MPU,ZERO,ERR,%s\n", esp_err_to_name(err));
        }
    } else if (strcasecmp(cmd, "balance") == 0) {
        char *arg = strtok_r(NULL, " \t\r\n", &saveptr);
        if (arg != NULL && strcasecmp(arg, "on") == 0) {
            if (!state.mpu_ready || !state.motor_ready) {
                printf("ERR,balance,drivers_not_ready\n");
            } else {
                motor_stop();
                printf("BALANCE,warmup,%d_steps\n", BALANCE_WARMUP_STEPS);
                for (int i = 0; i < BALANCE_WARMUP_STEPS; i++) {
                    mpu_sample_t tmp = {0};
                    if (mpu_read_sample(&tmp) == ESP_OK) {
                        last_mpu_sample = tmp;
                    }
                    vTaskDelay(pdMS_TO_TICKS(BALANCE_PERIOD_MS));
                }
                state.alphad = last_mpu_sample.comp_pitch_rad;
                state.balance_enabled = true;
                printf("BALANCE,on,alpha_init=%.2f_deg,alphad=%.4f_rad,kp=%.4f,kd=%.4f\n",
                       last_mpu_sample.comp_pitch_rad * RAD_TO_DEG,
                       state.alphad,
                       state.kp_balance,
                       state.kd_balance);
            }
        } else if (arg != NULL && strcasecmp(arg, "off") == 0) {
            state.balance_enabled = false;
            motor_stop();
            printf("BALANCE,off\n");
        } else {
            printf("BALANCE,%d,alphad=%.4f_rad (%.2f_deg),kp=%.4f,kd=%.4f\n",
                   state.balance_enabled,
                   state.alphad,
                   state.alphad * RAD_TO_DEG,
                   state.kp_balance,
                   state.kd_balance);
        }
    } else if (strcasecmp(cmd, "alphad") == 0) {
        char *arg = strtok_r(NULL, " \t\r\n", &saveptr);
        if (arg != NULL) {
            state.alphad = strtof(arg, NULL) * DEG_TO_RAD;
        }
        printf("ALPHAD,%.4f_rad (%.2f_deg)\n", state.alphad, state.alphad * RAD_TO_DEG);
    } else if (strcasecmp(cmd, "kp") == 0) {
        char *arg = strtok_r(NULL, " \t\r\n", &saveptr);
        if (arg != NULL) {
            state.kp_balance = strtof(arg, NULL);
        }
        printf("KP,%.4f\n", state.kp_balance);
    } else if (strcasecmp(cmd, "kd") == 0) {
        char *arg = strtok_r(NULL, " \t\r\n", &saveptr);
        if (arg != NULL) {
            state.kd_balance = strtof(arg, NULL);
        }
        printf("KD,%.4f\n", state.kd_balance);
    } else {
        printf("ERR,unknown,%s\n", cmd);
    }
}

/* ============================================================================
   9. Tareas
   ============================================================================ */

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

    printf("CSV_HEADER,t_ms,ir_l,ir_r,ir8_l,ir8_r,enc_l,enc_r,ax,ay,az,gx,gy,gz,acc_pitch_deg,comp_pitch_deg,pwm_l,pwm_r,u_bal,balance\n");

    for (;;) {
        motor_check_timeout();

        if (state.stream_enabled) {
            uint16_t ir_l = 0;
            uint16_t ir_r = 0;
            int enc_l = 0;
            int enc_r = 0;
            mpu_sample_t mpu = {0};

            bool line_ok = line_read(&ir_l, &ir_r) == ESP_OK;
            bool enc_ok = encoder_read_delta(&enc_r, &enc_l) == ESP_OK;
            bool mpu_ok;
            if (state.balance_enabled) {
                mpu = last_mpu_sample;
                mpu_ok = true;
            } else {
                mpu_ok = mpu_read_sample(&mpu) == ESP_OK;
            }

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
            printf("%.2f,%.2f,%d,%d,%.3f,%d\n",
                   mpu_ok ? mpu.accel_pitch_rad * RAD_TO_DEG : 0.0f,
                   mpu_ok ? mpu.comp_pitch_rad  * RAD_TO_DEG : 0.0f,
                   state.left_pwm,
                   state.right_pwm,
                   state.last_u_balance,
                   (int)state.balance_enabled);
        }

        vTaskDelay(pdMS_TO_TICKS(state.stream_period_ms));
    }
}

/* ============================================================================
   10. Tarea de balance - Etapa 2
   ============================================================================ */

static void balance_task(void *arg)
{
    (void)arg;
    const TickType_t period = pdMS_TO_TICKS(BALANCE_PERIOD_MS);
    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        vTaskDelayUntil(&last_wake, period);

        if (!state.balance_enabled) {
            continue;
        }

        mpu_sample_t mpu = {0};
        if (mpu_read_sample(&mpu) != ESP_OK) {
            motor_stop();
            state.balance_enabled = false;
            printf("BALANCE,err,mpu\n");
            continue;
        }
        last_mpu_sample = mpu;

        float alpha     = mpu.comp_pitch_rad;
        float alpha_dot = mpu.gy_rads;

        if (fabsf(alpha) > BALANCE_FALL_RAD) {
            motor_stop();
            state.balance_enabled = false;
            printf("BALANCE,fall,alpha=%.2f_deg\n", alpha * RAD_TO_DEG);
            continue;
        }

        /* ei = alphad - alpha  (negativo al inclinarse adelante)
           eip = -alpha_dot    (derivada del error = -vel angular)
           u = -kp*ei - kd*eip = -kp*ei + kd*alpha_dot   (igual que PIC) */
        float ei = state.alphad - alpha;
        float u_torque = -state.kp_balance * ei + state.kd_balance * alpha_dot;
        u_torque = clamp_float(u_torque, -ROBOT_v2tauM, ROBOT_v2tauM);

        /* modelo de motor: torque → voltaje  (back-EMF se añade en Etapa 3) */
        float u_voltage = u_torque * ROBOT_Rasnkm;
        u_voltage = clamp_float(u_voltage, -ROBOT_uM, ROBOT_uM);
        state.last_u_balance = u_voltage;

        int pwm_out = pwm_from_voltage(u_voltage);
        motor_apply_pwm(pwm_out, pwm_out, false);
    }
}

/* ============================================================================
   11. WiFi AP + servidor HTTP
   ============================================================================ */

static const char HTTP_HTML[] =
    "<!DOCTYPE html><html><head>"
    "<meta charset='utf-8'>"
    "<meta name='viewport' content='width=device-width,initial-scale=1'>"
    "<title>Balancin</title>"
    "<style>"
    "*{box-sizing:border-box}"
    "body{font-family:sans-serif;max-width:420px;margin:0 auto;padding:12px;"
    "background:#111;color:#eee}"
    "h2{text-align:center;color:#4fc;margin:8px 0}"
    ".c{background:#1e1e2e;border-radius:10px;padding:14px;margin:8px 0}"
    "#ang{font-size:3.5em;text-align:center;font-weight:bold;color:#4fc}"
    "#ang.w{color:#f55}"
    "#st{text-align:center;font-size:.88em;color:#aaa;margin:4px 0}"
    "label{font-size:.82em;color:#aaa;display:block;margin-top:6px}"
    "input{width:100%;padding:8px;margin:3px 0;background:#2a2a3e;"
    "border:1px solid #444;border-radius:6px;color:#fff;font-size:1em}"
    "button{width:100%;padding:11px;margin:3px 0;border:none;"
    "border-radius:6px;font-size:1em;font-weight:bold;cursor:pointer}"
    ".g{background:#4fc;color:#000}.r{background:#f55;color:#fff}"
    ".y{background:#fa0;color:#000}.b{background:#48f;color:#fff}"
    "</style></head><body>"
    "<h2>Balancin</h2>"
    "<div class='c'>"
    "<div id='ang'>--.-&deg;</div>"
    "<div id='st'>conectando...</div>"
    "</div>"
    "<div class='c'>"
    "<label>Angulo deseado alphad (grados)</label>"
    "<input type='number' id='alphad' step='0.1' value='5.9'>"
    "<button class='b' onclick='C(\"alphad \"+V(\"alphad\"))'>Aplicar alphad</button>"
    "</div>"
    "<div class='c'>"
    "<label>Kp</label>"
    "<input type='number' id='kp' step='0.1' value='5.4567'>"
    "<label>Kd</label>"
    "<input type='number' id='kd' step='0.01' value='0.6877'>"
    "<button class='b' onclick='C(\"kp \"+V(\"kp\"));C(\"kd \"+V(\"kd\"))'>Aplicar Kp y Kd</button>"
    "</div>"
    "<div class='c'>"
    "<button class='g' onclick='C(\"balance on\")'>Balance ON</button>"
    "<button class='r' onclick='C(\"balance off\")'>Balance OFF</button>"
    "<button class='y' onclick='C(\"stop\")'>STOP</button>"
    "<button class='b' onclick='C(\"stop\");setTimeout(function(){C(\"zero 200\")},300)'>"
    "Calibrar Gyro</button>"
    "</div>"
    "<a href='/monitor' style='display:block;text-align:center;color:#48f;"
    "padding:8px;background:#1e1e2e;border-radius:8px;margin:8px 0;"
    "text-decoration:none'>Ver gr&aacute;fica en vivo &rarr;</a>"
    "<script>"
    "function V(x){return document.getElementById(x).value}"
    "function C(c){fetch('/cmd',{method:'POST',body:c}).catch(console.error)}"
    "setInterval(function(){"
    "fetch('/status').then(function(r){return r.json()}).then(function(d){"
    "var el=document.getElementById('ang');"
    "el.textContent=d.pitch_deg.toFixed(1)+'\\u00b0';"
    "el.className=Math.abs(d.pitch_deg)<30?'':'w';"
    "document.getElementById('st').textContent="
    "'balance:'+(d.balance?'ON \\u2713':'OFF')"
    "+'  u='+d.u_v.toFixed(1)+'V'"
    "+'  alphad='+d.alphad_deg.toFixed(1)+'\\u00b0';"
    "}).catch(function(){})},200);"
    "</script></body></html>";

static const char HTTP_MONITOR[] =
    "<!DOCTYPE html><html><head>"
    "<meta charset='utf-8'>"
    "<meta name='viewport' content='width=device-width,initial-scale=1'>"
    "<title>Monitor</title>"
    "<style>"
    "body{font-family:monospace;max-width:440px;margin:0 auto;padding:10px;"
    "background:#111;color:#eee}"
    "h2{text-align:center;color:#4fc;margin:6px 0}"
    "canvas{width:100%;border:1px solid #2a2a2a;border-radius:6px;display:block}"
    "#info{font-size:.83em;padding:8px;background:#1e1e2e;border-radius:6px;"
    "margin:6px 0;line-height:2}"
    ".leg{font-size:.78em;color:#888;margin:4px 0}"
    "a{color:#48f;text-decoration:none;display:block;text-align:center;"
    "padding:8px;background:#1e1e2e;border-radius:8px;margin:6px 0}"
    "</style></head><body>"
    "<h2>Monitor en vivo</h2>"
    "<canvas id='cv' width='400' height='200'></canvas>"
    "<div id='info'>conectando...</div>"
    "<div class='leg'>"
    "&#9632; <span style=color:#4fc>pitch filtrado</span> &nbsp;"
    "&#9632; <span style=color:#444>acelerometro</span> &nbsp;"
    "&#9632; <span style=color:#fa0>voltaje u</span> &nbsp;"
    "- - <span style=color:#aaa>alphad</span>"
    "</div>"
    "<a href='/'>&#8592; Controles</a>"
    "<script>"
    "var N=200,pt=[],ra=[],uv=[],ad=0;"
    "for(var i=0;i<N;i++){pt.push(0);ra.push(0);uv.push(0);}"
    "function pl(ctx,data,W,yc,ys,sc,col,lw){"
    "ctx.strokeStyle=col;ctx.lineWidth=lw;ctx.beginPath();"
    "for(var i=0;i<data.length;i++){"
    "var x=i*W/data.length,y=yc-data[i]*ys*sc;"
    "i?ctx.lineTo(x,y):ctx.moveTo(x,y);}"
    "ctx.stroke();}"
    "function draw(){"
    "var cv=document.getElementById('cv');"
    "var ctx=cv.getContext('2d'),W=cv.width,H=cv.height,yc=H/2,ys=H/60;"
    "ctx.fillStyle='#111';ctx.fillRect(0,0,W,H);"
    "ctx.font='9px monospace';"
    "[-30,-20,-10,10,20,30].forEach(function(d){"
    "var y=yc-d*ys;"
    "ctx.strokeStyle='#222';ctx.lineWidth=0.5;"
    "ctx.beginPath();ctx.moveTo(0,y);ctx.lineTo(W,y);ctx.stroke();"
    "ctx.fillStyle='#555';ctx.fillText(d+'\\u00b0',2,y-1);});"
    "ctx.strokeStyle='#333';ctx.lineWidth=1;"
    "ctx.beginPath();ctx.moveTo(0,yc);ctx.lineTo(W,yc);ctx.stroke();"
    "ctx.strokeStyle='#fff4';ctx.lineWidth=1;ctx.setLineDash([5,4]);"
    "ctx.beginPath();ctx.moveTo(0,yc-ad*ys);ctx.lineTo(W,yc-ad*ys);ctx.stroke();"
    "ctx.setLineDash([]);"
    "pl(ctx,ra,W,yc,ys,1,'#3a3a3a',1);"
    "pl(ctx,uv,W,yc,ys,30/11,'#fa08',1);"
    "pl(ctx,pt,W,yc,ys,1,'#4fc',2);}"
    "setInterval(function(){"
    "fetch('/status').then(function(r){return r.json()}).then(function(d){"
    "pt.shift();pt.push(d.pitch_deg);"
    "ra.shift();ra.push(d.acc_pitch_deg);"
    "uv.shift();uv.push(d.u_v);"
    "ad=d.alphad_deg;draw();"
    "document.getElementById('info').innerHTML="
    "'<span style=color:#4fc>&#945;='+d.pitch_deg.toFixed(2)+'&deg;</span>&nbsp;'"
    "+'<span style=color:#888>acc='+d.acc_pitch_deg.toFixed(2)+'&deg;</span>&nbsp;'"
    "+'<span style=color:#fa0>u='+d.u_v.toFixed(2)+'V</span>&nbsp;'"
    "+'<span style=color:#88f>&#969;='+d.gy_dps.toFixed(1)+'&deg;/s</span>&nbsp;'"
    "+'<span style=color:#fff>bal='+(d.balance?'<b>ON</b>':'OFF')+'</span>';"
    "}).catch(function(){})},50);"
    "</script></body></html>";

static esp_err_t http_get_monitor(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_send(req, HTTP_MONITOR, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static esp_err_t http_get_root(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_send(req, HTTP_HTML, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static esp_err_t http_get_status(httpd_req_t *req)
{
    char json[220];
    snprintf(json, sizeof(json),
             "{\"pitch_deg\":%.2f,\"acc_pitch_deg\":%.2f,\"gy_dps\":%.1f,"
             "\"balance\":%d,\"alphad_deg\":%.2f,"
             "\"kp\":%.4f,\"kd\":%.4f,\"u_v\":%.2f}",
             last_mpu_sample.comp_pitch_rad * RAD_TO_DEG,
             last_mpu_sample.accel_pitch_rad * RAD_TO_DEG,
             last_mpu_sample.gy_rads * RAD_TO_DEG,
             (int)state.balance_enabled,
             state.alphad * RAD_TO_DEG,
             state.kp_balance,
             state.kd_balance,
             state.last_u_balance);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static esp_err_t http_post_cmd(httpd_req_t *req)
{
    char buf[COMMAND_LINE_SIZE] = {0};
    int len = httpd_req_recv(req, buf, sizeof(buf) - 1);
    if (len > 0) {
        buf[len] = '\0';
        process_command(buf);
    }
    httpd_resp_send(req, "ok", HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static void wifi_init(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t ap_config = {
        .ap = {
            .ssid           = WIFI_AP_SSID,
            .ssid_len       = sizeof(WIFI_AP_SSID) - 1,
            .channel        = WIFI_AP_CHANNEL,
            .password       = WIFI_AP_PASS,
            .max_connection = WIFI_AP_MAX_CONN,
            .authmode       = WIFI_AUTH_WPA2_PSK,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    printf("WIFI,AP,ssid=%s,ip=192.168.4.1\n", WIFI_AP_SSID);
}

static void http_server_init(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.core_id = 0;
    httpd_handle_t server = NULL;
    ESP_ERROR_CHECK(httpd_start(&server, &config));

    static const httpd_uri_t uri_root    = {"/",        HTTP_GET,  http_get_root,    NULL};
    static const httpd_uri_t uri_monitor = {"/monitor", HTTP_GET,  http_get_monitor, NULL};
    static const httpd_uri_t uri_status  = {"/status",  HTTP_GET,  http_get_status,  NULL};
    static const httpd_uri_t uri_cmd     = {"/cmd",     HTTP_POST, http_post_cmd,    NULL};

    httpd_register_uri_handler(server, &uri_root);
    httpd_register_uri_handler(server, &uri_monitor);
    httpd_register_uri_handler(server, &uri_status);
    httpd_register_uri_handler(server, &uri_cmd);

    printf("HTTP,server,192.168.4.1:80\n");
}

/* ============================================================================
   12. Arranque
   ============================================================================ */

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
        ESP_LOGW(TAG, "uart driver install: %s", esp_err_to_name(err));
    }
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_0, &uart_config));
}

void app_main(void)
{
    serial_init();
    wifi_init();

    printf("\n\nBalancin ESP32 Clean - Etapa 2\n");
    printf("Baud,%d\n", SERIAL_BAUD_RATE);
    print_pins();

    esp_err_t err = motor_init();
    printf("INIT,motor,%s\n", esp_err_to_name(err));

    err = line_init();
    printf("INIT,line,%s\n", esp_err_to_name(err));

    err = encoder_init();
    printf("INIT,encoder,%s\n", esp_err_to_name(err));

    err = mpu_init();
    printf("INIT,mpu,%s\n", esp_err_to_name(err));
    if (err == ESP_OK) {
        err = mpu_zero(300);
        if (err != ESP_OK) {
            printf("INIT,mpu_zero,%s\n", esp_err_to_name(err));
        }
    }

    http_server_init();

    xTaskCreatePinnedToCore(balance_task,   "balance",   4096, NULL, 10, NULL, 1);
    xTaskCreatePinnedToCore(command_task,   "cmd",       4096, NULL,  8, NULL, 0);
    xTaskCreatePinnedToCore(telemetry_task, "telemetry", 4096, NULL,  5, NULL, 1);

    /* Auto-arranque: calpha conocido, no se necesita cable */
    if (state.mpu_ready && state.motor_ready) {
        printf("AUTO_BALANCE,warmup,%d_steps\n", BALANCE_WARMUP_STEPS);
        for (int i = 0; i < BALANCE_WARMUP_STEPS; i++) {
            mpu_sample_t tmp = {0};
            if (mpu_read_sample(&tmp) == ESP_OK) {
                last_mpu_sample = tmp;
            }
            vTaskDelay(pdMS_TO_TICKS(BALANCE_PERIOD_MS));
        }
        state.alphad = BALANCE_ALPHAD_DEFAULT;
        state.balance_enabled = true;
        printf("AUTO_BALANCE,on,alphad=%.4f_rad (%.2f_deg)\n",
               state.alphad, state.alphad * RAD_TO_DEG);
    }
}
