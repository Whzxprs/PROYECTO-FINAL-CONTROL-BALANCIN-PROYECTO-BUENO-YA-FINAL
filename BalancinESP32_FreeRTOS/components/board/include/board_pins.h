#pragma once

#include "driver/gpio.h"

/*
 * Pin map base from _TEORIA/MAPEO DE PINES.xlsx for ESP32 DevKit V1 30 pins,
 * corrected with physical tests for swapped sensors/encoders.
 */

#define BALANCIN_I2C_SDA_GPIO GPIO_NUM_21
#define BALANCIN_I2C_SCL_GPIO GPIO_NUM_22

#define BALANCIN_LINE_LEFT_GPIO  GPIO_NUM_35
#define BALANCIN_LINE_RIGHT_GPIO GPIO_NUM_34

#define BALANCIN_ENCODER_RIGHT_A_GPIO GPIO_NUM_33
#define BALANCIN_ENCODER_RIGHT_B_GPIO GPIO_NUM_32
#define BALANCIN_ENCODER_LEFT_A_GPIO  GPIO_NUM_18
#define BALANCIN_ENCODER_LEFT_B_GPIO  GPIO_NUM_19

/* L298N connector order: ENA, IN1, IN2, IN3, IN4, ENB. */
#define BALANCIN_MOTOR_LEFT_PWM_GPIO  GPIO_NUM_14
#define BALANCIN_MOTOR_LEFT_IN1_GPIO  GPIO_NUM_5
#define BALANCIN_MOTOR_LEFT_IN2_GPIO  GPIO_NUM_4
#define BALANCIN_MOTOR_RIGHT_IN1_GPIO GPIO_NUM_2
#define BALANCIN_MOTOR_RIGHT_IN2_GPIO GPIO_NUM_15
#define BALANCIN_MOTOR_RIGHT_PWM_GPIO GPIO_NUM_27

/*
 * Flip these when a positive PWM command makes that wheel rotate backward.
 * With mirrored motors, one side commonly needs inversion so equal PWM signs
 * move both wheels forward instead of making the robot spin.
 */
#define BALANCIN_MOTOR_LEFT_INVERT   1
#define BALANCIN_MOTOR_RIGHT_INVERT  0
#define BALANCIN_ENCODER_RIGHT_INVERT 1
#define BALANCIN_ENCODER_LEFT_INVERT  0

#define BALANCIN_WIFI_SSID "balancin"
#define BALANCIN_WIFI_PASS "1q2w3e4r"
#define BALANCIN_TCP_PORT 4545
