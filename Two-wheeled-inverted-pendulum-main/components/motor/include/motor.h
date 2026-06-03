#ifndef MOTOR_H
#define MOTOR_H

#include "esp_err.h"
#include "esp_log.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "hal/ledc_types.h"

static const char motor_tag[] = "motor";

/* Motor A = rueda izquierda, Motor B = rueda derecha. */
#define INVERT_MOTOR_A
//#define INVERT_MOTOR_B

/* Mapeo _TEORIA/MAPEO DE PINES.xlsx para L298N. */
#define PWMA_pin 14
#define AIN1     5
#define AIN2     4
#define STBY_pin 26
#define BIN1     2
#define BIN2     15
#define PWMB_pin 27

#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_DUTY_RES           LEDC_TIMER_7_BIT
#define LEDC_CHANNEL_A          LEDC_CHANNEL_0
#define LEDC_CHANNEL_B          LEDC_CHANNEL_1
#define PWM_freq 20000

void init_motor();
void set_motor(int pwmA, int pwmB);
#endif
