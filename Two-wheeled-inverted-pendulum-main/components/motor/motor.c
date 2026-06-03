#include <stdio.h>
#include "motor.h"

void init_motor(){
    gpio_config_t gpio = {
       .pin_bit_mask   = (1ULL<<AIN1) | (1ULL<<AIN2) | (1ULL<<BIN1) | (1ULL<<BIN2) | (1ULL<<STBY_pin) ,
       .mode           = GPIO_MODE_OUTPUT,
       .pull_up_en     = GPIO_PULLUP_DISABLE,
       .pull_down_en   = GPIO_PULLDOWN_DISABLE,
       .intr_type      = GPIO_INTR_DISABLE
   };
   //initialize inputs
   ESP_ERROR_CHECK(gpio_config(&gpio));

    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .duty_resolution  = LEDC_DUTY_RES,
        .timer_num        = LEDC_TIMER,
        .freq_hz          = PWM_freq,  // Set output frequency at 4 kHz
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    ledc_channel_config_t ledc_channelA = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL_A,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = PWMA_pin,
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channelA));
    ledc_channel_config_t ledc_channelB = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL_B,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = PWMB_pin,
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channelB));
}
void set_motor(int pwmA, int pwmB){
    uint8_t stby_flag = 0;
    #ifndef INVERT_MOTOR_A
    if (pwmA > 0){
        gpio_set_level(AIN1, 1);
        gpio_set_level(AIN2, 0);
    }else if (pwmA < 0){
        gpio_set_level(AIN1, 0);
        gpio_set_level(AIN2, 1);
    }else{
        gpio_set_level(AIN1, 0);
        gpio_set_level(AIN2, 0);
        stby_flag |= 0x1;
    }
    #else
    if (pwmA > 0){
        gpio_set_level(AIN1, 0);
        gpio_set_level(AIN2, 1);
    }else if (pwmA < 0){
        gpio_set_level(AIN1, 1);
        gpio_set_level(AIN2, 0);
    }else{
        gpio_set_level(AIN1, 0);
        gpio_set_level(AIN2, 0);
        stby_flag |= 0x1;
    }
    #endif

    #ifndef INVERT_MOTOR_B
    if (pwmB > 0){
        gpio_set_level(BIN1, 1);
        gpio_set_level(BIN2, 0);
    }else if (pwmB < 0){
        gpio_set_level(BIN1, 0);
        gpio_set_level(BIN2, 1);
    }else{
        gpio_set_level(BIN1, 0);
        gpio_set_level(BIN2, 0);
        stby_flag |= 0x2;
    }
    #else
    if (pwmB > 0){
        gpio_set_level(BIN1, 0);
        gpio_set_level(BIN2, 1);
    }else if (pwmB < 0){
        gpio_set_level(BIN1, 1);
        gpio_set_level(BIN2, 0);
    }else{
        gpio_set_level(BIN1, 0);
        gpio_set_level(BIN2, 0);
        stby_flag |= 0x2;
    }
    #endif
    /*
    if(stby_flag == 0x3){
        gpio_set_level(STBY_pin, 0);
    }else{
        gpio_set_level(STBY_pin, 1);
    }
    */
    gpio_set_level(STBY_pin, 1);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_A, abs(pwmA));
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_A);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_B, abs(pwmB));
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_B);
}
