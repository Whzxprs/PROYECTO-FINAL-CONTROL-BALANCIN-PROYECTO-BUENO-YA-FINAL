#include <stdio.h>
#include "encoder.h"

pcnt_unit_handle_t pcntA_unit, pcntB_unit;
pcnt_channel_handle_t pcntA1_chan = NULL, pcntA2_chan = NULL, pcntB1_chan = NULL, pcntB2_chan = NULL;

void init_encoder(){
    pcnt_unit_config_t unit_config = { 
        .high_limit = PCNT_HIGH_LIMIT,
        .low_limit  = PCNT_LOW_LIMIT,
    };
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcntA_unit));
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcntB_unit));
    #ifdef INVERT_ENCODER_A
    pcnt_chan_config_t chanA1_config = {
        .edge_gpio_num  = ENC_A2,
        .level_gpio_num = ENC_A1,
    };
    pcnt_chan_config_t chanA2_config = {
        .edge_gpio_num  = ENC_A1,
        .level_gpio_num = ENC_A2,
    };
    #else
    pcnt_chan_config_t chanA1_config = {
        .edge_gpio_num  = ENC_A1,
        .level_gpio_num = ENC_A2,
    };
    pcnt_chan_config_t chanA2_config = {
        .edge_gpio_num  = ENC_A2,
        .level_gpio_num = ENC_A1,
    }; 
    #endif
    ESP_ERROR_CHECK(pcnt_new_channel(pcntA_unit, &chanA1_config, &pcntA1_chan));
    ESP_ERROR_CHECK(pcnt_new_channel(pcntA_unit, &chanA2_config, &pcntA2_chan));
    #ifdef INVERT_ENCODER_B
    pcnt_chan_config_t chanB1_config = {
        .edge_gpio_num  = ENC_B2,
        .level_gpio_num = ENC_B1,
    };  
    pcnt_chan_config_t chanB2_config = {
        .edge_gpio_num  = ENC_B1,
        .level_gpio_num = ENC_B2,
    };   
    #else
    pcnt_chan_config_t chanB1_config = {
        .edge_gpio_num  = ENC_B1,
        .level_gpio_num = ENC_B2,
    };  
    pcnt_chan_config_t chanB2_config = {
        .edge_gpio_num  = ENC_B2,
        .level_gpio_num = ENC_B1,
    };   
    #endif
    ESP_ERROR_CHECK(pcnt_new_channel(pcntB_unit, &chanB1_config, &pcntB1_chan));
    ESP_ERROR_CHECK(pcnt_new_channel(pcntB_unit, &chanB2_config, &pcntB2_chan));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcntA1_chan, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcntA1_chan, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcntA2_chan, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcntA2_chan, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcntB1_chan, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcntB1_chan, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcntB2_chan, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcntB2_chan, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_unit_enable(pcntA_unit));
    ESP_ERROR_CHECK(pcnt_unit_enable(pcntB_unit));
    ESP_ERROR_CHECK(pcnt_unit_start(pcntA_unit));
    ESP_ERROR_CHECK(pcnt_unit_start(pcntB_unit));
    ESP_LOGI(encoder_tag, "pcnt units created");
}

void read_encoder(int *encoderA, int *encoderB){
    pcnt_unit_get_count(pcntA_unit, encoderA);
    pcnt_unit_clear_count(pcntA_unit);
    pcnt_unit_get_count(pcntB_unit, encoderB);
    pcnt_unit_clear_count(pcntB_unit);
}
