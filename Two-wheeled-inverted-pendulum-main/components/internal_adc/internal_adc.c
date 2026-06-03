#include <stdio.h>
#include "internal_adc.h"

adc_oneshot_unit_handle_t ADC_unit;
adc_unit_t adc_unit;
adc_channel_t adc1, adc2;

void init_internal_adc(){
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1,
        .clk_src = ADC_RTC_CLK_SRC_DEFAULT,
        .ulp_mode = ADC_ULP_MODE_DISABLE
    };
    adc_oneshot_new_unit(&init_config, &ADC_unit);

    adc_oneshot_chan_cfg_t config_chan = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12
    };

    adc_oneshot_io_to_channel(ADC1, &adc_unit, &adc1);
    adc_oneshot_io_to_channel(ADC2, &adc_unit, &adc2);

    adc_oneshot_config_channel(ADC_unit, adc1, &config_chan);
    adc_oneshot_config_channel(ADC_unit, adc2, &config_chan);
}
