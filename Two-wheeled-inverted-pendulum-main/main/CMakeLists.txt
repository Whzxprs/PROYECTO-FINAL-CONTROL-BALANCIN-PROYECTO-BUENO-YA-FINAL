idf_component_register(SRCS "embedded.c"
                    INCLUDE_DIRS "."
                    REQUIRES "mpu6050" "motor" "encoder" "control" "internal_adc" "esp_adc" "wifi_tcp" "nvs_flash" "uart")
list(APPEND EXTRA_COMPONENT_DIRS components)
