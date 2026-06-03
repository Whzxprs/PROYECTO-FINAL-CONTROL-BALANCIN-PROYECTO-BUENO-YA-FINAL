idf_component_register(
    SRCS "motor_l298n.c"
    INCLUDE_DIRS "include"
    REQUIRES board esp_driver_gpio esp_driver_ledc
)
