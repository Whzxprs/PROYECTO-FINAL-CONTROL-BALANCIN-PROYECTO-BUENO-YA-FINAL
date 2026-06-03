idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES
        balancin_control
        encoder_quad
        line_adc
        motor_l298n
        mpu6050
        wifi_sta_udp
)
