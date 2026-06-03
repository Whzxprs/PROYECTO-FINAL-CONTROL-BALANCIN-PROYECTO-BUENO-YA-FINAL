#include "mpu6050.h"

#include "board_pins.h"
#include "driver/i2c_master.h"
#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define MPU6050_ADDR        0x68
#define MPU6050_REG_CONFIG  0x1A
#define MPU6050_REG_GYRO    0x1B
#define MPU6050_REG_ACCEL   0x1C
#define MPU6050_REG_PWR1    0x6B
#define MPU6050_REG_PWR2    0x6C
#define MPU6050_REG_DATA    0x3B

static const char *TAG = "mpu6050";

static i2c_master_bus_handle_t bus_handle;
static i2c_master_dev_handle_t dev_handle;

static esp_err_t write_reg(uint8_t reg, uint8_t value)
{
    uint8_t data[2] = {reg, value};
    return i2c_master_transmit(dev_handle, data, sizeof(data), 100);
}

esp_err_t mpu6050_init(void)
{
    i2c_master_bus_config_t bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_0,
        .scl_io_num = BALANCIN_I2C_SCL_GPIO,
        .sda_io_num = BALANCIN_I2C_SDA_GPIO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_RETURN_ON_ERROR(i2c_new_master_bus(&bus_config, &bus_handle),
                        TAG, "i2c_new_master_bus failed");

    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = MPU6050_ADDR,
        .scl_speed_hz = 400000,
    };
    ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(bus_handle, &dev_config, &dev_handle),
                        TAG, "i2c_master_bus_add_device failed");

    ESP_RETURN_ON_ERROR(write_reg(MPU6050_REG_PWR1, 0x80), TAG, "mpu reset failed");
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_RETURN_ON_ERROR(write_reg(MPU6050_REG_PWR1, 0x00), TAG, "mpu wake failed");
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_RETURN_ON_ERROR(write_reg(MPU6050_REG_PWR2, 0x00), TAG, "mpu pwr2 failed");
    ESP_RETURN_ON_ERROR(write_reg(MPU6050_REG_CONFIG, 0x01), TAG, "mpu config failed");
    ESP_RETURN_ON_ERROR(write_reg(MPU6050_REG_GYRO, 0x00), TAG, "mpu gyro failed");
    ESP_RETURN_ON_ERROR(write_reg(MPU6050_REG_ACCEL, 0x00), TAG, "mpu accel failed");

    return ESP_OK;
}

esp_err_t mpu6050_read_raw(int16_t *ax, int16_t *gy)
{
    uint8_t reg = MPU6050_REG_DATA;
    uint8_t data[14] = {0};
    esp_err_t err = i2c_master_transmit_receive(dev_handle, &reg, 1, data, sizeof(data), 10);
    if (err != ESP_OK) {
        return err;
    }

    *ax = (int16_t)((data[0] << 8) | data[1]);
    *gy = (int16_t)((data[10] << 8) | data[11]);
    return ESP_OK;
}
