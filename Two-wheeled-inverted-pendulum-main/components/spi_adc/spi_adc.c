#include <stdio.h>
#include "spi_adc.h"

void init_adc(){
    spi_bus_config_t spi_bus_conf = {
        .mosi_io_num = PIN_NUM_MOSI,
        .miso_io_num = PIN_NUM_MISO,
        .sclk_io_num = PIN_NUM_CLK,
        .quadhd_io_num = -1,
        .quadwp_io_num = -1,
        .data4_io_num = -1,
        .data5_io_num = -1,
        .data6_io_num = -1,
        .data7_io_num = -1,
        .flags = SPICOMMON_BUSFLAG_MASTER,
        .max_transfer_sz = 32
    };

    spi_device_interface_config_t spi_dev_conf = {
        .mode = 0,
        .clock_source = SPI_CLK_SRC_DEFAULT,
        .clock_speed_hz = 1000000,
        .input_delay_ns = 0,
        .spics_io_num = PIN_NUM_CS,
        .duty_cycle_pos = 128,
        .queue_size = 1,
        .flags = 0,
        .pre_cb = NULL,
        .post_cb = NULL
    };
    ESP_LOGI(adc_tag, "Initializing bus");
    spi_bus_initialize(SPI2_HOST, &spi_bus_conf, SPI_DMA_DISABLED);
    ESP_LOGI(adc_tag, "Adding device");
    spi_bus_add_device(SPI2_HOST, &spi_dev_conf, &spi_handle);
    ESP_LOGI(adc_tag, "SPI initialized");
}
void read_adc(uint16_t data[8]){
    uint8_t rx_buf[10], tx_buf[]={
        0x1, 0x80, 0x0
    };
    spi_transaction_t spi_transaction = {
        .flags = 0,
        .length = sizeof(tx_buf)*8,
        .rxlength = 0,
        .tx_buffer = tx_buf,
        .rx_buffer = rx_buf
    };
    spi_device_transmit(spi_handle, &spi_transaction);
    data[0] = (((uint16_t)(rx_buf[1] & 0x3))<<8) + rx_buf[2];
    //printf("0x%x, 0x%x = %u\r\n", rx_buf[1], rx_buf[2], data[0]);
    //*/
    tx_buf[1] = 0x90;
    spi_device_transmit(spi_handle, &spi_transaction);
    data[1] = (((uint16_t)(rx_buf[1] & 0x3))<<8) + rx_buf[2];
    tx_buf[1] = 0xA0;
    spi_device_transmit(spi_handle, &spi_transaction);
    data[2] = (((uint16_t)(rx_buf[1] & 0x3))<<8) + rx_buf[2];
    tx_buf[1] = 0xB0;
    spi_device_transmit(spi_handle, &spi_transaction);
    data[3] = (((uint16_t)(rx_buf[1] & 0x3))<<8) + rx_buf[2];
    tx_buf[1] = 0xC0;    esp_rom_delay_us(1);
    spi_device_transmit(spi_handle, &spi_transaction);
    data[4] = (((uint16_t)(rx_buf[1] & 0x3))<<8) + rx_buf[2];
    tx_buf[1] = 0xD0;
    spi_device_transmit(spi_handle, &spi_transaction);
    data[5] = (((uint16_t)(rx_buf[1] & 0x3))<<8) + rx_buf[2];
    tx_buf[1] = 0xE0;
    spi_device_transmit(spi_handle, &spi_transaction);
    data[6] = (((uint16_t)(rx_buf[1] & 0x3))<<8) + rx_buf[2];
    tx_buf[1] = 0xF0;
    spi_device_transmit(spi_handle, &spi_transaction);
    data[7] = (((uint16_t)(rx_buf[1] & 0x3))<<8) + rx_buf[2];
}
