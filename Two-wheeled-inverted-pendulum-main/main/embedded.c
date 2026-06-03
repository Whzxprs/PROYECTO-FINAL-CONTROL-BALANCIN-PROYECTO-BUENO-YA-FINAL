#include <math.h>
#include <stdio.h>

#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/idf_additions.h"
#include "freertos/projdefs.h"

#include "wifi_tcp.h"
#include "motor.h"
#include "mpu6050.h"
#include "internal_adc.h"
#include "encoder.h"
#include "uart.h"
#include "control.h"

#define EMBEBIDO

const char TAG[] = "balancin";

#define BUF_SIZE 256

volatile int connection_sock, socket_active = 0;



control_t ctrl;
TimerHandle_t control_timer_h = NULL;
EventGroupHandle_t xTcpEventGroup;
int encoderA, encoderB;
int16_t Ax, Gy;
uint16_t adc_read[8];

void TCP_receive(const int sock);
void control_task( void *pvParameters );



void app_main(void)
{
    init_wifi();
    xTaskCreate(tcp_server_task, "tcp_server", 4096, (void*)AF_INET, 17, NULL);
    init_internal_adc();
    init_encoder();
    init_motor();
    init_mpu6050();
    init_uart();
    init_control(&ctrl);

    xTcpEventGroup = xEventGroupCreate();

    /* Was the event group created successfully? */
    if( xTcpEventGroup == NULL )
    {
        /* The event group was not created because there was insufficient
           FreeRTOS heap available. */
        ESP_LOGE(TAG, "Error creating event group");
    }
    else
    {
        ESP_LOGI(TAG, "Event group was created successfully");
        /* The event group was created. */
    }
    
    ESP_LOGI(TAG, "Channel_%i, Channel_%i", adc1, adc2);

    printf("Ready to start\r\n");
    const char msg[] ="initialized\r\n";
    uart_write_bytes(UART_PORT, msg, sizeof(msg));

    xTaskCreatePinnedToCore(control_task, "control", 4096, NULL, 15, NULL, 1);
    
    //calculate_control(&ctrl);
    /*
    while (1) {
        read_encoder(&encoderA, &encoderB);
        set_motor(1023, -1023);
        read_adc(adc_read);
        read_mpu6050(&Ax, &Gy);
        printf("Positive\r\n");
        printf("encA: %i\tencB: %i\tAx: %i\tGy: %i\tA[1]: %u\tA[2]:%u\r\n", encoderA, encoderB, Ax, Gy, adc_read[3], adc_read[4]);
        vTaskDelay(pdMS_TO_TICKS(1000));


        read_encoder(&encoderA, &encoderB);
        set_motor(-1023, 1023);
        read_adc(adc_read);
        read_mpu6050(&Ax, &Gy);
        printf("Negative\r\n");
        printf("encA: %i\tencB: %i\tAx: %i\tGy: %i\tA[1]: %u\tA[2]:%u\r\n", encoderA, encoderB, Ax, Gy, adc_read[3], adc_read[4]);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    //*/
}

void control_task( void *pvParameters ){
    
    TickType_t xLastWakeTime;

    const TickType_t xFrequency = pdMS_TO_TICKS(10);

    xLastWakeTime = xTaskGetTickCount();

    for (; ;) {
        
        
        //read_adc(adc_read);
        //ESP_LOGI(TAG, "[0]: %u\t[1]: %u\t[2]: %u\t[3]: %u\t[4]: %u\t[5]: %u\t[6]: %u\t[7]: %u\t",adc_read[0] ,adc_read[1] ,adc_read[2] ,adc_read[3] ,adc_read[4] ,adc_read[5] ,adc_read[6] ,adc_read[7] );

        int sr,sl;
        adc_oneshot_read(ADC_unit, adc1, &sr);
        adc_oneshot_read(ADC_unit, adc2, &sl);
        //ESP_LOGI(TAG, "ADC: %i\t%i",sl>>4, sr>>4);
        // sr = (int)(((float)adc_read[3]*RATIO)+((float)adc_read[2]*(1-RATIO)));
        // sl = (int)(((float)adc_read[4]*RATIO)+((float)adc_read[5]*(1-RATIO)));
        //ESP_LOGI(TAG, "Sr=%i\tSl=%i", sr>>4, sl>>4);
        uint8_t mpu_data[14];
        read_mpu6050(mpu_data);
        Ax = mpu_data[1] + (((int16_t)mpu_data[0])<<8);
        Gy = mpu_data[11] + (((int16_t)mpu_data[10])<<8);


        read_encoder(&encoderA, &encoderB);

        //printf("encA: %i\tencB: %i\tAx: %i\tGy: %i\tA[1]: %u\tA[2]:%u\r\n", encoderA, encoderB, Ax, Gy, adc_read[3], adc_read[4]);
        ctrl.sl = (float)(sl>>4);
        ctrl.sr = (float)(sr>>4);
        ctrl.Ax = (float)Ax;
        ctrl.Gy = (float)Gy;
        ctrl.incl = (float) encoderA;
        ctrl.incr = (float) encoderB;
        //ESP_LOGI(TAG, "incr: %f\tencoderB: %i", ctrl.incr, encoderB);

        uint8_t packet[9];
        packet[0] = 0xAA;

        #ifdef EMBEBIDO

        calculate_control(&ctrl);
        packet[1] = ctrl.vdg;
        packet[2] = ctrl.vg;
        packet[3] = ctrl.thetag;
        packet[4] = ctrl.alphag;
        packet[5] = ctrl.omegalg;
        packet[6] = ctrl.omegarg;
        packet[7] = ctrl.ulg;
        packet[8] = ctrl.urg;
        set_motor(ctrl.uWl, ctrl.uWr);

        if(xEventGroupGetBits(xTcpEventGroup) & 0x1){
            int to_write = 9;
            while (to_write > 0) {
                int written = send(connection_sock, packet + (PACKET_SIZE - to_write), to_write, 0);
                if (written < 0) {
                    ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                    break;
                    // Failed to retransmit, giving up
                }
                to_write -= written;
            }
        }
        uart_write_bytes(UART_PORT, packet, sizeof(packet));
        
        
        #else
        //*/
        if(encoderB > 127){
            packet[2] = 127;
        }else if(encoderB < -128){
            packet[2] = -128;
        }else{
            packet[2] = (int8_t) (encoderB);
        }
        /*/
        if(encoderB > 127){
            packet[1] = 255;
        }else if(encoderB < -128){
            packet[1] = 0;
        }else{
            packet[1] = (int8_t) (encoderB + 127);
        }
        /*/
        if(encoderA > 127){
            packet[1] = 127;
        }else if(encoderA < -128){
            packet[1] = -128;
        }else{
            packet[1] = (int8_t) (encoderA);
        }
        /*/
        if(encoderA > 127){
            packet[2] = 255;
        }else if(encoderA < -128){
            packet[2] = 0;
        }else{
            packet[2] = (int8_t) (encoderA + 127);
        }
        //*/
        packet[3] = mpu_data[0];
        packet[4] = mpu_data[1];
        packet[5] = mpu_data[10];
        packet[6] = mpu_data[11];
        packet[7] = (uint8_t)(sr>>4);
        packet[8] = (uint8_t)(sl>>4);
        if(xEventGroupGetBits(xTcpEventGroup) & 0x1){
            int to_write = 9;
            while (to_write > 0) {
                int written = send(connection_sock, packet + (PACKET_SIZE - to_write), to_write, 0);
                if (written < 0) {
                    ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                    break;
                    // Failed to retransmit, giving up
                }
                to_write -= written;
            }
        }else {
            uart_write_bytes(UART_PORT, packet, sizeof(packet));
            int8_t rx_buffer[2];
            uart_flush_input(UART_PORT);
            int rxBytes = uart_read_bytes(UART_PORT, rx_buffer, sizeof(rx_buffer), pdMS_TO_TICKS(10));
            
            if (rxBytes >= 2) {
                printf("pwm:%i,%i\r\n", rx_buffer[0], rx_buffer[1]);
                set_motor((int8_t)rx_buffer[1], (int8_t)rx_buffer[0]);
            }else {
                set_motor(0, 0);
            }
        }
        
        #endif
        

        //printf("Alpha: %f\tTheta: %f\tuWl: %f\tuWr: %f\til: %f\tir: %f\r\n",ctrl.alpha, ctrl.theta, ctrl.uWl, ctrl.uWr, ctrl.incl, ctrl.incr);


        vTaskDelayUntil( &xLastWakeTime, xFrequency );

    }
    
}



void TCP_receive(int sock)
{
    
    xEventGroupSetBits( xTcpEventGroup,
                                0x1 );
    
    //socket_active = 1;
    connection_sock = sock;
    int len;
    uint8_t rx_buffer[BUF_SIZE];

    do {
        len = recv(sock, rx_buffer, sizeof(rx_buffer), 0);
        if (len < 0) {
            ESP_LOGE(TAG, "Error occurred during receiving: errno %d", errno);
            set_motor(0, 0);
        } else if (len == 0) {
            ESP_LOGW(TAG, "Connection closed");
            set_motor(0, 0);
        } else {
            /*/
            printf("len:%i\t0x", len);
            for(int i = 0; i<len; i++){
                printf("%i",rx_buffer[i]);
            }
            printf("\r\n");
            //*/
#ifndef EMBEBIDO
            int pwmA, pwmB;
            if(rx_buffer[1] > 128){
                pwmA = -(rx_buffer[0] - 128);
            }else{
                pwmA = rx_buffer[0];
            }
            if(rx_buffer[0] > 128){
                pwmB = -(rx_buffer[1] - 128);
            }else{
                pwmB = rx_buffer[1];
            }
            set_motor(pwmA, pwmB);
#endif
        }

    } while (len > 0);

    xEventGroupClearBits( xTcpEventGroup,
                                0x1 );
    //socket_active = 0;
}
