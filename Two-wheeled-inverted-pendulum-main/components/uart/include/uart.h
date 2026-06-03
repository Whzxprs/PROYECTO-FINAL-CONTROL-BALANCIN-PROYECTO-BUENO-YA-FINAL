#ifndef UART_H
#define UART_H

#include "driver/uart.h"

#define TXD_PIN (23)
#define RXD_PIN (25)

#define UART_PORT UART_NUM_1
#define UART_BAUDRATE 115200
#define UART_BUF_SIZE 512

void init_uart(void);

#endif
