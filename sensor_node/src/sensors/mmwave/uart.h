#ifndef UART_H
#define UART_H

#include <zephyr/drivers/uart.h>

void send_byte(const struct device *uart, unsigned char byte);
unsigned char recv_byte(const struct device *uart);

#endif
