#ifndef UART_H
#define UART_H

void init();
void send_byte(unsigned char byte);
unsigned char recv_byte();

#endif
