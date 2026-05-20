#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include "uart.h"

void send_byte(const struct device *uart, unsigned char byte)
{
    if(!device_is_ready(uart))
    {
        printk("UART not ready\n");
        return;
    }
    uart_poll_out(uart, byte);
}

unsigned char recv_byte(const struct device *uart)
{
    if(!device_is_ready(uart))
    {
        printk("UART not ready\n");
        return -1;
    }
    unsigned char byte = 0;
    while(uart_poll_in(uart, &byte) == -1);
    printk("Recv byte: %x\n", byte);
    return byte;
}
