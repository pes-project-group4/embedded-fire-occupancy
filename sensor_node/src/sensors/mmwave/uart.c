#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include "uart.h"

const struct device *uart1 = DEVICE_DT_GET(DT_NODELABEL(uart1));

void init()
{
    if(!device_is_ready(uart1))
    {
        printk("UART1 not ready\n");
        return;
    }
    printk("UART1 is ready!\n");
}

void send_byte(unsigned char byte)
{
    uart_poll_out(uart1, byte);
}

unsigned char recv_byte()
{
    unsigned char byte;
    uart_poll_in(uart1, &byte);
    return byte;
}
