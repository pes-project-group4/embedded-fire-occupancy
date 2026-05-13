#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>

const struct device *uart0 = DEVICE_DT_GET(DT_NODELABEL(uart0));

void init()
{
    if(!device_is_ready(uart0))
    {
        printk("UART0 not ready\n");
        return;
    }
    printk("UART0 is ready!\n");
}

void send_byte(unsigned char byte)
{
    uart_poll_out(uart0, byte);
}
