#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include "sensors/mmwave/uart.h"
#include "sensors/mmwave/mmwave.h"

const struct device *uart0 = DEVICE_DT_GET(DT_NODELABEL(uart0));

int main(void)
{
    char response[24] = {0};
    //printk("a");
    //set_normal_mode(response);
    firmware_version(response);

    for(int i = 0; i < 24; i++)
    {
        k_sleep(K_MSEC(50));
        printk("%x ", response[i]);
    }
    printk("\n");
}
