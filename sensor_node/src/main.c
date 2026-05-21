#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include "sensors/mmwave/uart.h"
#include "sensors/mmwave/mmwave.h"

//const struct device *uart0 = DEVICE_DT_GET(DT_NODELABEL(uart0));

int main(void)
{
    char response[18] = {0};
    set_normal_mode(response);
    //firmware_version(response);

    for(int i = 0; i < 18; i++)
    {
        printk("%02x ", response[i]);
        k_sleep(K_MSEC(50));
    }
    printk("\n");
}
