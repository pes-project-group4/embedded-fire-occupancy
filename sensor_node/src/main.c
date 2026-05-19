#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include "sensors/mmwave/uart.h"
#include "sensors/mmwave/mmwave.h"

//#define LED_NODE DT_ALIAS(led0)
//
//#if !DT_NODE_HAS_STATUS(LED_NODE, okay)
//#error "LED0 alias is not defined in the device tree"
//#endif

//static const struct gpio_dt_spec led =
//    GPIO_DT_SPEC_GET(LED_NODE, gpios);

int main(void)
{
    init();
    char response[24];
    firmware_version(response);
    for(int i = 0; i < 24; i++)
    {
        send_byte(response[i]);
        k_sleep(K_MSEC(500));
    }
}
