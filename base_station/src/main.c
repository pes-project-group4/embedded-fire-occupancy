#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/sys_io.h>

/* RP2350 hardware register addresses */
#define IO_BANK0_BASE       0x40028000
#define GPIO18_CTRL         (IO_BANK0_BASE + (18 * 8) + 4)

#define PADS_BANK0_BASE     0x40038000
#define PADS_GPIO18         (PADS_BANK0_BASE + 4 + (18 * 4))

#define SIO_BASE            0xD0000000
#define SIO_GPIO_OE_SET     (SIO_BASE + 0x024)
#define SIO_GPIO_OUT_SET    (SIO_BASE + 0x014)
#define SIO_GPIO_OUT_CLR    (SIO_BASE + 0x018)

/* LED still uses Zephyr API */
static const struct gpio_dt_spec led =
    GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);

int main(void)
{
    if (!device_is_ready(led.port)) {
        return 0;
    }
    gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE);

    /* Force GP18 to SIO (GPIO) function = 5 */
    sys_write32(5, GPIO18_CTRL);

    /* Configure pad: enable output, disable input isolation */
    uint32_t pad_val = sys_read32(PADS_GPIO18);
    pad_val &= ~(1 << 7);  /* clear OD (output disable) */
    pad_val |= (1 << 6);   /* set IE (input enable) */
    sys_write32(pad_val, PADS_GPIO18);

    /* Set GP18 as output */
    sys_write32(1u << 18, SIO_GPIO_OE_SET);

    /* Slow blink test first — 2 seconds */
    for (int i = 0; i < 2; i++) {
        gpio_pin_set_dt(&led, 1);
        sys_write32(1u << 18, SIO_GPIO_OUT_SET);
        k_msleep(1000);

        gpio_pin_set_dt(&led, 0);
        sys_write32(1u << 18, SIO_GPIO_OUT_CLR);
        k_msleep(1000);
    }

    /* Now play 1kHz tone */
    while (1) {
        gpio_pin_set_dt(&led, 1);

        for (int i = 0; i < 2000; i++) {
            sys_write32(1u << 18, SIO_GPIO_OUT_SET);
            k_busy_wait(500);
            sys_write32(1u << 18, SIO_GPIO_OUT_CLR);
            k_busy_wait(500);
        }

        gpio_pin_set_dt(&led, 0);
        k_msleep(1000);
    }
}