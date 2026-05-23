#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#include "sensors/bme680/bme680.h"
#include "sensors/mlx90614/mlx90614.h"
#include "sensors/mmwave/mmwave.h"
#include "sensors/spw2430/spw2430.h"
#include "registers/register_map.h"

int main(void)
{
    int ret;

    printk("\n=== Sensor node starting ===\n");

    /* Register map first — every sensor publishes into it. */
    regmap_init();
    printk("Register map ready (chip id 0x%02X)\n", 0x42);

    /* --- HMMD mmWave -------------------------------------------------- */
    ret = mmwave_init();
    if (ret < 0) {
        printk("mmwave_init failed: %d\n", ret);
    } else {
        printk("HMMD mmWave ready\n");

        /* Apply a starting configuration:
         *   max_range_gate = 5  -> cap detection at gate 5 (~350 cm)
         *   absence_delay_s = 5 -> radar holds ON for 5 s before reporting OFF
         */
        struct mmwave_config cfg = {
            .max_range_gate  = 5,
            .absence_delay_s = 5,
        };
        if (mmwave_apply_config(&cfg) == 0) {
            printk("mmWave config applied: max_gate=%u (~%u cm), absence=%us\n",
                   cfg.max_range_gate,
                   (unsigned)(cfg.max_range_gate + 1) * 70,
                   cfg.absence_delay_s);
        } else {
            printk("mmWave config apply failed\n");
        }
    }

    printk("\n");

    while (1) {
        uint8_t ctrl = regmap_get_ctrl();
        struct mmwave_state    mm;

        if ((ctrl & CTRL_EN_MMWAVE) && mmwave_get_state(&mm) == 0) {
            bool occupied = mmwave_is_occupied(2000);
            printk("Occupied=%s", occupied ? "yes" : "no ");
            if (mm.present && mm.range_cm > 0) {
                printk(" range=%u cm", (unsigned)mm.range_cm);
            }
            printk("  ");
            regmap_publish_mmwave(occupied, mm.range_cm);
        }

        printk("\n");
        k_sleep(K_SECONDS(1));
    }
}