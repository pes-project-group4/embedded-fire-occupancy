#include "config_mode.h"

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/printk.h>

#include <ctype.h>
#include <errno.h>
#include <limits.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "../drivers/sensor/remote_pico/remote_pico.h"
#include "remote_pico_interrupts.h"

#define LINE_BUF_LEN 32
#define DEFAULT_MMWAVE_MAX_GATE 5
#define DEFAULT_MMWAVE_ABSENCE_S 5

typedef bool (*exit_requested_fn)(void);

static const struct device *const console =
    DEVICE_DT_GET(DT_CHOSEN(zephyr_console));

static uint8_t current_max_gate = DEFAULT_MMWAVE_MAX_GATE;
static uint16_t current_absence_s = DEFAULT_MMWAVE_ABSENCE_S;

static void uart_echo(unsigned char c)
{
    if (device_is_ready(console)) {
        uart_poll_out(console, c);
    }
}

static bool read_line(char *buf, size_t len, exit_requested_fn exit_requested)
{
    size_t pos = 0;

    if (buf == NULL || len == 0 || !device_is_ready(console)) {
        return false;
    }

    while (1) {
        unsigned char c;

        if (exit_requested != NULL && exit_requested()) {
            printk("\n");
            return false;
        }

        if (uart_poll_in(console, &c) != 0) {
            k_msleep(20);
            continue;
        }

        if (c == '\r' || c == '\n') {
            buf[pos] = '\0';
            uart_echo('\r');
            uart_echo('\n');
            return true;
        }

        if (c == '\b' || c == 0x7F) {
            if (pos > 0) {
                pos--;
                uart_echo('\b');
                uart_echo(' ');
                uart_echo('\b');
            }
            continue;
        }

        if (isprint(c) && pos < len - 1) {
            buf[pos++] = (char)c;
            uart_echo(c);
        }
    }
}

static void skip_spaces(const char **p)
{
    while (isspace((unsigned char)**p)) {
        (*p)++;
    }
}

static int parse_centi_c(const char *s, int32_t *out)
{
    const char *p = s;
    bool negative = false;
    int32_t whole = 0;
    int32_t frac = 0;
    int frac_digits = 0;

    if (s == NULL || out == NULL) {
        return -EINVAL;
    }

    skip_spaces(&p);

    if (*p == '-' || *p == '+') {
        negative = (*p == '-');
        p++;
    }

    if (!isdigit((unsigned char)*p)) {
        return -EINVAL;
    }

    while (isdigit((unsigned char)*p)) {
        if (whole > (INT32_MAX / 100)) {
            return -ERANGE;
        }
        whole = (whole * 10) + (*p - '0');
        p++;
    }

    if (*p == '.') {
        p++;
        while (isdigit((unsigned char)*p)) {
            if (frac_digits < 2) {
                frac = (frac * 10) + (*p - '0');
                frac_digits++;
            }
            p++;
        }
    }

    while (frac_digits < 2) {
        frac *= 10;
        frac_digits++;
    }

    skip_spaces(&p);
    if (*p != '\0') {
        return -EINVAL;
    }

    *out = (whole * 100) + frac;
    if (negative) {
        *out = -*out;
    }

    return 0;
}

static int parse_u32_range(const char *s, uint32_t min, uint32_t max,
                           uint32_t *out)
{
    char *end;
    unsigned long value;

    if (s == NULL || out == NULL) {
        return -EINVAL;
    }

    errno = 0;
    value = strtoul(s, &end, 10);
    if (end == s || errno != 0) {
        return -EINVAL;
    }

    while (isspace((unsigned char)*end)) {
        end++;
    }

    if (*end != '\0' || value < min || value > max) {
        return -EINVAL;
    }

    *out = (uint32_t)value;
    return 0;
}

static void print_threshold(int32_t centi_c)
{
    int32_t frac = centi_c % 100;

    if (frac < 0) {
        frac = -frac;
    }

    printk("%d.%02d C", centi_c / 100, frac);
}

static void print_menu(void)
{
    printk("\n=== Configuration mode ===\n");
    printk("1) MLX90614 fire threshold: ");
    print_threshold(remote_pico_interrupts_get_fire_threshold());
    printk("\n");
    printk("2) mmWave max range gate: %u (~%u cm)\n",
           current_max_gate, (unsigned)(current_max_gate + 1) * 70);
    printk("3) mmWave absence delay: %u s\n", current_absence_s);
    printk("q) Return to normal mode\n");
    printk("> ");
}

static void configure_threshold(exit_requested_fn exit_requested)
{
    char line[LINE_BUF_LEN];
    int32_t centi_c;
    int ret;

    printk("Fire threshold in C (example 30 or 30.50): ");
    if (!read_line(line, sizeof(line), exit_requested)) {
        return;
    }

    ret = parse_centi_c(line, &centi_c);
    if (ret != 0) {
        printk("Invalid temperature\n");
        return;
    }

    ret = remote_pico_interrupts_set_fire_threshold(centi_c);
    if (ret == 0) {
        printk("Fire threshold updated to ");
        print_threshold(centi_c);
        printk("\n");
    } else {
        printk("Failed to update fire threshold: %d\n", ret);
    }
}

static void configure_max_gate(const struct device *sensor,
                               exit_requested_fn exit_requested)
{
    char line[LINE_BUF_LEN];
    uint32_t value;
    int ret;

    printk("Max range gate (0..15, each gate is about 70 cm): ");
    if (!read_line(line, sizeof(line), exit_requested)) {
        return;
    }

    ret = parse_u32_range(line, 0, 15, &value);
    if (ret != 0) {
        printk("Invalid max range gate\n");
        return;
    }

    remote_pico_interrupts_lock_sensor();
    ret = remote_pico_set_mmwave_max_gate(sensor, (uint8_t)value);
    remote_pico_interrupts_unlock_sensor();

    if (ret == 0) {
        current_max_gate = (uint8_t)value;
        printk("mmWave max range gate updated to %u\n", current_max_gate);
    } else {
        printk("Failed to update mmWave max range gate: %d\n", ret);
    }
}

static void configure_absence_delay(const struct device *sensor,
                                    exit_requested_fn exit_requested)
{
    char line[LINE_BUF_LEN];
    uint32_t value;
    int ret;

    printk("Absence delay in seconds (0..65535): ");
    if (!read_line(line, sizeof(line), exit_requested)) {
        return;
    }

    ret = parse_u32_range(line, 0, UINT16_MAX, &value);
    if (ret != 0) {
        printk("Invalid absence delay\n");
        return;
    }

    remote_pico_interrupts_lock_sensor();
    ret = remote_pico_set_mmwave_absence_delay(sensor, (uint16_t)value);
    remote_pico_interrupts_unlock_sensor();

    if (ret == 0) {
        current_absence_s = (uint16_t)value;
        printk("mmWave absence delay updated to %u s\n", current_absence_s);
    } else {
        printk("Failed to update mmWave absence delay: %d\n", ret);
    }
}

void config_mode_run(const struct device *sensor,
                     exit_requested_fn exit_requested)
{
    char line[LINE_BUF_LEN];

    if (!device_is_ready(console)) {
        printk("console UART not ready\n");
        return;
    }

    printk("\n--- Entered configuration mode ---\n");
    printk("Press GPIO20 again, or enter q, to return to normal mode.\n");

    while (1) {
        print_menu();
        if (!read_line(line, sizeof(line), exit_requested)) {
            break;
        }

        switch (line[0]) {
        case '1':
            configure_threshold(exit_requested);
            break;
        case '2':
            configure_max_gate(sensor, exit_requested);
            break;
        case '3':
            configure_absence_delay(sensor, exit_requested);
            break;
        case 'q':
        case 'Q':
            return;
        default:
            printk("Unknown option\n");
            break;
        }
    }
}
