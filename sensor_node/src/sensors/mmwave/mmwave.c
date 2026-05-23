#include "mmwave.h"

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/ring_buffer.h>
#include <errno.h>
#include <stdlib.h>
#include <string.h>

#define UART_NODE       DT_NODELABEL(uart1)
#define RX_RINGBUF_SIZE 256
#define LINE_BUF_LEN    32

// Put radar to normal ASCII output mode
static const uint8_t CMD_NORMAL_MODE[] = {
    0xFD, 0xFC, 0xFB, 0xFA,     // header
    0x08, 0x00,                 // length
    0x12, 0x00,                 // command
    0x00, 0x00,
    0x64, 0x00, 0x00, 0x00,     // normal mode
    0x04, 0x03, 0x02, 0x01      // tail
};

static const struct device *uart_dev;

static uint8_t rx_ringbuf_storage[RX_RINGBUF_SIZE];
static struct ring_buf rx_ringbuf;

static struct k_spinlock state_lock;
static struct mmwave_state cur_state;

static char line_buf[LINE_BUF_LEN];
static size_t line_len;

static bool initialized;

static void parse_work_handler(struct k_work *work);
static K_WORK_DEFINE(parse_work, parse_work_handler);

// Read UART data and pass it to the parser workqueue
static void uart_isr(const struct device *dev, void *user_data)
{
    ARG_UNUSED(user_data);

    if (!uart_irq_update(dev) || !uart_irq_rx_ready(dev)) {
        return;
    }

    uint8_t buf[64];
    int len = uart_fifo_read(dev, buf, sizeof(buf));

    if (len > 0) {
        ring_buf_put(&rx_ringbuf, buf, len);
        k_work_submit(&parse_work);
    }
}

// Parse one text line from the radar.
static void parse_line(const char *line)
{
    bool new_present = cur_state.present;
    uint16_t new_range_cm = cur_state.range_cm;
    bool should_update = false;

    if (strcmp(line, "ON") == 0) {
        new_present = true;
        should_update = true;
    } else if (strncmp(line, "OFF", 3) == 0) {
        new_present = false;
        new_range_cm = 0;
        should_update = true;
    } else if (strncmp(line, "Range ", 6) == 0) {
        int cm = atoi(line + 6);

        if (cm >= 0 && cm <= 65535) {
            new_range_cm = (uint16_t)cm;
            should_update = true;
        }
    }

    if (!should_update) return;

    K_SPINLOCK(&state_lock) {
        cur_state.present = new_present;
        cur_state.range_cm = new_range_cm;
        cur_state.last_update_ms = k_uptime_get_32();
    }
}

static void feed_byte(uint8_t byte)
{
    if (byte == '\n') {
        if (line_len > 0 && line_buf[line_len - 1] == '\r') {
            line_len--;
        }

        line_buf[line_len] = '\0';

        if (line_len > 0) {
            parse_line(line_buf);
        }

        line_len = 0;
        return;
    }

    if (line_len < LINE_BUF_LEN - 1) {
        line_buf[line_len++] = (char)byte;
    } else {
        line_len = 0;
    }
}

static void parse_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);

    uint8_t byte;

    while (ring_buf_get(&rx_ringbuf, &byte, 1) == 1) {
        feed_byte(byte);
    }
}

static void send_bytes(const uint8_t *buf, size_t len)
{
    for (size_t i = 0; i < len; i++) {
        uart_poll_out(uart_dev, buf[i]);
    }
}

int mmwave_init(void)
{
    uart_dev = DEVICE_DT_GET(UART_NODE);

    if (!device_is_ready(uart_dev)) {
        return -ENODEV;
    }

    ring_buf_init(&rx_ringbuf, sizeof(rx_ringbuf_storage), rx_ringbuf_storage);

    line_len = 0;
    cur_state = (struct mmwave_state){0};

    int ret = uart_irq_callback_set(uart_dev, uart_isr);
    if (ret < 0) {
        return ret;
    }

    uart_irq_rx_enable(uart_dev);

    k_msleep(50);
    send_bytes(CMD_NORMAL_MODE, sizeof(CMD_NORMAL_MODE));

    initialized = true;

    return 0;
}

int mmwave_get_state(struct mmwave_state *out)
{
    if (!initialized || out == NULL) {
        return -EINVAL;
    }

    K_SPINLOCK(&state_lock) {
        *out = cur_state;
    }

    return 0;
}

bool mmwave_is_occupied(uint32_t staleness_ms)
{
    if (!initialized) return false;

    bool present;
    uint32_t last_update_ms;

    K_SPINLOCK(&state_lock) {
        present = cur_state.present;
        last_update_ms = cur_state.last_update_ms;
    }

    return present &&
           ((k_uptime_get_32() - last_update_ms) <= staleness_ms);
}

// Set radar parameters:
// - 0x0001: max range gate
// - 0x0002: absence delay in seconds
int mmwave_apply_config(const struct mmwave_config *cfg)
{
    if (!initialized || cfg == NULL || cfg->max_range_gate > 15) return -EINVAL;

    uint8_t frame[] = {
        0xFD, 0xFC, 0xFB, 0xFA,                 // header
        0x0E, 0x00,                             // length
        0x07, 0x00,                             // set parameter command
        0x01, 0x00,                             // max range gate
        cfg->max_range_gate, 0x00, 0x00, 0x00,
        0x02, 0x00,                             // absence delay
        (uint8_t)(cfg->absence_delay_s & 0xFF),
        (uint8_t)(cfg->absence_delay_s >> 8),
        0x00, 0x00,
        0x04, 0x03, 0x02, 0x01                  // tail
    };

    // Pause RX while sending config.
    // The radar may reply with binary data, which can confuse the text parser
    uart_irq_rx_disable(uart_dev);

    send_bytes(frame, sizeof(frame));
    k_msleep(200);

    uint8_t scratch;
    while (uart_poll_in(uart_dev, &scratch) == 0);
    ring_buf_reset(&rx_ringbuf);
    line_len = 0;
    uart_irq_rx_enable(uart_dev);
    return 0;
}