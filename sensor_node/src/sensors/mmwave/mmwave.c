#include <zephyr/kernel.h>
#include "mmwave.h"
#include "uart.h"

static const unsigned char tail[4] = {0x4, 0x3, 0x2, 0x1};
static const unsigned char header[4] = {0xfd, 0xfc, 0xfb, 0xfa};
static const struct device *uart1 = DEVICE_DT_GET(DT_NODELABEL(uart1));

void set_normal_mode(char* response)
{
    recv_byte(uart1);
    const int response_length = 18;
    const char length[2] = {0x8, 0x0};
    const char command[8] = {0x12, 0x0, 0x0, 0x0, 0x64, 0x0, 0x0, 0x0};

    send_bytes(header, HEADER_LENGTH);
    send_bytes(length, 2);
    send_bytes(command, 8);
    send_bytes(tail, TAIL_LENGTH);

    for(int i = 0; i < response_length; i++)
    {
        response[i] = recv_byte(uart1);
    }
}

void firmware_version(char* response)
{
    const int response_length = 24;
    const char length[2] = {0x2, 0x0};
    const char command[2] = {0x0, 0x0};

    send_bytes(header, HEADER_LENGTH);
    send_bytes(length, 2);
    send_bytes(command, 2);
    send_bytes(tail, TAIL_LENGTH);

    for(int i = 0; i < response_length; i++)
    {
        response[i] = recv_byte(uart1);
    }
}

void serial_number(char* response)
{
    const int response_length = 18;
    const char length[2] = {0x2, 0x0};
    const char command[2] = {0x11, 0x0};

    send_bytes(header, HEADER_LENGTH);
    send_bytes(length, 2);
    send_bytes(command, 2);
    send_bytes(tail, TAIL_LENGTH);
    
    for(int i = 0; i < response_length; i++)
    {
        response[i] = recv_byte(uart1);
    }
}

void send_bytes(const char* const bytes, int length)
{
    for(int i = 0; i < length; i++)
    {
        send_byte(uart1, bytes[i]);
        printk("Sending: %x\n", bytes[i]);
    }
}
