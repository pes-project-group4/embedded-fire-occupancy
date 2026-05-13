#include "mmwave.h"
#include "uart.h"

char* firmware_version()
{
    const char* const length = {0x2, 0x0};
    const char* const command = {0x0, 0x0};

    send_bytes(header, HEADER_LENGTH);
    send_bytes(length, 2);
    send_bytes(command, 2);
    send_bytes(tail, TAIL_LENGTH);
}

char* serial_number()
{
    const char* const length = {0x2, 0x0};
    const char* const command = {0x11, 0x0};

    send_bytes(header, HEADER_LENGTH);
    send_bytes(length, 2);
    send_bytes(command, 2);
    send_bytes(tail, TAIL_LENGTH);
}

void send_bytes(const char* const bytes, int length)
{
    for(int i = 0; i < length; i++)
    {
        send_byte(bytes[i]);
    }
}
