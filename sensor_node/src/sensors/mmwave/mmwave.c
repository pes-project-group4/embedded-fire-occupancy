#include "mmwave.h"
#include "uart.h"

const unsigned char header[4] = {0xfd, 0xfc, 0xfb, 0xfa};
const unsigned char tail[4] = {0x4, 0x3, 0x2, 0x1};

char* firmware_version(char* response)
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
        response[i] = recv_byte();
    }
    return response;
}

char* serial_number(char* response)
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
        response[i] = recv_byte();
    }
    return response;
}

void send_bytes(const char* const bytes, int length)
{
    for(int i = 0; i < length; i++)
    {
        send_byte(bytes[i]);
    }
}
