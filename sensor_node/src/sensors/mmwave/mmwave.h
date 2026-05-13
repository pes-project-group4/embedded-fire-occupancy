#ifndef MMWAVE_H
#define MMWAVE_H

#define HEADER_LENGTH 4
#define TAIL_LENGTH 4

const char* const header = {0xfd, 0xfc, 0xfb, 0xfa};
const char* const tail = {0x4, 0x3, 0x2, 0x1};

char* firmware_version();
char* serial_number();
void send_bytes(const char* const bytes, int length);

#endif
