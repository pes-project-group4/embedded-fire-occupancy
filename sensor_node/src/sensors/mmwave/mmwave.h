#ifndef MMWAVE_H
#define MMWAVE_H

#define HEADER_LENGTH 4
#define TAIL_LENGTH 4

char* firmware_version(char* response);
char* serial_number(char* response);
void send_bytes(const char* const bytes, int length);

#endif
