#ifndef MMWAVE_H
#define MMWAVE_H

#define HEADER_LENGTH 4
#define TAIL_LENGTH 4

void set_normal_mode(char* response);
void firmware_version(char* response);
void serial_number(char* response);
void send_bytes(const char* const bytes, int length);

#endif
