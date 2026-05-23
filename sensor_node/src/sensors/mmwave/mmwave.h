#ifndef _MMWAVE_H_
#define _MMWAVE_H_

#include <stdbool.h>
#include <stdint.h>

/*
 HMMD mmWave radar over UART1.
 UART setting: 115200 8N1
 Radar output:
 - "ON/OFF"
 - "Range NNN" in cm
*/

struct mmwave_state {
    bool present;
    uint16_t range_cm;
    uint32_t last_update_ms;
};

struct mmwave_config {
    uint8_t max_range_gate;      // 0..15, each gate is 70 cm
    uint16_t absence_delay_s;    // Delay before changing from ON to OFF
};

int mmwave_init(void);
int mmwave_get_state(struct mmwave_state *out);
bool mmwave_is_occupied(uint32_t staleness_ms);
int mmwave_apply_config(const struct mmwave_config *cfg);

#endif