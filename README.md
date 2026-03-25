# Zephyr Sensor Node - Group 4
**base_station** – Zephyr application acting as the interface/driver node
**sensor_node** – Firmware handling connected sensors

## Requirements

* Zephyr RTOS (west workspace initialized)
* Zephyr SDK installed
* Python 3.12+

## Build Instructions

Build each application separately:

### Base Station

```bash
west build -b rpi_pico2/rp2350a/m33 -s base_station -d build/base_station
```

### Sensor Node

```bash
west build -b rpi_pico2/rp2350a/m33 -s sensor_node -d build/sensor_node
```

## Output

Firmware files will be generated in:

* `build/base_station/zephyr/`
* `build/sensor_node/zephyr/`

Look for:

* `zephyr.uf2` (for flashing)