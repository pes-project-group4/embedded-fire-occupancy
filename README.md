# Embedded Fire and Occupancy Monitoring System - Group 4

## Apps
* `base_station`: reads the sensor node through a Zephyr Sensor API driver, shows readings on the console, handles configuration mode, and drives alarm outputs.
* `sensor_node`: reads BME680, MLX90614, HMMD mmWave, and SPW2430 sensors and exposes a register map over I2C target mode.

## Requirements

* Zephyr RTOS west workspace initialized from `west.yml`
* Zephyr SDK with the `arm-zephyr-eabi` toolchain
* Python 3.12 or newer

## Build

Build each application separately from the repository root:

```bash
west build -b rpi_pico2/rp2350a/m33 -s base_station -d build/base_station -p auto
```

```bash
west build -b rpi_pico2/rp2350a/m33 -s sensor_node -d build/sensor_node -p auto
```

Firmware artifacts are generated under:

* `build/base_station/zephyr/`
* `build/sensor_node/zephyr/`

Flash the generated `zephyr.uf2` file for each application to the matching Pico 2 board.

## CI and Releases

GitHub Actions builds both applications on every push and pull request. Before opening a pull request, make sure both CI jobs pass.

To create a release, open GitHub Releases, draft a new release with a version tag such as `v1.0.0`, and publish it. The release workflow builds both applications and attaches `.uf2`, `.elf`, `.hex`, and `.bin` firmware artifacts.

## Review Workflow

1. Create a feature branch, for example `feature/add-temperature-sensor`.
2. Build both applications locally.
3. Push the branch and confirm the CI builds pass.
4. Open a pull request to `main`.
5. Wait for at least one team member review before merging.
6. Do not merge with failing builds.
