# Zephyr Sensor Node - Group 4

## Apps
- **base_station** – Acting as the interface/driver node
- **sensor_node** – Firmware handling connected sensors (Not allowed to use Sensors API)

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

## CI/CD

This repo has GitHub Actions set up to automatically build both apps on every push and pull request. You can check the build status on the Actions tab or by the ✅/❌ icon on each commit/PR.

To create a release, go to **GitHub → Releases → Draft a new release**, enter a version tag (ex: `v1.0.0`), and publish. The release workflow will automatically build both apps and attach the firmware files (`.uf2`, `.elf`, `.hex`, `.bin`) to the release.

## Workflow

1. **Create a branch** for your feature (ex: `feature/add-temperature-sensor`)
2. **Develop and push** your changes to that branch
3. **Check CI** — make sure both base_station and sensor_node build successfully (green ✅)
4. **Open a Pull Request** to `main` only after all builds pass
5. **Wait for review** — at least one team member should review before merging
6. **Do not merge with failing builds**
7. **To release** — after merging, create a release from the GitHub UI with a version tag