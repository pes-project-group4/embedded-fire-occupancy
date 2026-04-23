cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(sensor_node)

target_sources(app PRIVATE 
    src/main.c
    src/sensors/bme680/bme680.c
    src/sensors/mlx90614/mlx90614.c
    src/sensors/mmwave/mmwave.c
    src/sensors/spw2430/spw2430.c
)
target_include_directories(app PRIVATE src/inc)
