cmake_minimum_required(VERSION 3.16)
project(SteerByWire_HapticActuator C CXX)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

include_directories(
    include
)

set(SOURCES
    src/main.cpp
    src/impedance_controller.cpp
    src/can_telemetry.cpp
    src/sensor_drivers/as5600.cpp
)

add_executable(sbw_actuator_firmware ${SOURCES})

target_compile_options(sbw_actuator_firmware PRIVATE -Wall -Wextra -O2)
