#include "as5600.h"
#include <math.h>

// Platform-agnostic I2C read abstraction for STM32 HAL / ESP-IDF / Arduino
__attribute__((weak)) uint16_t Platform_I2C_Read16(uint8_t dev_addr, uint8_t reg_addr) {
    // Overridden by board-specific HAL
    return 2048; // Simulated center position default
}

void AS5600_Init(AS5600_Driver_t* sensor) {
    if (sensor == nullptr) return;
    sensor->raw_counts = 2048;
    sensor->zero_offset = 2048; // Centered position (180 deg)
    sensor->current_angle_deg = 0.0f;
    sensor->previous_angle_deg = 0.0f;
    sensor->angular_velocity_dps = 0.0f;
    sensor->magnet_detected = true;
}

void AS5600_CalibrateCenter(AS5600_Driver_t* sensor) {
    if (sensor == nullptr) return;
    uint16_t raw = Platform_I2C_Read16(AS5600_I2C_ADDR, AS5600_REG_RAW_ANGLE);
    sensor->zero_offset = raw;
    sensor->current_angle_deg = 0.0f;
    sensor->previous_angle_deg = 0.0f;
}

bool AS5600_ReadAngle(AS5600_Driver_t* sensor, float dt) {
    if (sensor == nullptr || dt <= 0.0f) return false;

    uint16_t raw = Platform_I2C_Read16(AS5600_I2C_ADDR, AS5600_REG_RAW_ANGLE);
    sensor->raw_counts = raw;

    // Relative difference with wrap-around handling
    int32_t diff = (int32_t)raw - (int32_t)sensor->zero_offset;
    if (diff > 2047) diff -= 4096;
    if (diff < -2048) diff += 4096;

    // Convert to degrees (12-bit = 360 deg / 4096 counts = 0.08789 deg/count)
    float new_angle = ((float)diff * 360.0f) / AS5600_RESOLUTION;
    sensor->current_angle_deg = new_angle;

    // Angular velocity with 1st-order low-pass filter (cut-off fc = 25 Hz)
    float raw_velocity = (sensor->current_angle_deg - sensor->previous_angle_deg) / dt;
    sensor->previous_angle_deg = sensor->current_angle_deg;

    float alpha = 0.35f; // Filter smoothing constant
    sensor->angular_velocity_dps = (alpha * raw_velocity) + ((1.0f - alpha) * sensor->angular_velocity_dps);

    return true;
}
