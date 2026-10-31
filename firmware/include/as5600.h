#pragma once
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define AS5600_I2C_ADDR       0x36
#define AS5600_REG_RAW_ANGLE  0x0C
#define AS5600_REG_STATUS     0x0B
#define AS5600_RESOLUTION     4096.0f

typedef struct {
    uint16_t raw_counts;
    uint16_t zero_offset;
    float current_angle_deg;
    float previous_angle_deg;
    float angular_velocity_dps;
    bool magnet_detected;
} AS5600_Driver_t;

void AS5600_Init(AS5600_Driver_t* sensor);
void AS5600_CalibrateCenter(AS5600_Driver_t* sensor);
bool AS5600_ReadAngle(AS5600_Driver_t* sensor, float dt);

#ifdef __cplusplus
}
#endif
