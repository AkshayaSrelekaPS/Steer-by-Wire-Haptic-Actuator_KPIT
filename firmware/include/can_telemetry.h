#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "impedance_controller.h"

#ifdef __cplusplus
extern "C" {
#endif

// Automotive CAN Arbitration IDs
#define CAN_ID_STEER_STATUS_THETA   0x102 // 20 ms cyclic
#define CAN_ID_MOTOR_TORQUE_CMD     0x104 // 20 ms cyclic
#define CAN_ID_ADAS_HAZARD_FLAG     0x220 // 50 ms cyclic
#define CAN_ID_SAFETY_OVERRIDE      0x350 // 100 ms cyclic / Event-triggered

typedef struct {
    uint32_t id;
    uint8_t dlc;
    uint8_t data[8];
} CAN_Frame_t;

void CAN_Telemetry_Init(void);
void CAN_Telemetry_BuildSteerFrame(const HapticActuatorState_t* state, CAN_Frame_t* frame);
void CAN_Telemetry_BuildTorqueFrame(const HapticActuatorState_t* state, float motor_current_amps, CAN_Frame_t* frame);
void CAN_Telemetry_BuildSafetyFrame(const HapticActuatorState_t* state, CAN_Frame_t* frame);

#ifdef __cplusplus
}
#endif
