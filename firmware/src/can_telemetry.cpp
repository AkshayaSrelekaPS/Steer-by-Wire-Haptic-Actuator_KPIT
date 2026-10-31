#include "can_telemetry.h"

void CAN_Telemetry_Init(void) {
    // Hardware CAN peripheral init (e.g., bxCAN / MCP2515 / TWAI)
}

void CAN_Telemetry_BuildSteerFrame(const HapticActuatorState_t* state, CAN_Frame_t* frame) {
    if (!state || !frame) return;

    frame->id = CAN_ID_STEER_STATUS_THETA;
    frame->dlc = 8;

    // Angle: scale factor 0.01 deg -> int16 [-90.00° = -9000]
    int16_t angle_scaled = (int16_t)(state->steering_angle_deg * 100.0f);
    // Angular Velocity: scale factor 0.1 deg/s -> int16
    int16_t vel_scaled = (int16_t)(state->angular_velocity_dps * 10.0f);
    // Driver Reaction Torque: scale factor 0.01 Nm -> int16
    int16_t torque_scaled = (int16_t)(state->driver_torque_nm * 100.0f);

    frame->data[0] = (uint8_t)(angle_scaled >> 8);
    frame->data[1] = (uint8_t)(angle_scaled & 0xFF);
    frame->data[2] = (uint8_t)(vel_scaled >> 8);
    frame->data[3] = (uint8_t)(vel_scaled & 0xFF);
    frame->data[4] = (uint8_t)(torque_scaled >> 8);
    frame->data[5] = (uint8_t)(torque_scaled & 0xFF);
    frame->data[6] = state->is_override_latched ? 0x01 : 0x00;
    frame->data[7] = 0xAA; // Rolling alive counter / checksum
}

void CAN_Telemetry_BuildTorqueFrame(const HapticActuatorState_t* state, float motor_current_amps, CAN_Frame_t* frame) {
    if (!state || !frame) return;

    frame->id = CAN_ID_MOTOR_TORQUE_CMD;
    frame->dlc = 8;

    int16_t cmd_torque_scaled = (int16_t)(state->commanded_motor_torque * 100.0f);
    uint16_t current_ma = (uint16_t)(motor_current_amps * 1000.0f);
    int16_t barrier_scaled = (int16_t)(state->barrier_torque * 100.0f);

    frame->data[0] = (uint8_t)(cmd_torque_scaled >> 8);
    frame->data[1] = (uint8_t)(cmd_torque_scaled & 0xFF);
    frame->data[2] = (uint8_t)(current_ma >> 8);
    frame->data[3] = (uint8_t)(current_ma & 0xFF);
    frame->data[4] = (uint8_t)(barrier_scaled >> 8);
    frame->data[5] = (uint8_t)(barrier_scaled & 0xFF);
    frame->data[6] = state->thermal_derating_active ? 0x02 : 0x00;
    frame->data[7] = 0x55;
}

void CAN_Telemetry_BuildSafetyFrame(const HapticActuatorState_t* state, CAN_Frame_t* frame) {
    if (!state || !frame) return;

    frame->id = CAN_ID_SAFETY_OVERRIDE;
    frame->dlc = 8;

    uint8_t flags = 0;
    if (state->is_override_latched) flags |= 0x01;
    if (state->left_obstacle_active) flags |= 0x02;
    if (state->right_obstacle_active) flags |= 0x04;
    if (state->rumble_strip_active) flags |= 0x08;

    frame->data[0] = flags;
    frame->data[1] = (uint8_t)(state->override_hold_ticks & 0xFF);
    frame->data[2] = (uint8_t)(state->override_hold_ticks >> 8);
    frame->data[3] = 0xD0; // ASIL-D heartbeat
    frame->data[4] = 0x00;
    frame->data[5] = 0x00;
    frame->data[6] = 0x00;
    frame->data[7] = 0xFE;
}
