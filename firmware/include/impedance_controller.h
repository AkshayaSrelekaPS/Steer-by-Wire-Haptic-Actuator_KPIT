#pragma once
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    // Kinematic Inputs
    float steering_angle_deg;     // Current off-center steering angle θ [-90° to +90°]
    float angular_velocity_dps;    // Angular velocity ω (deg/s)
    float vehicle_speed_kmh;       // Simulated or vehicle CAN speed (km/h)
    float driver_torque_nm;        // Reaction torque sampled from strain gauge AFE (Nm)
    
    // ADAS Inputs
    bool left_obstacle_active;     // Blind-spot / lane boundary hazard on left
    bool right_obstacle_active;    // Guardrail / obstacle on right
    bool rumble_strip_active;      // Lane departure rumble request
    
    // Output Torques
    float centering_torque;        // τ_centering (Nm)
    float damping_torque;          // τ_damping (Nm)
    float barrier_torque;          // τ_barrier (Nm)
    float vibration_torque;        // τ_vibe (Nm)
    float commanded_motor_torque;  // Net clamped motor torque τ_cmd (Nm)
    
    // Safety & Override State (ISO 26262 & UN R79)
    bool is_override_latched;      // True when driver successfully overrides barrier
    uint16_t override_hold_ticks;  // Loop ticks sustained above override threshold
    bool thermal_derating_active;  // Current/torque rollback due to FET overtemp
} HapticActuatorState_t;

// Controller Tuning Parameters
typedef struct {
    float angle_warn_deg;          // θ_warn (default 18.0°)
    float angle_limit_deg;         // θ_limit (default 35.0°)
    float max_torque_clamp_nm;     // τ_clamp (default 5.5 Nm)
    float override_threshold_nm;   // τ_override (default 7.5 - 8.0 Nm)
    uint16_t override_ticks_req;   // e.g. 150 ticks at 1 kHz = 150 ms
} HapticActuatorConfig_t;

void HapticController_Init(HapticActuatorConfig_t* config);
void HapticController_Update(HapticActuatorState_t* state, float dt);
void HapticController_Reset(HapticActuatorState_t* state);

#ifdef __cplusplus
}
#endif
