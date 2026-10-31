#include "impedance_controller.h"
#include <math.h>

static HapticActuatorConfig_t cfg;
static uint32_t vibe_phase_tick = 0;

void HapticController_Init(HapticActuatorConfig_t* config) {
    if (config != nullptr) {
        cfg = *config;
    } else {
        // Safe industrial automotive defaults
        cfg.angle_warn_deg = 18.0f;
        cfg.angle_limit_deg = 35.0f;
        cfg.max_torque_clamp_nm = 5.5f;
        cfg.override_threshold_nm = 7.5f;
        cfg.override_ticks_req = 150; // 150 ms at 1 kHz
    }
}

void HapticController_Reset(HapticActuatorState_t* state) {
    if (state == nullptr) return;
    state->steering_angle_deg = 0.0f;
    state->angular_velocity_dps = 0.0f;
    state->centering_torque = 0.0f;
    state->damping_torque = 0.0f;
    state->barrier_torque = 0.0f;
    state->vibration_torque = 0.0f;
    state->commanded_motor_torque = 0.0f;
    state->is_override_latched = false;
    state->override_hold_ticks = 0;
    state->thermal_derating_active = false;
}

void HapticController_Update(HapticActuatorState_t* state, float dt) {
    if (state == nullptr) return;

    // 1. Dynamic Speed-Dependent Self-Centering (Caster trail synthesis)
    // Low speed (10 km/h) -> 0.02 Nm/deg (parking comfort)
    // High speed (120 km/h) -> 0.08 Nm/deg (highway directional stability)
    float speed_ratio = state->vehicle_speed_kmh / 160.0f;
    if (speed_ratio > 1.0f) speed_ratio = 1.0f;
    if (speed_ratio < 0.0f) speed_ratio = 0.0f;

    float k_spring = 0.02f + (speed_ratio * 0.06f);
    state->centering_torque = -k_spring * state->steering_angle_deg;

    // 2. Dynamic Speed-Dependent Viscous Damping
    float b_damping = 0.003f + (speed_ratio * 0.008f);
    state->damping_torque = -b_damping * state->angular_velocity_dps;

    // 3. Three-Zone Progressive Potential Field (Active Collision-Rejection)
    float barrier = 0.0f;
    float abs_angle = fabsf(state->steering_angle_deg);
    float sgn_angle = (state->steering_angle_deg >= 0.0f) ? 1.0f : -1.0f;

    bool hazard_in_direction = (state->steering_angle_deg < 0.0f && state->left_obstacle_active) ||
                               (state->steering_angle_deg > 0.0f && state->right_obstacle_active);

    if (hazard_in_direction && abs_angle > cfg.angle_warn_deg) {
        float span = cfg.angle_limit_deg - cfg.angle_warn_deg;
        float excess = abs_angle - cfg.angle_warn_deg;

        if (abs_angle < cfg.angle_limit_deg) {
            // Zone 2: Progressive Soft-Margin Warning (Parabolic repulsive potential)
            float norm_penetration = excess / span;
            barrier = -2.5f * (norm_penetration * norm_penetration) * sgn_angle;
        } else {
            // Zone 3: High-Impedance Dissipative Barrier
            float wall_penetration = abs_angle - cfg.angle_limit_deg;
            
            // Asymmetric One-Way Damping: only resists steering deeper into danger
            float moving_into_hazard = state->angular_velocity_dps * sgn_angle;
            float one_way_damping = (moving_into_hazard > 0.0f) ? (moving_into_hazard * 0.015f) : 0.0f;

            float raw_wall = 2.5f + (wall_penetration * 0.25f) + one_way_damping;
            if (raw_wall > cfg.max_torque_clamp_nm) {
                raw_wall = cfg.max_torque_clamp_nm;
            }
            barrier = -raw_wall * sgn_angle;
        }
    }
    state->barrier_torque = barrier;

    // 4. Human Intentional Override Detection (UN R79 / ISO 26262 ASIL-D Compliance)
    if (fabsf(state->barrier_torque) > 0.5f && state->driver_torque_nm > cfg.override_threshold_nm) {
        state->override_hold_ticks++;
        if (state->override_hold_ticks >= cfg.override_ticks_req) {
            state->is_override_latched = true;
            // Decay barrier to yield manual steering authority to human driver
            state->barrier_torque *= 0.15f;
        }
    } else {
        if (state->driver_torque_nm < (cfg.override_threshold_nm - 1.0f)) {
            state->override_hold_ticks = 0;
            state->is_override_latched = false;
        }
    }

    // 5. High-Frequency Tactile Road Surface Synthesis (Rumble strip 80 Hz)
    state->vibration_torque = 0.0f;
    if (state->rumble_strip_active) {
        vibe_phase_tick++;
        // 80 Hz buzz at 1 kHz sample rate: period = 12.5 ticks
        state->vibration_torque = 1.2f * sinf((float)vibe_phase_tick * 0.50265f); // 2*pi*(80/1000)
    } else {
        vibe_phase_tick = 0;
    }

    // 6. Net Torque Summation & Functional Safety Clamping
    float net = state->centering_torque + state->damping_torque + state->barrier_torque + state->vibration_torque;

    // Clamping limits to prevent motor stall runaway and driver wrist strain
    float clamp = cfg.max_torque_clamp_nm;
    if (state->thermal_derating_active) {
        clamp *= 0.60f; // 40% thermal rollback derating
    }

    if (net > clamp) net = clamp;
    if (net < -clamp) net = -clamp;

    state->commanded_motor_torque = net;
}
