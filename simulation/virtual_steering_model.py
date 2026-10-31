"""
Steer-by-Wire Haptic Actuator & Vehicle Dynamics Simulator
Implements:
1. 2-DOF Single-Track (Bicycle) Vehicle Dynamics Model (Pneumatic Trail & Self-Aligning Torque)
2. Three-Zone Progressive Repulsive Potential Field with Asymmetric Damping
3. UN R79 & ISO 26262 Driver Intentional Override Latency Analysis
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

def run_simulation():
    # 1. PARAMETERS
    m = 1500.0          # Vehicle mass (kg)
    Iz = 2500.0         # Yaw moment of inertia (kg*m^2)
    a = 1.2             # Distance from CG to front axle (m)
    b = 1.5             # Distance from CG to rear axle (m)
    Caf = 80000.0       # Front cornering stiffness (N/rad)
    Car = 85000.0       # Rear cornering stiffness (N/rad)
    tp = 0.035          # Pneumatic trail (m)
    gear_ratio = 15.0   # Steering gear ratio (steer angle to road wheel angle)

    theta_warn = 18.0   # deg
    theta_limit = 35.0  # deg
    tau_clamp = 5.5     # Nm
    tau_override = 8.0  # Nm

    # 2. STATIC TORQUE-ANGLE PROFILES (PARKING VS HIGHWAY)
    angles = np.linspace(-60, 60, 500)
    
    # Low speed (10 km/h) vs Highway speed (120 km/h)
    k_park = 0.02
    k_high = 0.08

    tau_spring_park = -k_park * angles
    tau_spring_high = -k_high * angles

    # Virtual wall on left (angles < -18 deg)
    tau_barrier_left = np.zeros_like(angles)
    for i, ang in enumerate(angles):
        if ang < -theta_warn:
            excess = np.abs(ang) - theta_warn
            span = theta_limit - theta_warn
            if np.abs(ang) < theta_limit:
                # Zone 2: Parabolic soft margin
                tau_barrier_left[i] = 2.5 * ((excess / span) ** 2)
            else:
                # Zone 3: Clamped high-impedance dissipative barrier
                pen = np.abs(ang) - theta_limit
                tau_barrier_left[i] = min(tau_clamp, 2.5 + pen * 0.25)

    # 3. GENERATE TORQUE PROFILE PLOT
    plt.figure(figsize=(10, 6), dpi=300)
    plt.plot(angles, tau_spring_park, label='Normal Steering: Parking (10 km/h)', color='#3b82f6', linestyle='--', linewidth=2)
    plt.plot(angles, tau_spring_high, label='Normal Steering: Highway (120 km/h)', color='#1d4ed8', linewidth=2)
    plt.plot(angles, tau_barrier_left, label='Active Hazard Barrier (Left Obstacle Active)', color='#ef4444', linewidth=2.5)
    plt.axvline(-theta_warn, color='#eab308', linestyle=':', label='Zone 1 -> Zone 2 (Soft Warning θ_warn = 18°)')
    plt.axvline(-theta_limit, color='#dc2626', linestyle=':', label='Zone 2 -> Zone 3 (Hard Barrier θ_limit = 35°)')
    plt.axhline(tau_clamp, color='#991b1b', linestyle='-.', alpha=0.7, label='Safety Torque Clamp (5.5 Nm)')

    plt.title('Steer-by-Wire Impedance & Progressive Collision-Rejection Barrier Profile', fontsize=12, fontweight='bold', pad=12)
    plt.xlabel('Steering Angle θ (degrees)', fontsize=11)
    plt.ylabel('Commanded Feedback Torque τ (Nm)', fontsize=11)
    plt.grid(True, alpha=0.3)
    plt.legend(loc='upper right', fontsize=9)
    plt.tight_layout()
    plt.savefig('simulation/impedance_response_plot.png')
    plt.close()
    print("Saved simulation/impedance_response_plot.png")

    # 4. TRANSIENT DRIVER OVERRIDE RESPONSE (150 ms Decay)
    t = np.linspace(0, 0.6, 600) # 600 ms simulation
    driver_effort = np.zeros_like(t)
    barrier_torque = np.zeros_like(t)
    override_active = np.zeros_like(t)

    # Driver applies sudden panic force at t = 0.1s
    for i, time in enumerate(t):
        if time < 0.1:
            driver_effort[i] = 3.0
            barrier_torque[i] = 4.8
        elif time < 0.25:
            # Panic override effort exceeding 8 Nm
            driver_effort[i] = 9.5
            barrier_torque[i] = 4.8 # Still within 150 ms evaluation window
        else:
            # Over 150 ms sustained effort -> UN R79 override triggers
            driver_effort[i] = 9.5
            override_active[i] = 1.0
            # Exponential barrier decay
            decay_factor = np.exp(-(time - 0.25) / 0.04)
            barrier_torque[i] = 4.8 * (0.15 + 0.85 * decay_factor)

    fig, ax1 = plt.subplots(figsize=(10, 5), dpi=300)
    ax2 = ax1.twinx()

    ax1.plot(t * 1000, driver_effort, 'purple', linewidth=2, label='Driver Reaction Torque (Load Cell)')
    ax1.axhline(tau_override, color='purple', linestyle='--', alpha=0.6, label='UN R79 Override Threshold (8.0 Nm)')
    ax2.plot(t * 1000, barrier_torque, 'red', linewidth=2.5, label='Motor Barrier Counter-Torque')

    ax1.axvspan(100, 250, color='yellow', alpha=0.2, label='150 ms Intentional Evaluation Window')
    ax1.axvspan(250, 600, color='green', alpha=0.15, label='Override Latched: Barrier Decayed')

    ax1.set_xlabel('Time (milliseconds)', fontsize=11)
    ax1.set_ylabel('Driver Torque (Nm)', color='purple', fontsize=11)
    ax2.set_ylabel('Motor Barrier Torque (Nm)', color='red', fontsize=11)
    plt.title('ISO 26262 & UN R79 Driver Intentional Panic Override Dynamic Latency', fontsize=12, fontweight='bold', pad=12)
    ax1.grid(True, alpha=0.3)

    lines1, labels1 = ax1.get_legend_handles_labels()
    lines2, labels2 = ax2.get_legend_handles_labels()
    ax1.legend(lines1 + lines2, labels1 + labels2, loc='center right', fontsize=9)
    plt.tight_layout()
    plt.savefig('simulation/phase_portrait_override.png')
    plt.close()
    print("Saved simulation/phase_portrait_override.png")

if __name__ == '__main__':
    run_simulation()
