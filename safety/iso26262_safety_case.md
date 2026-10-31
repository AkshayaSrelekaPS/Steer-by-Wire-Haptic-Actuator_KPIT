# ISO 26262 Functional Safety Concept & ASIL-D Safety Case

## 1. Item Definition

The **Steer-by-Wire (SbW) Force-Feedback Haptic Actuator** replaces the traditional mechanical steering column with a direct cyber-physical drive-by-wire system. The actuator functions as an **electronic human-machine interface (HMI)** that:
1. Synthesizes artificial road-surface impedance (self-centering spring, viscous damping, road texture).
2. Generates active collision-rejection counter-torque (progressive virtual barrier) before crossing hazardous lane boundaries.
3. Detects and yields to intentional human driver overrides under emergency evasive maneuvers.

---

## 2. Hazard Analysis and Risk Assessment (HARA)

In accordance with **ISO 26262-3:2018 (Clause 6)**, automotive operational situations were evaluated across three risk parameters:
- **Severity (S):** S0 (No injuries) to S3 (Life-threatening / fatal injuries)
- **Exposure (E):** E0 (Incredible) to E4 (High probability / common driving scenario)
- **Controllability (C):** C0 (Controllable in general) to C3 (Difficult / uncontrollable)

### HARA Evaluation Matrix:

| Hazard ID | Operating Scenario | Hazardous Event | Severity (S) | Exposure (E) | Controllability (C) | Resulting ASIL | Safety Goal (SG) |
| :---: | :--- | :--- | :---: | :---: | :---: | :---: | :--- |
| **HZ-01** | Highway cruising ($120\text{ km/h}$) | **Inadvertent Actuator Lockup / Unintended Full Counter-Torque:** Sensor fault triggers false barrier, fighting driver during split-second avoidance. | **S3** | **E4** | **C3** | **ASIL-D** | **SG-1:** The actuator shall never lock or apply sustained counter-torque $>5.5\text{ Nm}$, and must yield full authority to the human driver within $150\text{ ms}$ upon sensing panic counter-torque $>8.0\text{ Nm}$. |
| **HZ-02** | High-speed lane departure | **Total Loss of Haptic Centering Resistance:** Actuator fails open (zero torque), leaving steering freely spinning without self-centering. | **S3** | **E3** | **C2** | **ASIL-B** | **SG-2:** Upon power inverter or MCU failure, the system shall broadcast a CAN emergency fail-silent state and allow free manual rotation without mechanical jamming. |
| **HZ-03** | Low-speed urban / parking ($10\text{ km/h}$) | **Unintended Torque Step / Oscillatory Limit Cycle:** High controller gain creates steering wheel wobble/oscillation. | **S1** | **E4** | **C1** | **ASIL-A** | **SG-3:** Closed-loop impedance damping shall adaptively stabilize velocity chattering with phase margin $\phi_m \ge 60^\circ$. |
| **HZ-04** | Continuous hazard engagement | **H-Bridge Power Stage Thermal Runaway:** MOSFET gate driver latch-up under continuous stall current. | **S2** | **E3** | **C1** | **ASIL-B** | **SG-4:** Continuous motor current shall be hardware-clamped to $4.0\text{ A}$, with automatic 40% thermal derating when FET temperature exceeds $85^\circ\text{C}$. |

---

## 3. Technical Safety Requirements (TSRs)

```
                                    +-----------------------------------------+
                                    |     SG-1: HUMAN DRIVER OVERRIDE         |
                                    |       (ISO 26262 ASIL-D / UN R79)       |
                                    +--------------------+--------------------+
                                                         |
                         +-------------------------------+-------------------------------+
                         v                                                               v
      +-------------------------------------+                         +-------------------------------------+
      |   TSR-1: DUAL SENSOR REDUNDANCY     |                         |    TSR-2: CURRENT & TORQUE CLAMP    |
      | - AS5600 12-bit angle encoder (I2C) |                         | - Shunt amplifier (INA240) clamped   |
      | - Strain Gauge Load Cell (Wheatstone|                         |   at 4.5A (5.5 Nm maximum output)   |
      |   Bridge + INA128 instrumentation)  |                         | - Hardware crowbar disconnect FET   |
      +-------------------------------------+                         +-------------------------------------+
```

1. **TSR-1 (Plausibility Cross-Check):** If the angular velocity ($\omega$) calculated from the AS5600 encoder opposes the motor torque direction, the controller performs a consistency verification against the analog reaction torque sensor to verify human intent.
2. **TSR-2 (Ergonomic Torque Ceiling):** The maximum motor command is clamped at $\tau_{\text{clamp}} = 5.5\text{ Nm}$ via software ceiling and hardware analog current limiter, ensuring the motor can never physically overpower an adult human driver ($12\text{–}15\text{ Nm}$ capability).
3. **TSR-3 (150 ms Decay Window):** In accordance with **UN R79 and ISO 11270**, if the driver exerts $>8.0\text{ Nm}$ opposing the barrier for $\ge 150\text{ ms}$, the barrier counter-torque decays exponentially ($\tau = 40\text{ ms}$ time constant) to yield authority.
4. **TSR-4 (Fail-Silent Safe State):** In the event of an MCU brownout, watchdog timeout, or gate driver fault, the H-bridge gate inputs are immediately pulled LOW, converting the actuator into a freewheeling compliant column.
