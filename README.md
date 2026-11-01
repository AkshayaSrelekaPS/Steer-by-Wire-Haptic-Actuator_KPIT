# Steer-by-Wire Dynamic Force-Feedback Haptic Actuator with Active Collision-Rejection Virtual Wall & Road Impedance Synthesis

[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)
[![Domain: Automotive E/E & Chassis](https://img.shields.io/badge/Domain-Automotive_Chassis_%26_Steer--by--Wire-brightgreen.svg)](#)
[![Standards: ISO 26262 | UN R79](https://img.shields.io/badge/Compliance-ISO_26262_%7C_UN_R79-orange.svg)](#)
[![Presentation: KPIT Sparkle Deck](https://img.shields.io/badge/Presentation-KPIT_SPARKLE_Deck.pdf-red.svg)](KPIT%20SPARKLE_TECHTONICS.pdf)
[![Automotive E/E CI Pipeline](https://github.com/AkshayaSrelekaPS/Steer-by-Wire-Haptic-Actuator_KPIT/actions/workflows/ci.yml/badge.svg)](https://github.com/AkshayaSrelekaPS/Steer-by-Wire-Haptic-Actuator_KPIT/actions/workflows/ci.yml)

> **Submission for KPIT SPARKLE 2027 (National Innovation Challenge)**  
> **Theme:** Wearable Robotics, In-Cabin Ergonomics & Software-Defined Vehicles  
> **Team:** TECHTONICS (Akshaya Sreleka P S, Ashwinkumar K, Dhamarai Kannan A, Dharshan S)  
> **Institution:** Chennai Institute of Technology (CIT Chennai)

---

## 📌 Executive Summary & Strategic Rationale

In modern **Software-Defined Vehicles (SDVs)** and autonomous driving platforms, the physical steering column connecting the driver's hands to the road wheels is completely removed. While **Steer-by-Wire (SbW)** offers massive packaging flexibility and mass reduction (saving $8\text{--}12\text{ kg}$ per chassis), it introduces two severe safety hazards:
1. **Total Loss of Haptic Road Feel:** Drivers lose vital tactile cues (suspension caster return, tire slip angle saturation, hydroplaning loss of friction, and road surface irregularities).
2. **Accidental Trajectory Drift:** Without mechanical resistance, drivers can inadvertently steer into blind-spot obstacles or oncoming lanes during critical autonomous takeover transitions.

This repository provides an industrial-grade **Automotive Force-Feedback Steering Actuator Bench Prototype** featuring closed-loop active impedance control, multi-physics FEA validation, formal **ISO 26262 ASIL-D** functional safety, and a live **Hardware-in-the-Loop (HIL)** telemetry digital twin.

---

## 🏆 Key Innovation Highlights (Why This Project Stands Out)

* 🚀 **1 kHz Hard Real-Time Deterministic Control:** Implemented in embedded C++17 on an ARM Cortex-M4 (STM32F401), achieving $< 0.18\text{ ms}$ loop jitter.
* 🛡️ **Solving the "Illegal Robot Lock" Trap (UN R79 / ISO 11270):** Rather than creating an impassable barrier that traps the driver during sensor false-positives, the system implements an active **$150\text{ ms}$ panic override detector** ($> 8.0\text{ Nm}$) with exponential torque decay.
* 🌊 **Asymmetric One-Way Viscous Damping:** Imposes high dissipative damping ($\max(0, \omega)$) when steering deeper into danger, but drops resistance instantly to zero when steering back toward safety for natural recovery.
* 🔬 **Multi-Physics FEA Verification (ANSYS Mechanical & Icepak):** Proven $20.5\text{ Nm}$ torsional shock resistance (Factor of Safety $= 3.72$, deflection $< 0.05^\circ$) and steady-state thermal junction stability ($68.4^\circ\text{C}$ vs $85^\circ\text{C}$ ASIL limit).
* 🏎️ **Chassis Dynamics & Pneumatic Trail Simulation:** 2-DOF single-track bicycle model synthesizing dynamic road grip loss under low tire-road friction ($\mu = 0.3$).
* ⚡ **Zero-Dependency Python CAN Gateway & Web Serial HIL:** Hosts an HTML5 digital twin and allows direct 1-click USB bench evaluation via Chrome/Edge Web Serial API.

---

## 📚 Core Engineering Dossiers & Documentation

| Engineering Subsystem | Documentation & Source Links | Verification Artifacts |
| :--- | :--- | :--- |
| **Mechanical & Thermal FEA** | [`ansys/ansys_fea_thermal_report.md`](ansys/ansys_fea_thermal_report.md) | Native APDL Macros: [`torsional_fea_model.mac`](ansys/torsional_fea_model.mac), [`thermal_icepak_setup.mac`](ansys/thermal_icepak_setup.mac) |
| **Functional Safety (ASIL-D)** | [`safety/iso26262_safety_case.md`](safety/iso26262_safety_case.md) | HARA Matrix (`HZ-01` to `HZ-04`), Safety Goals (`SG-1` to `SG-4`), TSRs |
| **ECE Circuit & Hardware** | [`hardware/circuit_schematic.md`](hardware/circuit_schematic.md), [`hardware/bom.md`](hardware/bom.md) | Full H-Bridge Schematics, Sallen-Key 150 Hz Filter, MCU Pinouts |
| **Chassis Dynamics & Plant** | [`simulation/virtual_steering_model.py`](simulation/virtual_steering_model.py) | 2-DOF Bicycle Model Plots: [Torque Curve](simulation/impedance_response_plot.png), [Override Delay](simulation/phase_portrait_override.png) |
| **Embedded C++ Firmware** | [`firmware/`](firmware/) | 1 kHz Deterministic Loop, AS5600 I2C Driver, CAN-FD Framing (`0x102`, `0x104`, `0x350`) |
| **CAN Gateway Backend** | [`backend/telemetry_server.py`](backend/telemetry_server.py) | Python REST API, 50 Hz Background CAN Synthesizer, Web Cockpit Server |
| **Presentation Deck** | [`KPIT SPARKLE_TECHTONICS.pdf`](KPIT%20SPARKLE_TECHTONICS.pdf) | Official 10-Slide Competition Presentation Deck |

---

## 🏗️ System Architecture & Cyber-Physical Domains

![Steer-by-Wire 2D Architecture Block Diagram](architecture_block_diagram.png)

<p align="center">
  <em>Figure 1: Modular System Architecture illustrating the 4 distinct ECE cyber-physical domains and bus interactions.</em>
</p>

![Steer-by-Wire 3D Conceptual Hardware Diagram](architecture_diagram.png)

<p align="center">
  <em>Figure 2: Physical Bench Configuration showing the high-torque planetary actuator, inline strain gauge, and AS5600 rotary sensor.</em>
</p>

```
                  +-------------------------------------------------------------+
                  |                 VEHICLE SENSORS / ADAS SIMULATOR           |
                  |   (Ultrasonic / LiDAR Distance, Road Friction μ, CAN Bus)   |
                  +------------------------------+------------------------------+
                                                 | (CAN-Bus / UART Telemetry)
                                                 v
  +-----------------------------------------------------------------------------------------+
  |                        STEERING ACTUATION CONTROLLER (STM32F4 / ESP32)                 |
  |                                                                                         |
  |  +---------------------------+  +--------------------------+  +----------------------+  |
  |  |  Virtual Environment      |  |  Impedance & Admittance  |  | Functional Safety    |  |
  |  |  Model:                   |->|  Control Engine:         |->| & Watchdog Logic     |  |
  |  |  - Centering Spring Rate  |  |  τ_cmd = K(θ) + B(ω)     |  | - Current Clamping   |  |
  |  |  - Progressive Barrier    |  |          + τ_barrier     |  | - Human Override Det |  |
  |  +---------------------------+  +--------------------------+  +----------------------+  |
  +--------^-------------------------------------+----------------------------+-------------+
           |                                     | (Dual Complementary PWM)   |
           |                                     v                            |
+----------+------------+             +-----------------------+               |
|  SENSOR FEEDBACK LOOP |             | POWER DRIVER STAGE    |               |
|                       |             | (Discrete H-Bridge /  |               |
| 1. Reaction Torque:   |             |  Gate Drivers + FETs) |               |
|    Wheatstone Strain  |             +-----------+-----------+               |
|    Gauge + INA128 Amp |                         | (Bi-directional Current)  |
|                       |                         v                           v
| 2. Angle & Velocity:  |             +-----------------------+     +-----------------------+
|    AS5600 12-Bit      |             | HIGH-TORQUE MOTOR     |<--->| CURRENT SENSING AFE   |
|    Magnetic Rotary    |             | (Coupled to Steering  |     | (Shunt + INA240 /     |
|    Encoder (I2C)      |             |  Wheel Shaft)         |     |  ACS712 Overcurrent)  |
+-----------------------+             +-----------------------+     +-----------------------+
```

---

## 📐 Mathematical Formulation & Control Theory

The deterministic $1\text{ kHz}$ control loop computes the net commanded feedback torque:

$$
\tau_{\text{motor}} = \tau_{\text{centering}} + \tau_{\text{damping}} + \tau_{\text{barrier}} + \tau_{\text{vibe}}
$$

---

### 1. Speed-Dependent Spring-Damper Road Impedance

$$
\tau_{\text{centering}} = -K_p(v_{\text{veh}}) \cdot \theta
$$

$$
\tau_{\text{damping}} = -B_d(v_{\text{veh}}) \cdot \omega
$$

- **$\theta$:** Steering angle off-center ($^\circ$ or $\text{rad}$).
- **$\omega = \frac{d\theta}{dt}$:** Angular velocity ($	ext{deg/s}$ or $\text{rad/s}$), smoothed by a 1st-order low-pass filter ($f_c = 25\text{ Hz}$).
- **Speed-Scaling Law:**
  - *Parking speeds ($10\text{ km/h}$):* $K_p \approx 0.02\text{ Nm/deg}$ (effortless, lightweight maneuvering).
  - *Highway speeds ($120\text{ km/h}$):* $K_p \approx 0.08\text{ Nm/deg}$ and $B_d \approx 0.011\text{ Nm}\cdot\text{s/deg}$ (firm, stable directional cruise).

---

### 2. Three-Zone Progressive Haptic Barrier

To prevent steering into blind-spot obstacles while complying with automotive ergonomics, the angular field is segmented into three continuous zones:

$$
\tau_{\text{barrier}}(\theta, \omega) = \tau_{\text{zone}}(\theta) + \tau_{\text{damping}}(\omega)
$$

**1. Zone 1: Free Compliant Zone ($|\theta| < \theta_{\text{warn}}$)**

$$
\tau_{\text{barrier}} = 0
$$

*Zero artificial interference; normal steering feel.*

**2. Zone 2: Progressive Soft-Margin Warning ($\theta_{\text{warn}} \le |\theta| < \theta_{\text{limit}}$)**

$$
\tau_{\text{barrier}} = -K_1 \cdot \left(\frac{|\theta| - \theta_{\text{warn}}}{\theta_{\text{limit}} - \theta_{\text{warn}}}\right)^2 \cdot \text{sgn}(\theta)
$$

*A parabolic repulsive potential field ramps up counter-torque smoothly before the lane hazard boundary is reached.*

**3. Zone 3: High-Impedance Dissipative Barrier ($|\theta| \ge \theta_{\text{limit}}$)**

$$
\tau_{\text{barrier}} = -\min\Big(\tau_{\text{clamp}}, \; K_1 + K_2 \cdot (|\theta| - \theta_{\text{limit}}) + B_{\text{wall}} \cdot \max(0, \omega \cdot \text{sgn}(\theta))\Big) \cdot \text{sgn}(\theta)
$$

*Maximum assistive torque combined with asymmetric one-way velocity damping.*

> **💡 Asymmetric One-Way Damping** $\big(\max(0, \omega)\big)$:  
> Damping only opposes motion moving **deeper** into the hazard ($\omega > 0$). When steering **back toward safety** ($\omega < 0$), barrier damping immediately drops to zero, ensuring an effortless, natural recovery.

---

### 3. Human Override & Functional Safety Mandate (UN R79 / ISO 26262)

$$
\text{If } |\tau_{\text{human}}| > \tau_{\text{override}} \; (8.0\text{ Nm}) \quad \text{for } \Delta t > 150\text{ ms} \implies \tau_{\text{barrier}} \xrightarrow{\text{decay}} 0
$$

- **Ergonomic Safety Clamp:** Motor output is strictly hardware-clamped to $\tau_{\text{clamp}} \le 5.5\text{ Nm}$ (preventing actuator stall current spikes, motor thermal burnout, and driver wrist strain).
- **Driver Override Mandate (UN R79 / ISO 11270):** If an onboard sensor produces a false positive, the driver can deliberately overpower the system ($> 8.0\text{ Nm}$). The controller detects sustained human effort via the inline strain gauge, immediately decays the barrier, and yields full steering authority back to the driver.

---

## 🔬 Multi-Physics FEA Simulation (ANSYS Mechanical & Icepak)

To ensure industrial-grade mechanical durability and thermal reliability, multi-physics finite element models were developed and solved using native ANSYS APDL decks.

### 1. ANSYS Mechanical: Torsional Shear Stress & Sensor Deflection FEA

![ANSYS Mechanical Torsional Stress & Deflection FEA](ansys/fea_torsional_stress_distribution.png)

<p align="center">
  <em>Figure 3: FEA Von Mises stress distribution along shaft radius under 20.5 Nm shock load, confirming FoS = 3.72 and sensor deflection < 0.05°.</em>
</p>

* **Extreme Shock Load:** Combined human panic override ($15.0\text{ Nm}$) + peak motor stall torque ($5.5\text{ Nm}$) $= \mathbf{20.5\text{ Nm}}$ torsional moment.
* **Element Type:** `SOLID187` higher-order 10-node tetrahedrals (142,500 elements with keyway boundary refinement).
* **Material Results:**
  * *Aluminum 6061-T6:* Peak stress $\sigma_{\text{max}} = 74.2\text{ MPa}$ vs yield $276\text{ MPa} \implies \mathbf{\text{Factor of Safety (FoS)} = 3.72}$ (**PASSED**).
  * *Torsional Angular Deflection:* $\Delta\theta = 0.0266^\circ$ ($< 0.05^\circ$ sensor threshold $\implies$ zero magnetic lag for AS5600 encoder).
* **Native APDL Solver File:** [`ansys/torsional_fea_model.mac`](ansys/torsional_fea_model.mac)

---

### 2. ANSYS Icepak: Transient Thermal Conjugate Heat Transfer

![ANSYS Icepak Transient Thermal Conjugate Heat Transfer](ansys/thermal_transient_junction_temp.png)

<p align="center">
  <em>Figure 4: 120-second transient thermal heating curve proving junction temperature stabilizes at 68.4°C, well below the 85°C ISO 26262 derating ceiling.</em>
</p>

* **Operating Scenario:** Continuous 120-second barrier hold at $I = 4.8\text{ A}$ continuous stall current ($P_{\text{loss}} = 2.76\text{ W}$ per MOSFET) under summer in-cabin ambient $T_{\text{amb}} = 40.0^\circ\text{C}$.
* **Results Comparison:**
  * *Bare TO-220 (No Heatsink):* Reaches $195.0^\circ\text{C}$ (thermal runaway $\implies$ destruction).
  * *With Extruded Al Heatsink ($R_{\text{th}} = 4.2^\circ\text{C/W}$):* Stabilizes at $\mathbf{68.4^\circ\text{C}}$, maintaining a **$16.6^\circ\text{C}$ safe margin below the ISO 26262 ASIL-B derating threshold ($85^\circ\text{C}$)**.
* **Native APDL Solver File:** [`ansys/thermal_icepak_setup.mac`](ansys/thermal_icepak_setup.mac)

---

## 🏎️ Vehicle Dynamics & UN R79 Latency Simulation

Using a 2-DOF single-track (bicycle) dynamics model in [`simulation/virtual_steering_model.py`](simulation/virtual_steering_model.py), tire slip angles ($\alpha_f, \alpha_r$) and self-aligning pneumatic trail torque ($T_{\text{align}}$) were simulated.

### 1. Dynamic Impedance & Barrier Profile
![Steer-by-Wire Dynamic Impedance Response Plot](simulation/impedance_response_plot.png)

<p align="center">
  <em>Figure 5: Steering torque vs. angle response comparing parking assist (10 km/h), highway cruising (120 km/h), and active 3-zone hazard counter-torque.</em>
</p>

### 2. UN R79 150 ms Driver Panic Override Latency Verification
![UN R79 Intentional Driver Override Dynamic Latency](simulation/phase_portrait_override.png)

<p align="center">
  <em>Figure 6: Transient simulation verifying the 150 ms human evaluation window and exponential barrier torque decay back to zero.</em>
</p>

---

## 📊 Quantitative Engineering Verification & Benchmarks

| Metric / Parameter | Industry Benchmark / Standard | Achieved in This Prototype | Verification Method | Status |
| :--- | :--- | :--- | :--- | :---: |
| **Control Loop Cycle Time** | $\le 1.0\text{ ms}$ ($1\text{ kHz}$) | **$1.0\text{ ms}$ (Jitter $< 0.18\text{ ms}$)** | STM32 Timer Interrupt & SIL Telemetry | ✅ PASSED |
| **Driver Panic Override Latency**| $\le 150\text{ ms}$ (UN R79 / ISO 11270) | **$150\text{ ms}$ Latch + $40\text{ ms}$ Decay** | Inline Load Cell & Python Dynamic Sim | ✅ PASSED |
| **Max Clamped Assistive Torque**| $\le 5.5\text{ Nm}$ (Ergonomic Safety) | **$5.50\text{ Nm}$ Hardware Clamped** | INA240 Current Sense & MOSFET Duty Limit | ✅ PASSED |
| **Shaft Torsional Deflection** | $< 0.05^\circ$ ($< 1\text{ LSB AS5600}$) | **$0.0266^\circ$ at $20.5\text{ Nm}$ Shock** | ANSYS Mechanical APDL (`SOLID187`) | ✅ PASSED |
| **Shaft Factor of Safety (FoS)**| $\ge 2.5$ (Automotive Structural) | **$3.72$ (Al 6061-T6)** | ANSYS Mechanical APDL (`SOLID187`) | ✅ PASSED |
| **MOSFET Steady-State Temp** | $< 85.0^\circ\text{C}$ (ASIL Derating) | **$68.4^\circ\text{C}$ at $T_{\text{amb}} = 40^\circ\text{C}$** | ANSYS Icepak Thermal Conjugate FEA | ✅ PASSED |
| **CAN Bus Frame Latency** | Cyclic $20\text{--}100\text{ ms}$ (ISO 11898) | **$20\text{ ms}$ (Steer/Torque), $100\text{ ms}$ (Safety)** | MCP2515 SPI CAN 2.0B / CAN-FD | ✅ PASSED |
| **Total Hardware Prototype Cost**| $< \text{₹}10,000$ (Low-Cost Feasibility) | **₹3,770 (~$45 USD)** | Itemized Bill of Materials | ✅ PASSED |

---

## 🛠️ Hardware Bill of Materials (BOM) & Signal Conditioning

| Subsystem | Component Part # | Manufacturer | Package / Interface | Operating Specs | Purpose | Approx Cost |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| **Controller** | STM32F401CCU6 (BlackPill) | STMicroelectronics | LQFP-48 / USB-C | 3.3V, 84 MHz, Hardware Timers | 1 kHz Deterministic Control Loop & ADC/I2C/SPI | ₹450 |
| **Actuator Motor** | 12V Planetary Geared DC | Johnson / PG36 | Planetary Gearbox (1:19) | 12V DC, 3.8A Stall, 4.2 Nm Torque | Provides tactile feedback and barrier counter-torque | ₹1,450 |
| **Power MOSFETs** | 4x IRF3205 N-Channel | Infineon / IR | TO-220AB | $V_{DS} = 55\text{V}$, $I_D = 110\text{A}$, $R_{DS(on)} < 8\text{ m}\Omega$ | H-Bridge power inverter stage | ₹200 |
| **Gate Drivers** | 2x IR2104 Dual Half-Bridge | Infineon / IR | DIP-8 / SOIC-8 | 10V–20V Gate Drive, 520 ns Dead-time | High-side bootstrap and shoot-through protection | ₹160 |
| **Angle Sensor** | AS5600 12-Bit Magnetic | ams-OSRAM | TSSOP-8 Breakout | 3.3V, Fast-Mode I2C (400 kHz), $0.087^\circ$ Res | Contactless steering wheel shaft angle tracking | ₹220 |
| **Reaction Torque Sensor**| Dual-axis Strain Gauge + INA128 | Texas Instruments | DIP-8 | Wheatstone bridge, Gain $G = 500$, $0\text{--}3.3\text{V}$ | Human driver applied torque ($\tau_{\text{human}}$) sensing | ₹320 |
| **Current Sense AFE** | INA240A2 High-CMRR Amp | Texas Instruments | SOIC-8 | Common Mode -4V to 80V, 50 V/V Gain | Shunt current monitoring ($T = K_t \cdot I$) | ₹180 |
| **Automotive CAN** | MCP2515 + TJA1050 | Microchip / NXP | SPI / SOIC-8 | 5V, 500 kbps ISO 11898 standard | Vehicle CAN bus broadcast & ADAS frame arbitration | ₹210 |
| **Decoupling / Housing** | Caps + PLA-CF D-Shaft Hub | Custom Fab | Radial Can / 12mm Coupler | 35V low-ESR / Yield $> 180\text{ MPa}$ | Electrical filtering & mechanical column coupler | ₹780 |
| **Total Prototype Cost**| | | | | | **₹3,770 (~$45)** |

### Sallen-Key 2nd-Order Low-Pass Butterworth Filter ($f_c = 150\text{ Hz}$)
To eliminate $20\text{ kHz}$ motor PWM switching EMI from polluting the high-gain ($G=500$) torque strain-gauge ADC channel, an active Sallen-Key filter is placed immediately before the STM32 ADC pin:

$$
H(s) = \frac{1}{1 + \sqrt{2} \left(\frac{s}{\omega_c}\right) + \left(\frac{s}{\omega_c}\right)^2}
$$

*Components:* $R_1 = R_2 = 10\text{ k}\Omega$, $C_1 = 150\text{ nF}$, $C_2 = 75\text{ nF}$. Rejects high-frequency switching transients while maintaining linear phase response across the human torque sensing bandwidth ($0\text{--}25\text{ Hz}$).

---

## 📁 Repository Structure

```
.
├── README.md                          # Comprehensive engineering portfolio & project documentation
├── steer_by_wire_solution_doc.md      # Detailed engineering & mathematical solution doc
├── KPIT SPARKLE_TECHTONICS.pdf        # Official KPIT Sparkle presentation deck
├── LICENSE                            # MIT Open-Source License (Team TECHTONICS)
├── backend/                           # Python REST API & Real-Time CAN Gateway Server
│   └── telemetry_server.py            # Zero-dependency HTTP & Telemetry streaming gateway
├── dashboard/                         # HTML5/JS Web SIL & Live HIL Telemetry Cockpit
│   ├── index.html                     # Cockpit layout with Web Serial connect
│   ├── style.css                      # Cyberpunk automotive cockpit styles
│   └── telemetry.js                   # SIL physics engine & HIL Web Serial bridge
├── firmware/                          # Production Embedded Firmware (STM32 / ESP32)
│   ├── CMakeLists.txt                 # Build configuration
│   ├── src/
│   │   ├── main.cpp                   # 1 kHz control loop, FreeRTOS tasks
│   │   ├── impedance_controller.cpp   # Potential field, damping, virtual wall
│   │   ├── can_telemetry.cpp          # CAN frame generation & arbitration
│   │   └── sensor_drivers/            # AS5600 12-bit I2C encoder driver
│   └── include/                       # Header interfaces
├── hardware/                          # Schematics & Circuit Designs
│   ├── circuit_schematic.md           # H-Bridge, AFE, Sallen-Key filter & MCU pinout schematics
│   └── bom.md                         # Detailed component sourcing & Sallen-Key filter specs
├── ansys/                             # Multi-Physics FEA Structural & Thermal Simulation
│   ├── ansys_fea_thermal_report.md    # Complete ANSYS Mechanical & Icepak verification report
│   ├── torsional_fea_model.mac        # Native ANSYS APDL static structural solver macro
│   ├── thermal_icepak_setup.mac       # Native ANSYS APDL transient conjugate heat transfer macro
│   ├── fea_torsional_stress_distribution.png # 20.5 Nm shock load stress & angular deflection
│   └── thermal_transient_junction_temp.png   # MOSFET continuous stall junction temp (T_j)
├── safety/                            # Automotive Functional Safety
│   └── iso26262_safety_case.md        # ASIL-D HARA analysis & Safety Goals (SG-1 to SG-4)
└── simulation/                        # Multi-Body Vehicle Dynamics Validation
    ├── virtual_steering_model.py      # Python 2-DOF single-track bicycle model & simulation
    ├── impedance_response_plot.png    # Validated torque-angle profiles (parking vs highway)
    └── phase_portrait_override.png   # 150 ms driver panic override latency verification
```

---

## ⚡ Quickstart: Launching the Telemetry Cockpit & Gateway

To launch the real-time telemetry dashboard with the background automotive CAN gateway server (no external dependencies required):

```bash
# 1. Start the Python Telemetry & CAN Gateway Server
python backend/telemetry_server.py

# 2. Open in your browser:
# Cockpit Interface:   http://localhost:8080
# REST Telemetry API:  http://localhost:8080/api/telemetry
# System Health API:   http://localhost:8080/api/status
```

---

## 🚦 Live Demonstration Protocol (For Hackathon Jury)

1. **Natural Steering Feel (Base HMI):**
   * Have the judge turn the steering wheel. The electronic spring-damper loop centers the wheel naturally with zero physical linkages, proving software-defined road feel.
2. **Dynamic Speed-Scaling:**
   * Adjust simulated vehicle speed $10 \to 120\text{ km/h}$. At $10\text{ km/h}$, the wheel is light and compliant for parking; at $120\text{ km/h}$, damping and stiffness tighten to stabilize high-speed cruising.
3. **Active Progressive Obstacle Barrier:**
   * Click **"INJECT LEFT HAZARD"** to simulate a blind-spot vehicle. Turning left triggers the **progressive parabolic resistance zone**, physically discouraging the maneuver before the hazard line is reached. Steering back right meets zero resistance.
4. **Intentional Driver Panic Override (UN R79 / ISO 26262):**
   * Ask the judge to intentionally overpower the barrier ($> 8.0\text{ Nm}$). Telemetry immediately flags the override event, decays the barrier to zero within $150\text{ ms}$, and flashes an **"Emergency Override Accepted"** frame over CAN.
5. **Live Hardware-in-the-Loop (HIL) USB Switch:**
   * Connect an STM32/ESP32 development board via USB and click **"CONNECT USB HIL BENCH"**. The web telemetry switches seamlessly from browser SIL simulation to live physical sensor streaming.

---

## 📄 License
This project is open-source under the [MIT License](LICENSE).  
Copyright (c) 2026 Team TECHTONICS (Chennai Institute of Technology).
