# Steer-by-Wire Dynamic Force-Feedback Haptic Actuator with Active Obstacle Counter-Torque & Virtual Impedance

## 1. Executive Summary & Selection Rationale

### 1.1 Why Idea #3 Trumps Idea #4 for KPIT Technologies
Between **Idea 3 (Steer-by-Wire Force-Feedback Haptic Actuator)** and **Idea 4 (Biometric Grip & Micro-Climatic Thermal Actuation)**, **Idea 3 is decidedly superior** for the KPIT Hackathon based on four core criteria:

1. **Strategic Core Alignment with KPIT:**
   * KPIT is an undisputed global Tier-1 partner in **Software-Defined Vehicles (SDVs), Chassis Dynamics, and Autonomous Driving Systems**. 
   * Steer-by-Wire (SbW) is the defining hardware/software shift in modern chassis design (eliminating mechanical steering racks as seen in Tesla Cybertruck, Lexus RZ, and next-gen BMWs). Idea 3 touches the intersection of **Chassis Control, Functional Safety (ISO 26262), and Autonomous Handover**, whereas micro-climate thermal seats belong to secondary body comfort accessories.
2. **Technical Depth for Electronics & Communication Engineering (ECE):**
   * Idea 3 exercises power electronics (high-current MOSFET H-Bridges, gate drivers, inductive spike snubbing), precision closed-loop control (high-frequency PID/Impedance loops), analog sensor amplification (strain gauge Wheatstone bridges for torque sensing), and automotive communication (CAN bus arbitration).
3. **Live Demonstration Dynamics & Judge Engagement:**
   * Thermal changes take tens of seconds to be physically perceived. In contrast, **haptic torque feedback is instantaneous (sub-10 millisecond response)**. A judge gripping the steering wheel and experiencing dynamic centering stiffness, simulated asphalt/gravel texture, and an impenetrable "Virtual Wall" counter-torque as a simulated vehicle approaches an obstacle produces an unforgettable interactive demo.

---

## 2. Technical System Architecture

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
  |  |  - Active Virtual Barrier |  |          + τ_virtual_wall|  | - Thermal Cutoff     |  |
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

## 3. Hardware Circuit Design & Component Breakdown

### 3.1 Power Electronic Driver Stage
* **Actuator Selection:** High-torque planetary brushed DC motor (12V–24V, rated stall torque > 3.5 Nm) or low-cogging high-pole BLDC motor.
* **H-Bridge Inverter Stage:**
  * **Switching Elements:** 4x N-Channel Power MOSFETs (IRF3205 or AOD4184, $R_{DS(on)} < 8\text{ m}\Omega$, $V_{DS} = 55\text{V}$, $I_D = 110\text{A}$) mounted on an aluminum heatsink.
  * **Gate Drivers:** Dual half-bridge driver ICs (IR2104 / IR2184) providing high-voltage bootstrap rail and integrated hardware dead-time generation ($500\text{ ns}$) to prevent shoot-through.
  * **Snubber & Decoupling:** Schottky flyback protection diodes (SS34/MBR20100) and low-ESR electrolytic capacitor bank ($2200\,\mu\text{F} \times 2$) to buffer inductive kickback transients during rapid torque reversal.

### 3.2 Sensor & Analog Front-End (AFE) Subsystem
1. **Shaft Position & Angular Velocity Sensing:**
   * **Sensor:** AS5600 contactless 12-bit magnetic rotary encoder mounted axially on a diametrically magnetized neodymium magnet at the base of the steering column.
   * **Resolution:** 4096 counts per revolution ($0.087^\circ$ precision) communicating over I2C at 400 kHz with fast hardware interrupts.
2. **Driver Dynamic Reaction Torque Sensing:**
   * **Sensor:** Dual-axis torsion strain gauge load cell mounted inline between the steering grip and the motor drive gear.
   * **Analog Signal Conditioning:** 
     * Full Wheatstone bridge arrangement.
     * Precision Instrumentation Amplifier (INA128 / AD620) configured for a gain of $G = 500$.
     * Active 2nd-order Sallen-Key low-pass filter ($f_c = 150\text{ Hz}$) to eliminate motor commutation electromagnetic interference (EMI) before ADC sampling.
3. **Current Feedback & Motor Health Monitoring:**
   * High-side/bidirectional current sense amplifier (INA240 with high common-mode rejection ratio) or Hall-effect sensor (ACS712-20A).
   * Measures instantaneous coil current to calculate actual electromagnetic torque ($T = K_t \cdot I$) for closed-loop torque control.

---

## 4. Software & Control Algorithms

### 4.1 Closed-Loop Impedance & Progressive Haptic Barrier Model
The microcontroller runs a deterministic $1\text{ kHz}$ control loop implementing an active human-in-the-loop admittance/impedance model. In compliance with automotive functional safety standards (ISO 26262 and UN R79 / ISO 11270 for lane-keeping haptic intervention), the system functions as an **assistive haptic intervention, not an absolute mechanical lock**, ensuring the driver always retains override authority.

```math
\tau_{\text{motor}} = \tau_{\text{centering}} + \tau_{\text{damping}} + \tau_{\text{barrier}} + \tau_{\text{haptic\_vibe}}
```

#### 1. Self-Centering Spring Effect:
```math
\tau_{\text{centering}} = -K_p(v_{\text{veh}}) \cdot \theta
```
*(Where $\theta$ is the steering angle off-center, simulating caster angle alignment in real vehicle suspension).*

#### 2. Speed-Dependent Viscous Damping:
```math
\tau_{\text{damping}} = -B_d(v_{\text{veh}}) \cdot \omega
```
*(Where $\omega = \frac{d\theta}{dt}$, providing stable, heavy resistance at high simulated vehicle speeds).*

#### 3. Progressive Potential Field & Assistive Haptic Barrier (Obstacle Mitigation):
Rather than a late step-barrier after the vehicle enters danger, a **three-zone progressive repulsive field** is established:

```math
\tau_{\text{barrier}}(\theta, \omega) = \tau_{\text{zone}}(\theta) + \tau_{\text{damping}}(\omega)
```

* **Zone 1: Free Compliant Zone** ($|\theta| < \theta_{\text{warn}}$):
  ```math
  \tau_{\text{barrier}} = 0
  ```
  *(Normal steering feel; zero artificial interference).*

* **Zone 2: Progressive Soft-Margin Resistance** ($\theta_{\text{warn}} \le |\theta| < \theta_{\text{limit}}$):
  ```math
  \tau_{\text{barrier}} = -K_1 \cdot \left(\frac{|\theta| - \theta_{\text{warn}}}{\theta_{\text{limit}} - \theta_{\text{warn}}}\right)^2 \cdot \operatorname{sgn}(\theta)
  ```
  *(A parabolic stiffness gradient ramps up counter-torque before crossing the unsafe boundary).*

* **Zone 3: High-Impedance Dissipative Barrier** ($|\theta| \ge \theta_{\text{limit}}$):
  ```math
  \tau_{\text{barrier}} = -\min\Big(\tau_{\text{clamp}}, \; K_1 + K_2 \cdot (|\theta| - \theta_{\text{limit}}) + B_{\text{wall}} \cdot \max(0, \omega \cdot \operatorname{sgn}(\theta))\Big) \cdot \operatorname{sgn}(\theta)
  ```
  *(Imposes an assistive torque plateau combined with asymmetric one-way velocity damping).*

> **💡 Asymmetric One-Way Damping ($\max(0, \omega)$):**  
> Damping only opposes motion moving **deeper** into danger ($\omega > 0$). As soon as the driver steers back toward safety ($\omega < 0$), barrier resistance drops to zero, encouraging natural corrective recovery.

#### 4. Driver Intentional Override & Clamping Protocol (UN R79 / ISO 26262 Compliance):
```math
\text{If } |\tau_{\text{human}}| > \tau_{\text{override}} \; (8.0\text{ Nm}) \quad \text{for } \Delta t > 150\text{ ms} \implies \tau_{\text{barrier}} \xrightarrow{\text{decay}} 0
```
* **Ergonomic Safety Clamp:** Motor output is strictly clamped to $\tau_{\text{clamp}} \le 5.5\text{ Nm}$ to prevent actuator runaway, motor stall current overloads, or driver wrist injury.
* **Human Override Detection:** The inline reaction torque sensor continuously samples $\tau_{\text{human}}$. If the driver intentionally exerts an evasive counter-torque exceeding the override threshold ($\tau_{\text{human}} > \tau_{\text{override}} \approx 8.0\text{ Nm}$ for $\Delta t > 150\text{ ms}$), the system yields authority, softly decaying $\tau_{\text{barrier}} \to 0$ and allowing the driver full manual steering capability in emergency evasion scenarios.

#### 5. Surface Texture Synthesis (Haptic Feedback):
Superimposes high-frequency micro-vibrations ($f = 60\text{–}120\text{ Hz}$, square/sine modulation) to simulate road rumble strips, lane departure drift, or gravel surfaces.

---

## 5. Implementation Roadmap & Bill of Materials (BOM)

### 5.1 Estimated Component Budget
| Component | Part Description | Approx Cost (INR) |
| :--- | :--- | :--- |
| **Microcontroller Board** | STM32F401 BlackPill / ESP32-S3 | ₹450 – ₹650 |
| **Actuator Motor** | 12V High-Torque Planetary Gear DC Motor (with encoder mount) | ₹1,200 – ₹1,800 |
| **Power Stage** | 4x IRF3205 + 2x IR2104 Gate Drivers + Passive Heatsinks | ₹350 – ₹500 |
| **Angle Sensor** | AS5600 Magnetic Rotary Encoder Module + Magnet | ₹180 – ₹250 |
| **Torque Sensor / Amp** | Strain Gauge Bar / Mini Load Cell + INA128 / HX711 | ₹250 – ₹400 |
| **Current Sensor** | ACS712-20A or INA240 Current Monitor IC | ₹150 – ₹220 |
| **CAN Transceiver** | MCP2515 SPI Controller + TJA1050 Transceiver | ₹180 – ₹250 |
| **Structural Frame** | 3D-Printed Steering Wheel Hub + Acrylic Base Plate | ₹600 – ₹900 |
| **Total Estimated BOM** | | **₹3,360 – ₹5,070 (~$40 – $60)** |

---

## 6. Hackathon Live Demonstration Protocol

### Step-by-Step Judging Presentation
1. **Normal Steering Feel (Base HMI):**
   * Have the judge grab the wheel. They feel smooth, natural mechanical resistance and spring-return behavior, proving that pure code and electronics can mimic physical steering rods.
2. **Speed-Variable Resistance:**
   * Adjust simulated vehicle speed on an accompanying GUI. At $10\text{ km/h}$, the wheel is light and easy to spin (parking assist). At $120\text{ km/h}$, the motor ramps up damping resistance, becoming tight and stable.
3. **Progressive Obstacle Barrier Demonstration (The Technical Highlight):**
   * Trigger an obstacle on the left lane (simulated via an ultrasonic sensor or GUI toggle).
   * As the judge gently turns toward the left hazard, demonstrate the **progressive soft-margin resistance zone**: the wheel pushes back with escalating parabolic stiffness *before* the hazard line is reached, naturally discouraging the maneuver.
   * If the judge continues into the limit zone, show the high-damping barrier. Show how steering away from the barrier to the right meets zero resistance.
4. **Intentional Driver Override (Addressing Functional Safety):**
   * Ask the judge to intentionally force the wheel past the barrier (exerting firm panic torque $> 8\text{ Nm}$).
   * Show on the live telemetry how the strain gauge detects deliberate driver override, smoothly disengages the barrier, and flashes an "Emergency Driver Override Accepted" event over CAN, directly meeting UN R79 and ISO 26262 safety requirements.
5. **Lane Departure Rumble Strip & Telemetry:**
   * Drift the wheel near an imaginary lane line; the motor emits an audible, tactile vibrational buzz directly through the driver's hands.
   * Show an oscilloscope or serial plotter tracking motor current, torque strain, and CAN bus arbitration frames in real time under current-limiting conditions.
