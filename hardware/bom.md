# Hardware Bill of Materials (BOM) & Electrical Architecture

## 1. Electrical Component Specifications

| Subsystem | Component Part # | Manufacturer | Package / Interface | Operating Voltage / Current | Purpose | Approx Cost (INR) |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| **Microcontroller** | STM32F401CCU6 (BlackPill) | STMicroelectronics | LQFP-48 / USB-C | 3.3V, 84 MHz, 256KB Flash | 1 kHz Deterministic Control Loop & ADC/I2C/SPI | ₹450 |
| **Actuator Motor** | 12V Planetary Geared DC | Johnson / PG36 | Planetary Gearbox (1:19) | 12V DC, 3.8A Stall, 4.2 Nm Torque | Provides tactile feedback and barrier counter-torque | ₹1,450 |
| **Power MOSFETs** | 4x IRF3205 N-Channel | Infineon / IR | TO-220AB | $V_{DS} = 55\text{V}$, $I_D = 110\text{A}$, $R_{DS(on)} < 8\text{ m}\Omega$ | H-Bridge power inverter stage | ₹200 |
| **Gate Drivers** | 2x IR2104 Dual Half-Bridge | Infineon / IR | DIP-8 / SOIC-8 | 10V–20V Gate Drive, 520 ns Dead-time | High-side bootstrap and shoot-through protection | ₹160 |
| **Angle Sensor** | AS5600 12-Bit Magnetic | ams-OSRAM | TSSOP-8 Breakout | 3.3V, Fast-Mode I2C (400 kHz), $0.087^\circ$ Res | Contactless steering wheel shaft angle tracking | ₹220 |
| **Reaction Torque Sensor**| Dual-axis Strain Gauge + INA128 | Texas Instruments | DIP-8 | Wheatstone bridge, Gain $G = 500$, $0\text{–}3.3\text{V}$ | Human driver applied torque ($\tau_{\text{human}}$) sensing | ₹320 |
| **Current Sense AFE** | INA240A2 High-CMRR Amp | Texas Instruments | SOIC-8 | Common Mode -4V to 80V, 50 V/V Gain | Shunt current monitoring ($T = K_t \cdot I$) | ₹180 |
| **Automotive CAN** | MCP2515 + TJA1050 | Microchip / NXP | SPI / SOIC-8 | 5V, 500 kbps ISO 11898 standard | Vehicle CAN bus broadcast & ADAS frame arbitration | ₹210 |
| **Power Decoupling** | 2x 2200 µF Low-ESR Caps | Panasonic | Radial Can | 35V, 105°C | Inductive flyback transient energy buffering | ₹80 |
| **Torsion Shaft & Hub** | 3D-Printed PLA-CF / Al 6061 | Custom Fab | 12mm D-Shaft coupler | Yield Strength > 180 MPa | Mechanical link between steering grip and motor | ₹500 |
| **Total Estimated Cost**| | | | | | **₹3,770 (~$45)** |

---

## 2. Analog Signal Conditioning: Sallen-Key Low-Pass Filter Design

To prevent motor PWM switching noise ($20\text{ kHz}$) and commutation spikes from alias-corrupting the high-gain strain gauge torque signal ($G = 500$), a **2nd-Order Sallen-Key Butterworth Low-Pass Filter** is placed immediately before the STM32 ADC pin:

* **Cut-off Frequency ($f_c$):** $150\text{ Hz}$
* **Components:**
  * $R_1 = R_2 = 10\text{ k}\Omega$ ($1\%$ metal film)
  * $C_1 = 150\text{ nF}$, $C_2 = 75\text{ nF}$
* **Transfer Function:**
  $$H(s) = \frac{1}{1 + \sqrt{2} \left(\frac{s}{\omega_c}\right) + \left(\frac{s}{\omega_c}\right)^2}$$
* **Result:** Rejects high-frequency electromagnetic noise while maintaining zero phase distortion across the human torque sensing bandwidth ($0\text{–}25\text{ Hz}$).
