# Steer-by-Wire Hardware Circuit Schematics & Pinout Mapping

This document provides the complete hardware schematic diagrams and pin interconnection mapping for the **Steer-by-Wire (SbW) Force-Feedback Actuator Prototype**.

---

## 1. Power Electronic Inverter Stage (Discrete MOSFET H-Bridge)

The power stage drives the 12V high-torque planetary geared DC actuator motor using complementary PWM with hardware shoot-through protection.

```
       +12V_POWER (Automotive Battery / Lab Supply)
           |
           +-----------------------------+-----------------------------+
           |                             |                             |
      [2200uF/35V]                  [2200uF/35V]                  [0.1uF Ceramic]
           |                             |                             |
          GND                           GND                           GND
           |                             |
           +-----------------------------+
           |                             |
      +----+----+                   +----+----+
      | Q1:     |                   | Q3:     |
      | IRF3205 |                   | IRF3205 |
      | (High-  |                   | (High-  |
      |  Side)  |                   |  Side)  |
      +----+----+                   +----+----+
           |                             |
           +---------> MOTOR_A           +---------> MOTOR_B
           |        (Actuator +)         |        (Actuator -)
           |                             |
      +----+----+                   +----+----+
      | Q2:     |                   | Q4:     |
      | IRF3205 |                   | IRF3205 |
      | (Low-   |                   | (Low-   |
      |  Side)  |                   |  Side)  |
      +----+----+                   +----+----+
           |                             |
           +--------------+--------------+
                          |
                   [R_SHUNT: 0.01Ω]  <------- High-Side / Shunt Sense (INA240)
                          |
                         GND
```

### Half-Bridge Gate Drivers (2x IR2104)
* **Bootstrap Diodes:** 1N4148 / UF4007 high-speed ultrafast recovery diodes.
* **Bootstrap Capacitors:** $10\,\mu\text{F}$ Tantalum + $0.1\,\mu\text{F}$ Ceramic in parallel across $V_B$ and $V_S$.
* **Hardware Dead-Time:** Fixed $520\text{ ns}$ integrated dead-time prevents high-side and low-side cross-conduction (shoot-through) during fast directional torque reversals.

---

## 2. Sensor & Analog Front-End (AFE) Conditioning

### Dual-Axis Strain Gauge Wheatstone Bridge & Sallen-Key Low-Pass Filter

```
 +3.3V_REF
     |
  +--+--+
  |  R  |  (Strain Gauge Bridge, R = 350Ω)
  +--+--+
     |--------> INA128 Pin 2 (Vin-)
  +--+--+
  |  R  |
  +--+--+
     |
    GND

 +3.3V_REF
     |
  +--+--+
  |  R  |
  +--+--+
     |--------> INA128 Pin 3 (Vin+)
  +--+--+
  |  R  |
  +--+--+
     |
    GND

 [Gain Resistor: R_G = 100Ω between Pin 1 & 8 => Gain G = 1 + 50000/100 = 501]

                    INA128 AMP
                 +--------------+
   Vin- -------->| Pin 2        |
   Vin+ -------->| Pin 3  Pin 6 |-------> SALLEN-KEY 2ND-ORDER LOW-PASS FILTER
                 +--------------+         (fc = 150 Hz, Butterworth)
                                                |
                                                v
                                         R1 (10k)      R2 (10k)
                              V_amp -----[====]----+---[====]----+----> STM32 PA1
                                                   |             |      (ADC1_IN1)
                                                 [C1]          [C2]
                                               (150nF)        (75nF)
                                                   |             |
                                                  GND           GND
```

---

## 3. Microcontroller Pinout Mapping (STM32F401 BlackPill)

| Peripheral | STM32 Pin | Function | Hardware Connection |
| :--- | :--- | :--- | :--- |
| **PWM Output** | `PA8` (TIM1_CH1) | 20 kHz Complementary PWM | IR2104 Gate Driver 1 (IN) |
| **DIR Control** | `PA9` (GPIO_OUT) | Motor Direction H-Bridge | IR2104 Gate Driver 2 (IN) |
| **Analog ADC** | `PA1` (ADC1_IN1) | Driver Reaction Torque | Sallen-Key Filter Output ($\tau_{\text{human}}$) |
| **Current Sense**| `PA2` (ADC1_IN2) | Motor Coil Current | INA240 Current Monitor Output |
| **I2C Fast-Mode**| `PB6` (SCL) | 400 kHz Sensor Bus | AS5600 12-Bit Rotary Encoder SCL |
| **I2C Fast-Mode**| `PB7` (SDA) | 400 kHz Sensor Bus | AS5600 12-Bit Rotary Encoder SDA |
| **CAN Bus (SPI)**| `PA5` (SCK) | SPI1 High-Speed Bus | MCP2515 CAN Controller SCK |
| **CAN Bus (SPI)**| `PA6` (MISO) | SPI1 High-Speed Bus | MCP2515 CAN Controller SO |
| **CAN Bus (SPI)**| `PA7` (MOSI) | SPI1 High-Speed Bus | MCP2515 CAN Controller SI |
| **CAN Bus (CS)** | `PB0` (GPIO_OUT) | Chip Select | MCP2515 CAN Controller CS |
| **CAN Interrupt**| `PB1` (EXTI1) | Frame RX Interrupt | MCP2515 CAN Controller INT |
| **Serial Debug** | `PA2` (USART2_TX)| 115.2 kbps Telemetry | USB-UART Bridge to Web Dashboard |
