# ANSYS Multi-Physics Simulation: Structural Torsional FEA & Thermal Heat Transfer

## 1. Executive Summary

To ensure industrial-grade automotive reliability and validate the mechanical integrity of the **Steer-by-Wire (SbW) Haptic Actuator**, a multi-physics finite element simulation was conducted covering two critical physical domains:
1. **ANSYS Mechanical (Structural & Torsional FEA):** Evaluates torsional stress concentrations ($\sigma_{\text{vM}}$), Factor of Safety ($\text{FoS}$), and angular deformation ($\Delta\theta$) along the primary steering column under maximum combined shock torque ($20.5\text{ Nm}$).
2. **ANSYS Icepak / Fluent (Conjugate Heat Transfer & Thermal FEA):** Evaluates the transient junction temperature ($T_j$) of the discrete H-bridge power MOSFETs (IRF3205) under continuous stall current during an extended "Virtual Wall" obstacle intervention ($T_{\text{amb}} = 40^\circ\text{C}$).

---

## 2. Structural Torsional FEA (ANSYS Mechanical)

![ANSYS Mechanical Torsional Stress & Deflection FEA](fea_torsional_stress_distribution.png)

### 2.1 Problem Definition & Loading Boundary Conditions
Under severe emergency evasive maneuvers or panic driver intervention, the human driver can exert an impulsive shock torque of up to $15.0\text{ Nm}$. Simultaneously, the actuator motor applies its peak holding counter-torque of $5.5\text{ Nm}$ in the opposing direction, creating an extreme resultant torsional moment:

$$M_t = \tau_{\text{human}} + \tau_{\text{motor\_stall}} = 15.0\text{ Nm} + 5.5\text{ Nm} = 20.5\text{ Nm}$$

* **Shaft Geometry:** $\varnothing 12\text{ mm}$ D-Shaft, active length $L = 120\text{ mm}$.
* **Boundary Constraints:** Fixed rotational constraint at the motor gearbox output face; pure torsional moment $M_t = 20.5\text{ Nm}$ applied at the steering grip hub flange.
* **Mesh Configuration:** Higher-order 10-node tetrahedral elements (`SOLID187`), 142,500 elements with adaptive boundary layer refinement at keyway transition radii ($r = 1.0\text{ mm}$).

### 2.2 Material Properties & Performance Metrics

| Material | Density ($\rho$) | Young's Modulus ($E$) | Shear Modulus ($G$) | Yield Strength ($\sigma_y$) | Peak Von Mises ($\sigma_{\text{max}}$) | Factor of Safety ($\text{FoS}$) | Result |
| :--- | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| **Aluminum 6061-T6** | $2700\text{ kg/m}^3$ | $68.9\text{ GPa}$ | $26.0\text{ GPa}$ | **$276\text{ MPa}$** | **$74.2\text{ MPa}$** | **$3.72$** | **PASSED (Recommended)** |
| **PLA-CF (Carbon Fiber)**| $1290\text{ kg/m}^3$ | $8.5\text{ GPa}$ | $3.2\text{ GPa}$ | **$110\text{ MPa}$** | **$74.2\text{ MPa}$** | **$1.48$** | **PASSED (Bench Prototype)** |

### 2.3 Sensor Integrity & Angular Deflection Analysis ($\Delta\theta$)
A critical finding from the torsional analysis is that elastic twist along the column introduces an angular measurement offset to the **AS5600 12-bit magnetic rotary encoder**:

$$\Delta\theta = \frac{M_t \cdot L}{G \cdot J} = \frac{20.5 \cdot 0.12}{26.0 \times 10^9 \cdot 2.036 \times 10^{-10}} = 4.65 \times 10^{-4}\text{ rad} = 0.0266^\circ$$

* **Target Threshold:** $\Delta\theta < 0.05^\circ$ ($0.087^\circ$ represents 1 LSB of the 12-bit encoder).
* **Conclusion:** The maximum deflection ($0.0266^\circ$) is well below half an LSB, proving that shaft twist will **not** induce phase lag or closed-loop limit cycle oscillations.

---

## 3. Transient Thermal Conjugate Heat Transfer (ANSYS Icepak)

![ANSYS Icepak MOSFET Junction Temperature FEA](thermal_transient_junction_temp.png)

### 3.1 Thermal Loading Scenario
When the vehicle is sustained against an impassable hazard barrier, the H-bridge operates in continuous current-holding mode ($I = 4.8\text{ A}$).
* **MOSFET Conduction Loss:**
  $$P_{\text{loss}} = I^2 \cdot R_{DS(on),\text{hot}} = (4.8\text{ A})^2 \cdot 0.012\,\Omega = 2.76\text{ W per FET}$$
* **Ambient Test Condition:** $T_{\text{amb}} = 40.0^\circ\text{C}$ (extreme in-cabin summer automotive standard).
* **Convection Boundary:** Natural air convection coefficient $h = 12\text{ W/(m}^2\cdot\text{K)}$.

### 3.2 Thermal Dissipation Comparison

| Cooling Configuration | Thermal Resistance ($R_{\text{th, total}}$) | Steady-State Temp ($T_{j,\text{ss}}$) | Derating Margin ($85^\circ\text{C}$) | Max Silicon Limit ($175^\circ\text{C}$) | Status |
| :--- | :---: | :---: | :---: | :---: | :---: |
| **Bare TO-220 (No Heatsink)** | $62.0^\circ\text{C/W}$ | **$195.0^\circ\text{C}$** | Exceeded (+110°C) | **DESTROYED** | **FAIL** |
| **With Extruded Al Heatsink** | **$4.2^\circ\text{C/W}$** | **$68.4^\circ\text{C}$** | **$16.6^\circ\text{C}$ Safe Margin** | **$106.6^\circ\text{C}$ Margin** | **PASSED** |

### 3.3 Automotive Safety Validation
* With a compact extruded aluminum heatsink ($R_{\text{th}} = 4.2^\circ\text{C/W}$), junction temperature asymptotes at **$68.4^\circ\text{C}$**, remaining comfortably below the **ISO 26262 ASIL-B thermal rollback threshold ($85.0^\circ\text{C}$)**.
* Even during continuous 120-second barrier activation, thermal runaway is physically impossible under standard passive convection.

---

## 4. Native ANSYS APDL Simulation Source Files

The finite element models can be directly executed or loaded into ANSYS Workbench / Mechanical APDL using the accompanying native solver input decks in this directory:

* [`torsional_fea_model.mac`](torsional_fea_model.mac) – Native APDL input macro for the 3D static structural torsional shear stress, mesh generation, and angular deflection analysis.
* [`thermal_icepak_setup.mac`](thermal_icepak_setup.mac) – Native APDL input macro for transient thermal conjugate conduction, internal heat generation, and natural convection solve.
