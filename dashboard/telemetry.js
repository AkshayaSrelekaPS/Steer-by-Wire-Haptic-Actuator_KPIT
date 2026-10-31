/**
 * KPIT Steer-by-Wire Software-in-the-Loop (SIL) Telemetry & Physics Engine
 * Implements 1 kHz deterministic impedance, progressive potential field,
 * driver intentional override logic (UN R79), and simulated CAN-FD telemetry.
 */

// STATE MANAGEMENT
const state = {
  steeringAngle: 0,       // θ in degrees [-90 to +90]
  steeringVelocity: 0,    // ω in deg/s
  previousAngle: 0,
  humanTorque: 2.5,       // Applied driver torque (Nm)
  vehicleSpeed: 40,       // Speed in km/h [0 to 160]
  
  // Hazards
  leftHazardActive: false,
  rightHazardActive: false,
  rumbleActive: false,

  // Calculated torques
  springTorque: 0,        // Centering
  dampingTorque: 0,       // Speed-dependent viscosity
  barrierTorque: 0,       // Obstacle counter-torque
  rumbleTorque: 0,        // High-frequency tactile vibe
  netTorque: 0,           // Command sent to H-bridge

  // Safety & Override
  isOverrideActive: false,
  overrideHoldCounter: 0,
  
  // Limits & Constants
  warnAngle: 18.0,        // θ_warn (degrees)
  limitAngle: 35.0,       // θ_limit (degrees)
  clampTorque: 5.5,       // Max allowable motor torque (Nm)
  overrideThreshold: 7.5, // Panic human torque threshold (Nm)
};

// DOM ELEMENTS
const wheelSvg = document.getElementById('steeringWheelSvg');
const angleDisplay = document.getElementById('angleDisplay');
const steeringSlider = document.getElementById('steeringInputSlider');
const sliderAngleVal = document.getElementById('sliderAngleVal');
const humanTorqueSlider = document.getElementById('humanTorqueSlider');
const humanTorqueVal = document.getElementById('humanTorqueVal');
const vehicleSpeedSlider = document.getElementById('vehicleSpeedSlider');
const speedDisplay = document.getElementById('speedDisplay');
const springTorqueDisplay = document.getElementById('springTorqueDisplay');
const barrierTorqueDisplay = document.getElementById('barrierTorqueDisplay');
const netTorqueDisplay = document.getElementById('netTorqueDisplay');
const hazardStatusBadge = document.getElementById('hazardStatusBadge');
const egoVehicle = document.getElementById('egoVehicle');
const leftHazard = document.getElementById('leftHazard');
const rightHazard = document.getElementById('rightHazard');
const leftBarrierArc = document.getElementById('leftBarrierArc');
const rightBarrierArc = document.getElementById('rightBarrierArc');
const complianceBanner = document.getElementById('complianceBanner');
const compTitle = document.getElementById('compTitle');
const compDesc = document.getElementById('compDesc');
const overrideBadge = document.getElementById('overrideBadge');
const canLogStream = document.getElementById('canLogStream');
const systemClock = document.getElementById('systemClock');
const diagCurrent = document.getElementById('diagCurrent');
const diagTemp = document.getElementById('diagTemp');
const diagPwm = document.getElementById('diagPwm');

// BUTTONS
const btnHazardLeft = document.getElementById('btnHazardLeft');
const btnHazardRight = document.getElementById('btnHazardRight');
const btnRumble = document.getElementById('btnRumble');
const btnResetScenario = document.getElementById('btnResetScenario');
const speedPresets = document.querySelectorAll('.btn-preset');

// SETUP CHART.JS REAL-TIME PLOTTER
const ctx = document.getElementById('telemetryChart').getContext('2d');
const maxDataPoints = 40;
const chartData = {
  labels: Array(maxDataPoints).fill(''),
  datasets: [
    {
      label: 'Driver Input Angle (deg)',
      borderColor: '#00f2fe',
      borderWidth: 2,
      pointRadius: 0,
      data: Array(maxDataPoints).fill(0),
      yAxisID: 'yAngle'
    },
    {
      label: 'Motor Torque Command (Nm)',
      borderColor: '#ef4444',
      borderWidth: 2,
      pointRadius: 0,
      data: Array(maxDataPoints).fill(0),
      yAxisID: 'yTorque'
    },
    {
      label: 'Human Applied Torque (Nm)',
      borderColor: '#8b5cf6',
      borderWidth: 1.5,
      borderDash: [4, 4],
      pointRadius: 0,
      data: Array(maxDataPoints).fill(2.5),
      yAxisID: 'yTorque'
    }
  ]
};

const telemetryChart = new Chart(ctx, {
  type: 'line',
  data: chartData,
  options: {
    responsive: true,
    maintainAspectRatio: false,
    animation: false,
    scales: {
      x: { display: false },
      yAngle: {
        type: 'linear',
        position: 'left',
        min: -90,
        max: 90,
        grid: { color: 'rgba(255, 255, 255, 0.05)' },
        ticks: { color: '#00f2fe', font: { family: 'JetBrains Mono', size: 9 } }
      },
      yTorque: {
        type: 'linear',
        position: 'right',
        min: -8,
        max: 8,
        grid: { display: false },
        ticks: { color: '#ef4444', font: { family: 'JetBrains Mono', size: 9 } }
      }
    },
    plugins: {
      legend: {
        labels: { color: '#9ca3af', font: { family: 'Outfit', size: 10 } }
      }
    }
  }
});

// EVENT LISTENERS
steeringSlider.addEventListener('input', (e) => {
  state.steeringAngle = parseFloat(e.target.value);
  sliderAngleVal.textContent = `${state.steeringAngle.toFixed(1)}°`;
});

humanTorqueSlider.addEventListener('input', (e) => {
  state.humanTorque = parseFloat(e.target.value);
  humanTorqueVal.textContent = `${state.humanTorque.toFixed(1)} Nm`;
});

vehicleSpeedSlider.addEventListener('input', (e) => {
  state.vehicleSpeed = parseInt(e.target.value);
  speedDisplay.textContent = `${state.vehicleSpeed} km/h`;
  updateActiveSpeedButton(state.vehicleSpeed);
});

speedPresets.forEach(btn => {
  btn.addEventListener('click', () => {
    const spd = parseInt(btn.dataset.speed);
    vehicleSpeedSlider.value = spd;
    state.vehicleSpeed = spd;
    speedDisplay.textContent = `${spd} km/h`;
    updateActiveSpeedButton(spd);
  });
});

function updateActiveSpeedButton(spd) {
  speedPresets.forEach(b => {
    b.classList.toggle('active', parseInt(b.dataset.speed) === spd);
  });
}

// HAZARD TOGGLES
btnHazardLeft.addEventListener('click', () => {
  state.leftHazardActive = !state.leftHazardActive;
  btnHazardLeft.classList.toggle('active', state.leftHazardActive);
  leftHazard.classList.toggle('active', state.leftHazardActive);
  leftBarrierArc.classList.toggle('active', state.leftHazardActive);
  updateHazardStatus();
});

btnHazardRight.addEventListener('click', () => {
  state.rightHazardActive = !state.rightHazardActive;
  btnHazardRight.classList.toggle('active', state.rightHazardActive);
  rightHazard.classList.toggle('active', state.rightHazardActive);
  rightBarrierArc.classList.toggle('active', state.rightHazardActive);
  updateHazardStatus();
});

btnRumble.addEventListener('click', () => {
  state.rumbleActive = !state.rumbleActive;
  btnRumble.classList.toggle('active', state.rumbleActive);
});

btnResetScenario.addEventListener('click', () => {
  state.leftHazardActive = false;
  state.rightHazardActive = false;
  state.rumbleActive = false;
  btnHazardLeft.classList.remove('active');
  btnHazardRight.classList.remove('active');
  btnRumble.classList.remove('active');
  leftHazard.classList.remove('active');
  rightHazard.classList.remove('active');
  leftBarrierArc.classList.remove('active');
  rightBarrierArc.classList.remove('active');
  updateHazardStatus();
});

function updateHazardStatus() {
  if (state.leftHazardActive || state.rightHazardActive) {
    hazardStatusBadge.textContent = 'HAZARD INTERLOCK ACTIVE';
    hazardStatusBadge.className = 'tag alert danger';
  } else {
    hazardStatusBadge.textContent = 'PATH CLEAR';
    hazardStatusBadge.className = 'tag alert';
  }
}

// MATHEMATICAL IMPEDANCE & PHYSICS ENGINE (Executed at 50 Hz Simulation Tick)
let tickCount = 0;

function computePhysics() {
  // 1. Angular Velocity estimation: ω = dθ / dt
  state.steeringVelocity = (state.steeringAngle - state.previousAngle) * 50; // deg/s
  state.previousAngle = state.steeringAngle;

  // 2. Speed-Dependent Spring-Centering Model
  // Lower at parking (0.02 Nm/deg), higher at highway (0.08 Nm/deg)
  const speedFactor = 0.02 + (state.vehicleSpeed / 160) * 0.06;
  state.springTorque = -1 * speedFactor * state.steeringAngle;

  // 3. Speed-Dependent Viscous Damping Model (τ_damp = -B * ω)
  const dampingFactor = 0.003 + (state.vehicleSpeed / 160) * 0.008;
  state.dampingTorque = -1 * dampingFactor * state.steeringVelocity;

  // 4. Progressive Potential Field & Barrier Calculation
  state.barrierTorque = 0;

  // Check Left Hazard (Turning Negative θ)
  if (state.leftHazardActive && state.steeringAngle < -state.warnAngle) {
    const excess = Math.abs(state.steeringAngle) - state.warnAngle;
    const span = state.limitAngle - state.warnAngle;
    if (Math.abs(state.steeringAngle) < state.limitAngle) {
      // Zone 2: Soft margin parabolic stiffness
      state.barrierTorque = 2.5 * Math.pow(excess / span, 2);
    } else {
      // Zone 3: High-impedance dissipative barrier with one-way damping
      const penetration = Math.abs(state.steeringAngle) - state.limitAngle;
      const oneWayDamping = state.steeringVelocity < 0 ? Math.abs(state.steeringVelocity) * 0.015 : 0;
      state.barrierTorque = Math.min(state.clampTorque, 2.5 + penetration * 0.25 + oneWayDamping);
    }
  }

  // Check Right Hazard (Turning Positive θ)
  if (state.rightHazardActive && state.steeringAngle > state.warnAngle) {
    const excess = state.steeringAngle - state.warnAngle;
    const span = state.limitAngle - state.warnAngle;
    if (state.steeringAngle < state.limitAngle) {
      // Zone 2: Soft margin
      state.barrierTorque = -2.5 * Math.pow(excess / span, 2);
    } else {
      // Zone 3: High-impedance wall
      const penetration = state.steeringAngle - state.limitAngle;
      const oneWayDamping = state.steeringVelocity > 0 ? Math.abs(state.steeringVelocity) * 0.015 : 0;
      state.barrierTorque = -Math.min(state.clampTorque, 2.5 + penetration * 0.25 + oneWayDamping);
    }
  }

  // 5. Driver Intentional Override Detection (UN R79 Compliance)
  if (Math.abs(state.barrierTorque) > 0.5 && state.humanTorque > state.overrideThreshold) {
    state.overrideHoldCounter++;
    if (state.overrideHoldCounter > 8) { // >160 ms sustained panic force
      state.isOverrideActive = true;
      // Decay barrier to yield manual control
      state.barrierTorque *= 0.15;
    }
  } else {
    state.overrideHoldCounter = 0;
    if (state.humanTorque <= state.overrideThreshold - 1.0) {
      state.isOverrideActive = false;
    }
  }

  // 6. Lane Departure Rumble Strip Synthesis (80 Hz oscillation)
  state.rumbleTorque = 0;
  if (state.rumbleActive) {
    state.rumbleTorque = 1.2 * Math.sin(tickCount * 1.6); // Oscillating buzz
  }

  // 7. Net Torque Summation with Ergonomic Safety Clamping
  let rawNet = state.springTorque + state.dampingTorque + state.barrierTorque + state.rumbleTorque;
  state.netTorque = Math.max(-state.clampTorque, Math.min(state.clampTorque, rawNet));

  tickCount++;
}

// UPDATE UI & HARDWARE SIMULATION METRICS
function updateUI() {
  // Update Steering Wheel SVG Rotation
  wheelSvg.style.transform = `rotate(${state.steeringAngle}deg)`;
  angleDisplay.textContent = `${state.steeringAngle.toFixed(1)}°`;

  // Update Ego Car Translation in Visualizer
  const carX = (state.steeringAngle / 90) * 120;
  egoVehicle.style.transform = `translateX(${carX}px) rotate(${state.steeringAngle * 0.25}deg)`;

  // Update Stat Displays
  springTorqueDisplay.textContent = `${state.springTorque.toFixed(2)} Nm`;
  barrierTorqueDisplay.textContent = `${state.barrierTorque.toFixed(2)} Nm`;
  netTorqueDisplay.textContent = `${state.netTorque.toFixed(2)} Nm`;

  // Update Safety Compliance Banner
  if (state.isOverrideActive) {
    complianceBanner.className = 'compliance-banner override';
    compTitle.textContent = 'UN R79 PANIC OVERRIDE ACTIVE';
    compDesc.textContent = `Driver force (${state.humanTorque.toFixed(1)} Nm) exceeded threshold (>7.5 Nm). Haptic barrier yielded to human authority.`;
    overrideBadge.textContent = 'OVERRIDE LATCHED';
  } else if (Math.abs(state.barrierTorque) > 1.5) {
    complianceBanner.className = 'compliance-banner override';
    compTitle.textContent = 'ACTIVE ASSISTIVE BARRIER ENGAGED';
    compDesc.textContent = 'Applying counter-torque to prevent unsafe trajectory into hazardous zone.';
    overrideBadge.textContent = 'BARRIER ACTIVE';
  } else {
    complianceBanner.className = 'compliance-banner';
    compTitle.textContent = 'NORMAL OPERATIONAL MODE (ISO 26262 ASIL-D)';
    compDesc.textContent = 'Closed-loop admittance active. Driver retains full compliant override authority.';
    overrideBadge.textContent = 'OVERRIDE IDLE';
  }

  // Power Stage Diagnostic Emulation
  const currentDraw = Math.abs(state.netTorque) * 0.95 + 0.35;
  diagCurrent.textContent = `${currentDraw.toFixed(2)} A`;
  diagPwm.textContent = `${((Math.abs(state.netTorque) / state.clampTorque) * 100).toFixed(1)} %`;
  diagTemp.textContent = `${(32.0 + currentDraw * 2.8).toFixed(1)} °C`;

  // Update Chart Data
  chartData.datasets[0].data.shift();
  chartData.datasets[0].data.push(state.steeringAngle);

  chartData.datasets[1].data.shift();
  chartData.datasets[1].data.push(state.netTorque);

  chartData.datasets[2].data.shift();
  chartData.datasets[2].data.push(state.humanTorque);

  telemetryChart.update();
}

// SIMULATE CAN-FD BUS STREAM (Periodic CAN Frames)
const canIds = [
  { id: '0x102', dlc: 8, name: 'STEER_STATUS_THETA', rate: 20 },
  { id: '0x104', dlc: 8, name: 'MOTOR_TORQUE_CMD', rate: 20 },
  { id: '0x220', dlc: 4, name: 'ADAS_HAZARD_FLAG', rate: 50 },
  { id: '0x350', dlc: 8, name: 'SAFETY_OVERRIDE_UNR79', rate: 100 }
];

let canTick = 0;
function emitCanFrame() {
  canTick++;
  const target = canIds[canTick % canIds.length];

  // Synthesize hex payload based on state
  const hexAngle = Math.floor((state.steeringAngle + 90) * 100).toString(16).padStart(4, '0').toUpperCase();
  const hexTorque = Math.floor((state.netTorque + 10) * 100).toString(16).padStart(4, '0').toUpperCase();
  const hexOverride = state.isOverrideActive ? '01' : '00';
  const hexHazard = (state.leftHazardActive ? '01' : '00') + (state.rightHazardActive ? '02' : '00');
  
  let payload = `${hexAngle} ${hexTorque} ${hexHazard} ${hexOverride} FF AA`;

  const row = document.createElement('div');
  const isAlert = state.isOverrideActive || Math.abs(state.barrierTorque) > 2.0;
  row.className = `can-row ${isAlert ? 'alert' : ''}`;
  row.innerHTML = `
    <span class="can-id">${target.id}</span>
    <span>${target.dlc}</span>
    <span>${payload}</span>
    <span>${target.rate} ms</span>
  `;

  canLogStream.insertBefore(row, canLogStream.firstChild);
  if (canLogStream.children.length > 5) {
    canLogStream.removeChild(canLogStream.lastChild);
  }
}

// SYSTEM CLOCK
function updateClock() {
  const now = new Date();
  const ms = String(now.getMilliseconds()).padStart(3, '0');
  systemClock.textContent = now.toTimeString().split(' ')[0] + '.' + ms;
}

// MAIN 50 Hz LOOP
setInterval(() => {
  computePhysics();
  updateUI();
  updateClock();
}, 20);

// CAN EMITTER AT 10 Hz
setInterval(emitCanFrame, 100);
