
# Reaction Wheel-Based CubeSat Attitude Control System

This project implements a **PID-based reaction wheel system** to stabilize the attitude of a CubeSat in space using three BLDC motors driven by VESCs. The system operates autonomously, with real-time data logging up to 24 hours, and includes safety features for thermal and electrical protection. This forms a critical part of satellite attitude control systems used in space missions.

---

## Working Principle & Features

1. **PID Control for Attitude Stabilization** &#x20;
   The system uses real-time PID-based control to stabilize a CubeSat's attitude to a predefined setpoint using three orthogonal reaction wheels (X, Y, Z axes).

2. **Triple BLDC Motor Control** &#x20;
   Each reaction wheel (BLDC motor) is controlled by an individual VESC (Electronic Speed Controller), all interfaced with a central Arduino Nano. The control logic is axis-specific:

   * X-axis: Direct UART connection to Arduino (MASTER)
   * Y, Z axes: Controlled via CAN bus communication from the master VESC

3. **Sensor Fusion Using MPU9250 + Madgwick Filter** &#x20;
   The onboard MPU9250 IMU sensor sends real-time orientation data over I2C to the Arduino Nano. Sensor fusion is performed using the Madgwick algorithm to obtain stable attitude feedback, which is compared against the target setpoint to generate PID corrections.

4. **System Behavior Under Different Conditions**

   * If the CubeSat is misaligned, PID logic drives the respective BLDC motor until the desired setpoint is reached.
   * If the motor temperature exceeds safety thresholds, the VESC automatically slows down or cuts power to prevent damage.
   * The system dynamically adjusts motor RPM in all three axes based on sensor feedback to ensure real-time orientation stability.
   * In the event of communication failure with a secondary VESC (Y or Z), the system continues operating using the X-axis (master) while logging the fault for diagnosis.

5. **VESC Configuration for Safety** &#x20;
   VESCs are configured with:

   * **Temperature Cutoff:**

     * Warning: 80°C
     * Cutoff Start: 80°C
     * Cutoff End: 100°C
   * **RPM, Voltage & Current Limits:** Predefined for safe operation
   * **Auto slowdown & shutdown:** For thermal protection

6. **CAN Communication Between VESCs**

   * VESC 0 (Master): UART enabled, CAN ID = 0, termination resistor enabled
   * VESC 1: UART disabled, CAN ID = 1, termination resistor disabled
   * VESC 2: UART disabled, CAN ID = 2, termination resistor enabled
   * All VESCs must have the following options enabled in VESC Tool:

     * Send CAN Status
     * Multiple VESCs over CAN
     * CAN Forwarding

7. **Real-Time Data Logging** &#x20;
   Data from PID loops, orientation readings, motor RPM, and temperature are logged on an SD card using a MicroSD module via SPI. This enables analysis of system performance over 24 hours.

8. **Manual CSV Reset Feature** &#x20;
   A dedicated push-button on the PCB is implemented to allow manual erasure of the current `.csv` file on the SD card and creation of a new log file. This is useful for resetting data logs between different test sessions without reprogramming or removing the SD card.

---

## Components Used

### Microcontroller

* **Arduino Nano** &#x20;
  Central controller executing PID logic and interfacing with IMU, SD module, and VESCs.

### Sensors

* **MPU9250 (x1)** &#x20;
  9-axis IMU for real-time orientation sensing over I2C.

* **Hall Sensors (x3)** &#x20;
  Built into BLDC motors and connected to VESCs for RPM feedback.

* **NTC 10K Thermistors (x3)** &#x20;
  Glued to BLDC motors and connected to VESCs for temperature monitoring.

### Actuators

* **NEMA17 BLDC Motors (x3)** &#x20;
  Sensored brushless motors used as reaction wheels.

* **VESC (x3)** &#x20;
  Open-source motor controllers, configured via VESC Tool.

### Storage

* **Catalex MicroSD Card Module** &#x20;
  SPI-based SD card module for logging operational data.

---

## How to Setup

Each VESC must be connected to a computer via USB to configure it using the VESC Tool software.

1. **VESC Configuration (via VESC Tool)**

   * Enable:

     * Send CAN Status
     * Multiple VESCs over CAN
     * CAN Forwarding
   * Set CAN IDs:

     * VESC 0: CAN ID = 0, UART = Enabled, Termination Resistor = Enabled
     * VESC 1: CAN ID = 1, UART = Disabled, Termination Resistor = Disabled
     * VESC 2: CAN ID = 2, UART = Disabled, Termination Resistor = Enabled
   * Set Temperature Cutoffs:

     * Warning Temp: 80°C
     * Cutoff Start: 80°C
     * Cutoff End: 100°C

2. **Arduino Setup**

   * Upload the code to Arduino Nano
   * Connect MPU9250 to I2C (A4/A5)
   * Connect SD Module to SPI (D10-D13)
   * Use Software Serial (D8, D9) to connect to VESC UART (x-axis)
   * Ensure CAN communication lines are properly wired between VESC 0 (master), VESC 1, and VESC 2 according to CAN protocol. Use twisted pair wiring for CANH and CANL with proper termination resistors as configured in VESC Tool.

---

## Repository Structure 

```bash
Reaction-Wheel-Controller/
├── src_Task_3/
│   └── src_Task_3.ino                        # Arduino control code
├── README.md                                 # Project documentation
├── hardware_Task_1&2/
│   ├── kicad files opening instructions.pdf  # steps to get kicad_sch & kicad_pcb & kicad_pro files
│   ├── Reaction Wheels Design.step           # CAD model for simulation
│   └── Task-2  .pro , .sch , _pcb files.zip  # zip folder of kicad files
├── Power_Task2_doc/
│   └── Explanation pdf.pdf                   # Power circuit explanation
├── challenges/ 
│   └── challenges.txt                        # Design/implementation difficulties
└── docs & screenshots/
    └── Circuit& CAD screenshots/             # image folders and pdf related to project
    └── VESC_TOOL_Screenshots/
    └── README.pdf                                   
```
