//include all mandatory external modules for code's operation 
#include <VescUart.h>
#include <Wire.h>
#include <MPU9250.h>
#include <MadgwickAHRS.h>
#include <SoftwareSerial.h>
#include <SD.h>

// MPU9250 and Madgwick filter setup.
MPU9250 mpu;
Madgwick filter;
float roll = 0.0, pitch = 0.0, yaw = 0.0;

// VESC and Uart communication setup.
VescUart vesc;
#define RX_PIN 10
#define TX_PIN 11
SoftwareSerial vescSerial(RX_PIN, TX_PIN);

// SD Card ,logfile setup
#define SD_CS 4  //declaring chip select for the arduino .
File logfile;

// Manual data flushing : Reset Button for SD file (active LOW).
#define RESET_BUTTON 7

// PID parameters
float kp = 1.2f, ki = 0.5f, kd = 0.1f;
float setpoint[3] = {0.0f, 0.0f, 0.0f}; // Roll, Pitch, Yaw (target).
float integral[3] = {0.0f, 0.0f, 0.0f}, prev_error[3] = {0.0f, 0.0f, 0.0f};
unsigned long prev_time = 0;

// Anti-windup limits for integral term
const float INTEGRAL_LIMIT = 5000.0f; // tune this to your system

// Minimum acceptable dt (seconds)
const float MIN_DT = 0.0001f; // 0.1 ms - prevents divide-by-zero and huge derivative

// VESC telemetry buffer, storing critical data of VESC and its corresponding BLDC.
struct VESCData {
  float rpm;
  float Dutycycle;
  float energy_consumed;
  float VESC_temp;
  float Motor_temp;
  float inputVoltage;
  float Motor_current;
};
VESCData vescTelemetry[3];   //Three buffers for three VESCs.

//uint32_t : 32bit(4 byte) unsigned integer
const uint32_t MAX_FILE_SIZE = 1024UL * 1024UL * 1024UL; // 1 GB for 24-hour logging.

void setup() {
  Serial.begin(115200);
  vescSerial.begin(115200);             // VESC_x and Arduino Uart connection setup.
  vesc.setSerialPort(&vescSerial);      // VESC setup.
  Wire.begin();
  if (!mpu.setup(0x68)) {
    Serial.println("MPU setup failed!");
    // proceed but IMU updates won't occur; handle as needed
  }
  delay(1000);
  filter.begin(100);                    // 100 Hz sample rate, filter setup.

  pinMode(RESET_BUTTON, INPUT_PULLUP);  // setup flush button for the logfile.

  if (!SD.begin(SD_CS)) {
    Serial.println("SD Card init failed!");
    // block here so you notice the storage issue
    while (1);
  }
  Serial.println("SD Card init successful.");

  initializeLogFile();

  // Important: initialize prev_time to current millis to avoid huge dt on first loop.
  prev_time = millis();
}

// logfile initializing funtion, call period-24 hours,can be called manually via reset button.
// deletes existing file and creates a new logfile. 
void initializeLogFile() {
  if (SD.exists("log.csv")) {
    SD.remove("log.csv");
    Serial.println("Old log file deleted.");
  }
  logfile = SD.open("log.csv", FILE_WRITE);
  if (logfile) {
    logfile.println("TIME(ms),ROLL(°),PITCH(°),YAW(°),SET_ROLL(°),SET_PITCH(°),SET_YAW(°),DUTY_X,ENERGY_X(Wh),RPM_X,VESC_X_TEMP(°C),MOTOR_X_TEMP(°C),VOLTAGE_X_IN,MOTOR_X_CURRENT,DUTY_Y,ENERGY_Y(Wh),RPM_Y,VESC_Y_TEMP(°C),MOTOR_Y_TEMP(°C),VOLTAGE_Y_IN,MOTOR_Y_CURRENT,DUTY_Z,ENERGY_Z(Wh),RPM_Z,VESC_Z_TEMP(°C),MOTOR_Z_TEMP(°C),VOLTAGE_Z_IN,MOTOR_Z_CURRENT");
    logfile.close();
  } else {
    Serial.println("Failed to create log.csv header.");
  }
}

// funtion for reading satellite attitude and filtering.
//stores the value in variables-(roll,pitch,yaw) .
void readOrientation() {
  if (mpu.update()) {
    filter.update(
      mpu.getGyroX(), mpu.getGyroY(), mpu.getGyroZ(),
      mpu.getAccX(), mpu.getAccY(), mpu.getAccZ(),
      mpu.getMagX(), mpu.getMagY(), mpu.getMagZ()
    );
    roll = filter.getRoll();
    pitch = filter.getPitch();
    yaw = filter.getYaw();
  }
}

//function for computing PID-based signal to VESC.
//formula for output control signal- "u(t) = kp * error + ki * integral + kd * derivative".
// integral and prev_error are passed by reference so they update the arrays in-place.
float computePID(float setpoint_val, float current, float &integral_ref, float &prev_error_ref, float dt) {
  if (dt < MIN_DT) dt = MIN_DT; // safety clamp
  float error = setpoint_val - current;

  // integrate with anti-windup (clamp integral)
  integral_ref += error * dt;
  if (integral_ref > INTEGRAL_LIMIT) integral_ref = INTEGRAL_LIMIT;
  else if (integral_ref < -INTEGRAL_LIMIT) integral_ref = -INTEGRAL_LIMIT;

  float derivative = (error - prev_error_ref) / dt;
  prev_error_ref = error;

  float output = kp * error + ki * integral_ref + kd * derivative;
  return output;
}

//function setting the motor RPM(via computePID funtion) corresponding to input axis.
//canID - ( VESC_X(master) = 0 , VESC_Y = 1 , VESC_Z = 2 ) .
//if axis=0, command is sent via Uart else, command is sent to master and then master forwards to VESC based on the INPUT CAN_id .
void controlAxis(int axis, float current_angle, float dt) {
  if (dt < MIN_DT) return; // safety, though computePID also handles it

  float control = computePID(setpoint[axis], current_angle, integral[axis], prev_error[axis], dt);
  int rpm = constrain((int)(control * 100.0f), -4000, 4000); // Scale and clamp to VESC rpm range.

  if (axis == 0) {
    vesc.setRPM(rpm);
  } else {
    sendCANCommand(axis, rpm);
  }
}

//funtion to send COMM_SET_RPM command to desired VESC via master over CAN.
void sendCANCommand(uint8_t id, int rpm) {
  COMM_PACKET_ID command_id = COMM_SET_RPM; // command type .
  uint8_t payload[5];
  payload[0] = id;
  int32_t rpm_val = (int32_t)rpm;
  // Big-endian packing (match your master firmware expectations)
  payload[1] = (rpm_val >> 24) & 0xFF;
  payload[2] = (rpm_val >> 16) & 0xFF;
  payload[3] = (rpm_val >> 8) & 0xFF;
  payload[4] = rpm_val & 0xFF;
  vesc.sendPacket(command_id, payload, 5);  // send command and data via CAN.
}

//funtion for retrieving the real time data from VESC and its motor over CAN and then storing it in telemetry buffer and also printing onto serial moniter. 
void requestCANValues(uint8_t can_id) {
  uint8_t payload[2];
  payload[0] = can_id;
  payload[1] = COMM_GET_VALUES;
  vesc.sendPacket(COMM_FORWARD_CAN, payload, 2);    //sending request for retrieving data over CAN ..

  delay(10); // small wait for response; increase if necessary on slow bus

  if (vesc.getVescValues()) {
    // protect index range
    if (can_id < 3) {
      vescTelemetry[can_id].rpm = vesc.data.rpm;
      vescTelemetry[can_id].Dutycycle = vesc.data.dutyCycleNow;
      vescTelemetry[can_id].energy_consumed = vesc.data.wattHours;
      vescTelemetry[can_id].VESC_temp = vesc.data.tempMosfet;
      vescTelemetry[can_id].Motor_temp = vesc.data.tempMotor;
      vescTelemetry[can_id].inputVoltage = vesc.data.inpVoltage;
      vescTelemetry[can_id].Motor_current = vesc.data.currentMotor;
    }

    char axis = (can_id == 0) ? 'X' : (can_id == 1) ? 'Y' : (can_id == 2) ? 'Z' : '?';
    Serial.print("[VESC "); Serial.print(axis); Serial.print("] RPM: "); Serial.print(vesc.data.rpm);
    Serial.print(" | Duty Cycle: "); Serial.print(vesc.data.dutyCycleNow);
    Serial.print(" | Energy consumed: "); Serial.print(vesc.data.wattHours);
    Serial.print(" | VESC Temp: "); Serial.print(vesc.data.tempMosfet);
    Serial.print(" | Motor Temp: "); Serial.print(vesc.data.tempMotor);
    Serial.print(" | Input Voltage: "); Serial.print(vesc.data.inpVoltage);
    Serial.print(" | Motor Current: "); Serial.println(vesc.data.currentMotor);
  } else {
    // No response / parse failed - it's helpful to know occasionally
    // Serial.println("No VESC values parsed for CAN id " + String(can_id));
  }
}

// funtion for logging data , storing the contents of buffer in .csv file, and flushing data.
void logData() {
  if (digitalRead(RESET_BUTTON) == LOW) {
    Serial.println("Reset button pressed — wiping log file.");
    initializeLogFile();
    delay(500);
    return;
  }

  // Safe file-size check (open read-only, check size, close)
  File f = SD.open("log.csv", FILE_READ);
  if (f) {
    uint32_t size = f.size();
    f.close();
    if (size > MAX_FILE_SIZE) {
      Serial.println("Max file size reached — auto wiping log file.");
      initializeLogFile();
      return;
    }
  }

  logfile = SD.open("log.csv", FILE_WRITE);
  if (logfile) {
    logfile.print(millis()); logfile.print(",");
    logfile.print(roll); logfile.print(",");
    logfile.print(pitch); logfile.print(",");
    logfile.print(yaw); logfile.print(",");
    logfile.print(setpoint[0]); logfile.print(",");
    logfile.print(setpoint[1]); logfile.print(",");
    logfile.print(setpoint[2]); logfile.print(",");

    for (int i = 0; i < 3; i++) {
      logfile.print(vescTelemetry[i].Dutycycle); logfile.print(",");
      logfile.print(vescTelemetry[i].energy_consumed); logfile.print(",");
      logfile.print(vescTelemetry[i].rpm); logfile.print(",");
      logfile.print(vescTelemetry[i].VESC_temp); logfile.print(",");
      logfile.print(vescTelemetry[i].Motor_temp); logfile.print(",");
      logfile.print(vescTelemetry[i].inputVoltage); logfile.print(",");
      logfile.print(vescTelemetry[i].Motor_current);
      if (i < 2) logfile.print(",");
    }
    logfile.println();
    logfile.close();
  } else {
    Serial.println("SD write failed! Possibly card full or corrupted.");
  }
}

void loop() {
  // Compute dt once per loop (consistent for all axes)
  unsigned long now = millis();
  float dt = (now - prev_time) / 1000.0f;
  if (dt <= 0.0f) {
    // if clock moved backward or prev_time uninitialized, reset prev_time and skip this loop
    prev_time = now;
    return;
  }

  // Read sensors and update orientation
  readOrientation();

  // Control each axis using the same dt
  controlAxis(0, roll, dt);   // ROLL
  controlAxis(1, pitch, dt);  // PITCH
  controlAxis(2, yaw, dt);    // YAW

  // Request telemetry (can be staggered if bus is busy)
  requestCANValues(0);
  requestCANValues(1);
  requestCANValues(2);

  // Log data
  logData();

  // Update prev_time for next loop
  prev_time = now;

  // Short delay - tune as necessary. Keep in mind dt accounts for real elapsed time.
  delay(10);
}

