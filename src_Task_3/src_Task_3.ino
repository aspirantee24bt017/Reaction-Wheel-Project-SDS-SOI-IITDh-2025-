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
float roll, pitch, yaw;

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
float kp = 1.2, ki = 0.5, kd = 0.1;
float setpoint[3] = {0.0, 0.0, 0.0}; // Roll, Pitch, Yaw (target).
float integral[3] = {0, 0, 0}, prev_error[3] = {0, 0, 0};
unsigned long prev_time = 0;

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
  mpu.setup(0x68);                      // IMU sensor setup.
  delay(1000);
  filter.begin(100);                    // 100 Hz sample rate,filter setup.

  pinMode(RESET_BUTTON, INPUT_PULLUP);  // setup flush button for the logfile.

  if (!SD.begin(SD_CS)) {
    Serial.println("SD Card init failed!");
    while (1);
  }
  Serial.println("SD Card init successful.");

  initializeLogFile();
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
//function for computing PID-based signal to VESC .
//formula for output control signal- "u(t) = kp * error + ki * integral + kd * derivative".
float computePID(float setpoint, float current, float integral, float prev_error, float dt) {
  float error = setpoint - current;
  integral += error * dt;
  float derivative = (error - prev_error) / dt;
  prev_error = error;
  return kp * error + ki * integral + kd * derivative;
}


//function setting the motor RPM(via computePID funtion) corresponding to input axis.
//canID - ( VESC_X(master) = 0 , VESC_Y = 1 , VESC_Z = 2 ) .
//if axis=0, command is sent via Uart else, command is sent to master and then master forwards to VESC based on the INPUT CAN_id .
void controlAxis(int axis, float current_angle) {
  unsigned long now = millis();
  float dt = (now - prev_time) / 1000.0;
  if (dt <= 0) return;
  float control = computePID(setpoint[axis], current_angle, integral[axis], prev_error[axis], dt);
  int rpm = constrain(control * 100, -4000, 4000); // Scale and clamp to VESC rpm range.

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
  int32_t rpm_val = rpm;
  payload[1] = (rpm_val >> 24) & 0xFF;      // rpm_val is split into an array of one signed byte.
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

  delay(10);

  if (vesc.getVescValues()) {
    vescTelemetry[can_id].rpm = vesc.data.rpm;
    vescTelemetry[can_id].Dutycycle = vesc.data.dutyCycleNow;
    vescTelemetry[can_id].energy_consumed = vesc.data.wattHours;
    vescTelemetry[can_id].VESC_temp = vesc.data.tempMosfet;
    vescTelemetry[can_id].Motor_temp = vesc.data.tempMotor;
    vescTelemetry[can_id].inputVoltage = vesc.data.inpVoltage;
    vescTelemetry[can_id].Motor_current = vesc.data.currentMotor;

    char axis = (can_id == 0) ? 'X' : (can_id == 1) ? 'Y' : (can_id == 2) ? 'Z' : '?';
    Serial.print("[VESC "); Serial.print(axis); Serial.print("] RPM: "); Serial.print(vesc.data.rpm);
    Serial.print(" | Duty Cycle: "); Serial.print(vesc.data.dutyCycleNow);
    Serial.print(" | Energy consumed: "); Serial.print(vesc.data.wattHours);
    Serial.print(" | VESC Temp: "); Serial.print(vesc.data.tempMosfet);
    Serial.print(" | Motor Temp: "); Serial.print(vesc.data.tempMotor);
    Serial.print(" | Input Voltage: "); Serial.print(vesc.data.inpVoltage);
    Serial.print(" | Motor Current: "); Serial.println(vesc.data.currentMotor);
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

  if (SD.exists("log.csv") && SD.open("log.csv").size() > MAX_FILE_SIZE) {
    Serial.println("Max file size reached — auto wiping log file.");
    initializeLogFile();
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
    Serial.println(" SD write failed! Possibly card full or corrupted.");
  }
}

void loop() {
  readOrientation();            // read current satellite attitude.

  controlAxis(0, roll);         // update RPM of motor_x to control 'ROLL' parameter with respect to setpoint.
  controlAxis(1, pitch);        // update RPM of motor_y to control 'PITCH' parameter with respect to setpoint.
  controlAxis(2, yaw);          // update RPM of motor_z to control 'YAW' parameter with respect to setpoint.

  requestCANValues(0);          // retriving real time data from VESC_X (CANid=0).
  requestCANValues(1);          // retriving real time data from VESC_y (CANid=1).
  requestCANValues(2);          // retriving real time data from VESC_z (CANid=2).

  logData();                    // storing the retireved data into log file.
  prev_time=millis();
  delay(10);                    // delay time for stabalizing control signals.
}



