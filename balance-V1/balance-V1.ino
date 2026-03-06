#include <Wire.h>
#include <Servo.h>

#define MPU_ADDR 0x68
Servo massServo;

// --- Config Flags ---
bool enablePID = true, enableWheels = false, showDebug = false;
int8_t axisSelect = 1; // 1:X, 2:Y, 3:Z
int8_t invertAxis = 1; // 1 or -1

// --- PID & IMU Vars ---
float Kp = 15.0, Ki = 0.1, Kd = 0.6;
float pitch = 0, targetAngle = 0, offset = 0;
float lastError = 0, integral = 0;
unsigned long lastMicros;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000); 
  
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); Wire.write(0); 
  Wire.endTransmission();

  massServo.attach(9);
  lastMicros = micros();
  Serial.println("System Ready. Commands: 'c' Calibrate, 'a[1-3]' Axis, 'v' Invert, 's' Toggle Serial");
}

void loop() {
  unsigned long currentMicros = micros();
  float dt = (currentMicros - lastMicros) / 1000000.0;
  lastMicros = currentMicros;

  readIMU_Fast(dt);

  if (Serial.available()) handleSerial();

  if (enablePID) {
    float error = (pitch - targetAngle);
    integral += error * dt;
    float derivative = (error - lastError) / dt;
    float output = (Kp * error) + (Ki * integral) + (Kd * derivative);
    lastError = error;

    massServo.write(constrain(90 + (int)output, 0, 180));
    if (enableWheels) driveMotors(output * 10);
  }

  if (showDebug) {
    static unsigned long lastPrint;
    if (millis() - lastPrint > 40) {
      Serial.print("P:"); Serial.print(pitch);
      Serial.print(" Ax:"); Serial.print(axisSelect);
      Serial.print(" Inv:"); Serial.println(invertAxis);
      lastPrint = millis();
    }
  }
}

void readIMU_Fast(float dt) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); // Start at Accel_X
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true); // Read all axes

  int16_t data[7]; // ax, ay, az, temp, gx, gy, gz
  for(int i=0; i<7; i++) data[i] = Wire.read() << 8 | Wire.read();

  // Select axis based on user input
  float rawAccel, rawGyro;
  if(axisSelect == 1) { rawAccel = atan2(data[1], data[2]); rawGyro = data[4]/131.0; }
  else if(axisSelect == 2) { rawAccel = atan2(data[0], data[2]); rawGyro = data[5]/131.0; }
  else { rawAccel = atan2(data[0], data[1]); rawGyro = data[6]/131.0; }

  float accelAngle = (rawAccel * 57.295) - offset;
  pitch = 0.99 * (pitch + (rawGyro * invertAxis) * dt) + 0.01 * accelAngle;
}

void calibrateIMU() {
  Serial.println("Calibrating... Keep robot still.");
  float sum = 0;
  for(int i=0; i<200; i++) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B); Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 6, true);
    int16_t ax = Wire.read()<<8|Wire.read();
    int16_t ay = Wire.read()<<8|Wire.read();
    int16_t az = Wire.read()<<8|Wire.read();
    sum += (axisSelect == 1) ? atan2(ay, az)*57.295 : atan2(ax, az)*57.295;
    delay(5);
  }
  offset = sum / 200.0;
  pitch = 0; // Reset integration
  Serial.print("New Offset: "); Serial.println(offset);
}

void handleSerial() {
  char cmd = Serial.read();
  switch(cmd) {
    case 'c': calibrateIMU(); break;
    case 's': showDebug = !showDebug; break;
    case 'v': invertAxis *= -1; break;
    case 'a': axisSelect = Serial.parseInt(); break;
    case 'p': Kp = Serial.parseFloat(); break;
    case 'e': enablePID = !enablePID; break;
    case 'w': enableWheels = !enableWheels; break;
  }
}

void driveMotors(float speed) { /* Motor Logic */ }