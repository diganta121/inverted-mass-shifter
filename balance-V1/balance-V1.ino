#include <Wire.h>
#include <Servo.h>

#define MPU_ADDR 0x68
Servo massServo;

// --- Config & Flags ---
bool enablePID = true;
bool enableWheels = false;
bool showDebug = false; // CMD 's' to toggle serial printing

float Kp = 12.0, Ki = 0.0, Kd = 0.5;
float pitch = 0, targetAngle = -1.5;
float lastError = 0, integral = 0;
unsigned long lastMicros;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C Fast Mode
  
  // Wake up MPU6050
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission();

  massServo.attach(9);
  lastMicros = micros();
}

void loop() {
  // 1. Calculate precise Delta Time
  unsigned long currentMicros = micros();
  float dt = (currentMicros - lastMicros) / 1000000.0;
  lastMicros = currentMicros;

  // 2. High-Speed IMU Read (Direct Register Access)
  readIMU_Fast(dt);

  // 3. Non-Blocking Serial Parser
  if (Serial.available()) handleSerial();

  // 4. PID Controller
  if (enablePID) {
    float error = pitch - targetAngle;
    integral += error * dt;
    float derivative = (error - lastError) / dt;
    float output = (Kp * error) + (Ki * integral) + (Kd * derivative);
    lastError = error;

    massServo.write(constrain(90 + (int)output, 30, 150));
    if (enableWheels) driveMotors(output * 10);
  }

  // 5. Conditional Telemetry (The "Silent" Mode)
  if (showDebug) {
    // Only prints every 50ms to save CPU
    static unsigned long lastPrint;
    if (millis() - lastPrint > 50) {
      Serial.print(pitch); Serial.print(",");
      Serial.print(Kp); Serial.print(",");
      Serial.println(enableWheels);
      lastPrint = millis();
    }
  }
}

void readIMU_Fast(float dt) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3D); // Start at Accel_Y
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6, true);

  int16_t ay = Wire.read() << 8 | Wire.read();
  int16_t az = Wire.read() << 8 | Wire.read();
  int16_t gx = Wire.read() << 8 | Wire.read();

  float accelAngle = atan2(ay, az) * 57.295; // 180/PI
  float gyroRate = gx / 131.0;
  // Alpha 0.99 for higher stability during walking vibrations
  pitch = 0.99 * (pitch + gyroRate * dt) + 0.01 * accelAngle;
}

void handleSerial() {
  char cmd = Serial.read();
  if (cmd == 's') showDebug = !showDebug;
  if (cmd == 'e') enablePID = !enablePID;
  if (cmd == 'w') enableWheels = !enableWheels;
  if (cmd == 'p') Kp = Serial.parseFloat(); // Still used for tuning, but doesn't run every loop
}

void driveMotors(float speed) { /* PWM Logic */ }