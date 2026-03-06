#include <Wire.h>
#include <Servo.h>

const int MPU = 0x68; 
Servo massServo;

// --- Control Variables ---
bool enablePID = true;
bool enableWheels = false;

float Kp = 12.0, Ki = 0.0, Kd = 0.5; // Default PID
float targetAngle = -1.5; 
float pitch = 0, error, lastError, integral;
unsigned long lastTime;

void setup() {
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B); 
  Wire.write(0);    
  Wire.endTransmission(true);
  
  massServo.attach(9);
  Serial.begin(115200);
  
  Serial.println("--- 2-Wheeled Biped Tuner ---");
  Serial.println("Commands: 'p1.5' (set Kp), 'i0.1' (set Ki), 'd0.5' (set Kd)");
  Serial.println("'e1' (Enable PID), 'e0' (Disable PID)");
  Serial.println("'w1' (Enable Wheels), 'w0' (Disable Wheels)");
  
  lastTime = millis();
}

void loop() {
  float dt = (millis() - lastTime) / 1000.0;
  lastTime = millis();

  // 1. Read MPU6050 & Filter
  readIMU(dt);

  // 2. Serial Tuning Parser
  checkSerial();

  // 3. PID Calculation
  if (enablePID) {
    error = pitch - targetAngle;
    integral += error * dt;
    float derivative = (error - lastError) / dt;
    float output = (Kp * error) + (Ki * integral) + (Kd * derivative);
    lastError = error;

    // Actuate Servo
    int servoPos = constrain(90 + output, 30, 150);
    massServo.write(servoPos);

    // Actuate Wheels (Only if enabled)
    if (enableWheels) {
      driveMotors(output * 10); // Simple proportional drive
    } else {
      driveMotors(0);
    }
  }

  // 4. Print Telemetry for Plotting
  Serial.print("Pitch:"); Serial.print(pitch);
  Serial.print(" | Kp:"); Serial.print(Kp);
  Serial.print(" | PID_En:"); Serial.print(enablePID);
  Serial.print(" | Wheel_En:"); Serial.println(enableWheels);

  delay(10); 
}

void readIMU(float dt) {
  int16_t AcY, AcZ, GyX;
  Wire.beginTransmission(MPU);
  Wire.write(0x3D); // Start at Accel Y
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true);
  
  AcY = Wire.read() << 8 | Wire.read();
  AcZ = Wire.read() << 8 | Wire.read();
  GyX = Wire.read() << 8 | Wire.read();

  float accelAngle = atan2(AcY, AcZ) * 180 / PI;
  float gyroRate = GyX / 131.0;
  pitch = 0.98 * (pitch + gyroRate * dt) + 0.02 * accelAngle;
}

void checkSerial() {
  if (Serial.available() > 0) {
    char type = Serial.read();
    float val = Serial.parseFloat();

    if (type == 'p') Kp = val;
    else if (type == 'i') Ki = val;
    else if (type == 'd') Kd = val;
    else if (type == 'e') enablePID = (val > 0.5);
    else if (type == 'w') enableWheels = (val > 0.5);
  }
}

void driveMotors(float speed) {
  // Add your H-Bridge PWM logic here
}