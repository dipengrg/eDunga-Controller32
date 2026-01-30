// Program to control servo motor using ESP32 v3.3.1 and BTS7960 driver with potentiometer feedback
#include <Arduino.h>

// PPM & Potentiometer
const int ppmPin = 22;
const int potPin = 34;

// BTS7960 motor driver pins
const int RPWM = 18; // Right direction PWM
const int LPWM = 19; // Left direction PWM

// PWM / Motor settings
const int pwmFreq = 20000; // 20 kHz (inaudible)
const int pwmResolution = 8; // 0-255
const int pwmMin = 46; // minimum PWM to overcome dead zone
const int pwmMaxLimit = 191; // Limits max voltage to ~9V (9/12 * 255)

// PID settings
float Kp = 4.0; // Proportional gain
float Ki = 0.02; // Integral gain
float Kd = 0.3; // Derivative gain

float targetPosition = 0;
float actualPosition = 0;
float lastError = 0;
float integral = 0;

const float dt = 0.02; // 20ms loop
const int potMinLimit = 10;
const int potMaxLimit = 4080;

unsigned long lastTime = 0;

void setup() {
  Serial.begin(115200);
  pinMode(ppmPin, INPUT);
  pinMode(potPin, INPUT);

  ledcAttach(RPWM, pwmFreq, pwmResolution);
  ledcAttach(LPWM, pwmFreq, pwmResolution);

  // Motor off initially
  ledcWrite(RPWM, 0);
  ledcWrite(LPWM, 0);
}

void loop() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n'); // format: "Kp,Ki,Kd"
    sscanf(input.c_str(), "%f,%f,%f", &Kp, &Ki, &Kd);
    Serial.print("Updated PID: ");
    Serial.print(Kp); Serial.print(", ");
    Serial.print(Ki); Serial.print(", ");
    Serial.println(Kd);
  }

  if (millis() - lastTime >= 20) {
    lastTime = millis();

    // 1. Read PPM input -- //
    unsigned long pulseWidth = pulseIn(ppmPin, HIGH, 30000);

    // If the input is in the required range
    if (pulseWidth > 900 && pulseWidth < 2100) {
      targetPosition = map(pulseWidth, 1000, 2000, -100, 100);
    } else {
      // In case of signal missing
      targetPosition = 0; // failsafe center
    }

    // 2. Read potentiometer feedback -- //
    int potValue = analogRead(potPin);

    // Map the value 12 bit value (0 ~ 4095 in the scale of -100 ~ 100)
    actualPosition = map(potValue, 0, 4095, -100, 100);

    // Make the input smooth
    static float filteredTarget = 0;
    static float filteredActual = 0;
    filteredTarget = 0.90 * filteredTarget + 0.10 * targetPosition;
    filteredActual = 0.90 * filteredActual + 0.10 * actualPosition;
    targetPosition = filteredTarget;
    actualPosition = filteredActual;

    // 3. PID control
    float error = targetPosition - actualPosition;

    // Deadbanding
    if (abs(error) < 1) {
      error = 0;
      integral = 0;
    }

    // If the potentiometer reading is not in expected range
    if (potValue < potMinLimit || potValue > potMaxLimit) {
      // STOP THE MOTOR!
      ledcWrite(RPWM, 0);
      ledcWrite(LPWM, 0);
      integral = 0;
      lastError = error;
      return;
    }

    // --- PID computation ---
    integral += error * dt;
    integral = constrain(integral, -100, 100);
    float derivative = (error - lastError) / dt;
    float output = Kp * error + Ki * integral + Kd * derivative;
    lastError = error;

    // --- PWM scaling ---
    // Map 'output' (0-100) to raw PWM (0-255)
    int mappedPWM = map(abs(output), 0, 100, 0, 255); 
    int pwmValue = 0;
    if (mappedPWM > 0) {
      // If the calculated PWM is non-zero, apply the minimum necessary to move
      pwmValue = constrain(mappedPWM, pwmMin, pwmMaxLimit); 
    }

    Serial.print("target:");
    Serial.print(abs(targetPosition));
    Serial.print(" actual:");
    Serial.print(abs(actualPosition));
    Serial.print(" error:");
    Serial.print(abs(error));
    Serial.print(" pwm:");
    Serial.print(abs(pwmValue));
    Serial.println();

    // 4. Motor control
    if (output > 0) {
      ledcWrite(RPWM, 0);
      ledcWrite(LPWM, pwmValue);
    } else if (output < 0) {
      ledcWrite(RPWM, pwmValue);
      ledcWrite(LPWM, 0);
    } else {
      ledcWrite(RPWM, 0);
      ledcWrite(LPWM, 0);
    }
  }
}
