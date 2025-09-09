#include <Servo.h>

#define TRIG_PIN 8
#define ECHO_PIN 9
#define SERVO_PIN 4

Servo myServo;

int servoBaseAngle = 93;
unsigned long lastTime = 0;
float elapsedTime;

float setpoint = 16.0;
float currentPosition = 0;
float error, previousError = 0;
float integral = 0;
float derivative;
float previousPosition = 0;

float Kp = 1.8;
float Ki = 0.006;
float Kd = 0.02;

void setup() {
  
  Serial.begin(9600);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  myServo.attach(SERVO_PIN);  
  myServo.write(30); 
  delay(1000);
}

void loop() {

  // Read distance
  currentPosition = readDistance_cm();

  // Reject junk readings
  if (currentPosition > 30 || currentPosition < 2) {
    currentPosition = previousPosition;
  } else {
    previousPosition = currentPosition;
  }


  // PID Calculation 
  error = setpoint - currentPosition;

  if (millis() < 1000 || abs(error) > 20) {
    integral = 0;
  }

  unsigned long currentTime = millis();
  elapsedTime = (currentTime - lastTime) / 1000.0;  
  lastTime = currentTime;

  integral += error * elapsedTime;
  integral = constrain(integral, -300, 300);  // clamp the accumulated value
  if (abs(error) < 0.2) integral = 0;
  derivative = (error - previousError) / elapsedTime;
  previousError = error;

  float output = Kp * error + Ki * integral + Kd * derivative;
  if (abs(error) < 0.5 && abs(output) < 1.0) {
    output = 0;
  }
  output = constrain(output, -30, 30);  // limits servo swing


  // Apply correction
  int servoAngle = servoBaseAngle + output;

  servoAngle = constrain(servoAngle, 30, 150);

  myServo.write(servoAngle);

  // Debug info
  Serial.print("Ball: ");
  Serial.print(currentPosition);
  Serial.print(" cm | Error: ");
  Serial.print(error);
  Serial.print(" | Servo: ");
  Serial.println(servoAngle);

  Serial.print(error);
  Serial.print(",");
  Serial.print(output);
  Serial.print(",");
  Serial.println(currentPosition);

  

  delay(30);
  
}

float readDistance_cm() {

  // Send a 10 microseconds pulse to trigger the ultrasonic burst
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Measure the time it takes for echo to return
  long duration = pulseIn(ECHO_PIN, HIGH);

  // Calculate distance in cm
  float distance_cm = duration * 0.0343/2;

  return distance_cm;

}

