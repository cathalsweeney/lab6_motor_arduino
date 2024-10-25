#include <Encoder.h>

#include <PID_v1.h>

// Motor driver pins
const int motorPin1 = 11;
const int motorPin2 = 10;
const int pwmPin = 9; // PWM pin for speed control

// Encoder pins
const int encoderPinA = 3;
const int encoderPinB = 4;

// Setpoint and PID parameters
double setpoint = -1000;  // Desired position
double currentPosition = 0;  // Encoder position
double motorOutput = 0;  // PWM output to motor

// PID tuning parameters (Kp, Ki, Kd)
double Kp = 6.0, Ki = 0.0, Kd = 0.002;
PID myPID(&currentPosition, &motorOutput, &setpoint, Kp, Ki, Kd, DIRECT);

// Encoder object
Encoder myEncoder(encoderPinA, encoderPinB);

void setup() {
  // Initialize motor pins
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(pwmPin, OUTPUT);

  // Initialize PID
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-8, 8);  // Set PWM limits


  Serial.begin(9600);  // Debugging
}

int counter = 0;
void loop() {
  counter += 1;
  // Read the current encoder position
  currentPosition = myEncoder.read();

  // Run PID control
  myPID.Compute();

  // Control motor based on PID output
  if (motorOutput > 0) {
    digitalWrite(motorPin1, HIGH);
    digitalWrite(motorPin2, LOW);
    analogWrite(pwmPin, motorOutput);
  } else if (motorOutput < 0) {
    digitalWrite(motorPin1, LOW);
    digitalWrite(motorPin2, HIGH);
    analogWrite(pwmPin, -motorOutput);
  } else {
    // Stop motor if output is zero
    digitalWrite(motorPin1, LOW);
    digitalWrite(motorPin2, LOW);
  }

  if(counter%100000 == 0){
  // Debugging output
  Serial.print("Setpoint: ");
  Serial.print(setpoint);
  Serial.print(" Current Position: ");
  Serial.print(currentPosition);
  Serial.print(" Motor Output: ");
  Serial.println(motorOutput);
  Serial.println("------------");
}

  // Add code to change the setpoint based on desired position
  // For example, you could read a new setpoint from Serial or a potentiometer.
}
