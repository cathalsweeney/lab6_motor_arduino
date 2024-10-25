#include <Encoder.h>

#include <PID_v1.h>

//#define ENCODER_DO_NOT_USE_INTERRUPTS

// Motor driver pins
const int motorPin1 = 10;
const int motorPin2 = 11;
const int pwmPin = 9; // PWM pin for speed control

// Encoder pins
const int encoderPinA = 13;//7;//2
const int encoderPinB = 12;//4;//3

// Setpoint and PID parameters
double setpoint = 5; //5;//5 //-10;  // Desired position // speed?
double currentPosition = 0.0;//10;  //0; // Encoder position
double motorOutput = 0.0;//5.5; //0; // PWM output to motor
double sign = -1.0; // -1.0

// Ticks per revolution
const long ticksPerRevolution = 100*4*14; // Adjust this value based on your encoder
const long targetTicks = 2 * ticksPerRevolution; // 13 turns

// PID tuning parameters (Kp, Ki, Kd)
double Kp = 2.0, Ki = 0.0, Kd = 0.0; // Ki = 0.0, Kd = 0.0
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
  myPID.SetOutputLimits(-255, 255);  // Set PWM limits


  Serial.begin(9600);  // Debugging
}

void loop() {
  // Read the current encoder position
  currentPosition = myEncoder.read();

  // Print the current position for debugging
  //Serial.print("Current Position: ");
  //Serial.println(currentPosition);
  
  if (abs(currentPosition) < setpoint) {
    // Run PID control
    myPID.Compute();

    if (sign > 0) {
      digitalWrite(motorPin1, HIGH);
      digitalWrite(motorPin2, LOW);
      analogWrite(pwmPin, motorOutput);
    } 
    else if (sign < 0) {
      digitalWrite(motorPin1, LOW);
      digitalWrite(motorPin2, HIGH);
      analogWrite(pwmPin, motorOutput);
    } 
    else {
      digitalWrite(motorPin1, LOW);
      digitalWrite(motorPin2, LOW);
    }
  }
  else {
    // Stop the motor once the desired number of turns is reached
    digitalWrite(motorPin1, LOW);
    digitalWrite(motorPin2, LOW);
    analogWrite(pwmPin, 0);
  }
  if (currentPosition >= targetTicks) {
      myEncoder.write(0); // Reset encoder position
  }
  // Debugging output
  
  Serial.print("Setpoint: ");
  Serial.print(setpoint);
  Serial.print(" Current Position: ");
  Serial.print(currentPosition);
  Serial.print(" Target Ticks: ");
  Serial.println(targetTicks);
  Serial.print(" Motor Output: ");
  Serial.println(motorOutput);
  
  // Add code to change the setpoint based on desired position
  // For example, you could read a new setpoint from Serial or a potentiometer.
  //delay(500);  // Delay for readability
}