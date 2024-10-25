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
double setpoint = -5600;  // Desired position
double currentPosition = 0;  // Encoder position
double motorOutput = 0;  // PWM output to motor

// PID tuning parameters (Kp, Ki, Kd)
double Kp = 0.01, Ki = 0.0, Kd = 0.001;
PID myPID(&currentPosition, &motorOutput, &setpoint, Kp, Ki, Kd, DIRECT);

// Encoder object
Encoder myEncoder(encoderPinA, encoderPinB);
  unsigned long last_time = millis();
  unsigned long last_time_speed = millis();
  unsigned long last_time_print = millis();	

void setup() {
  // Initialize motor pins
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(pwmPin, OUTPUT);

  // Initialize PID
  myPID.SetMode(AUTOMATIC);
  //myPID.SetOutputLimits(-40, 40);  // Set PWM limits
  myPID.SetOutputLimits(-20, 20);  // Set PWM limits


  Serial.begin(9600);  // Debugging
}

int wait_time = 10;
int wait_time_speed = 100;
int wait_time_print = 10000;
double floor_speed = 5.;
double last_pos = 0.;
double last_output;
void loop() {
  // Read the current encoder position
  currentPosition = myEncoder.read();

  unsigned long time = millis();


  if( (time - last_time_speed) > wait_time_speed){
    last_time_speed = time;
    if(last_pos == currentPosition){	
      floor_speed *= 1.05;
    }    
    if(last_output*motorOutput < 0.){
      floor_speed *= 0.5;
    }
    last_pos = currentPosition;
    last_output = motorOutput;
  }

  if( (time - last_time_print) > wait_time_print){
    last_time_print = time;
    //  Debugging output
    Serial.print("Setpoint: ");
    Serial.print(setpoint);
    Serial.print(" Current Position: ");
    Serial.print(currentPosition);
    Serial.print(" Motor Output: ");
    Serial.println(motorOutput);
    Serial.println("------------");
}  

  if(time - last_time > wait_time){
    last_time = time;


  // Run PID control  
    myPID.Compute();
    if(abs(motorOutput) < floor_speed){
      motorOutput = (abs(motorOutput)/motorOutput)*floor_speed;
    }
  
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
}


  // Add code to change the setpoint based on desired position
  // For example, you could read a new setpoint from Serial or a potentiometer.
}