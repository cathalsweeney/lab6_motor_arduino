#define ENCODER_USE_INTERRUPTS

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
double setpoint = -56000;  // Desired position
double currentPosition = 0;  // Encoder position
double last_pos = 0;

bool forward = true;

// Encoder object
Encoder myEncoder(encoderPinA, encoderPinB);

unsigned long last_time = millis();
unsigned long last_time_home = millis();
unsigned long last_time_print = millis();	

void setup() {
  // Initialize motor pins
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(pwmPin, OUTPUT);

  if(currentPosition > setpoint){
    forward = false;
  }

  last_pos = currentPosition;

  Serial.begin(9600);  // Debugging
}

int wait_time = 1;
int wait_time_print = 1000;
int wait_time_home = 1000;
double base_speed = 20.;
int speed = base_speed;
double last_output;
int nHome = 0;
int nSame = 0;
bool flip = false;
int counter = 0;
int floor_speed = 2;
int nRun = 0;

void loop() {
  // Read the current encoder position
  currentPosition = myEncoder.read();

  unsigned long time = millis();
  counter++;

//  if( (time - last_time_print) > wait_time_print){
//    last_time_print = time;
//  }  

  if(nHome < 500){
    if( (time - last_time > wait_time)  ){
      last_time = time;
  
      if(flip){
        wait_time = 1;
      }

      if(currentPosition == last_pos){
        nSame++;
      }
      else{
	nSame = 0;
      }
      last_pos = currentPosition;
      if(nSame > 1000){ // if we haven't moved in 1000 function calls, increase the speed
       speed *= 2.0;
       nSame = 0;
      }


      flip = false;
      if( ( (currentPosition < setpoint) && !forward) ||
          ( (currentPosition > setpoint) && forward) ){
  	forward = !forward;
  	nHome = 0;
  	flip = true;
  	speed /= 1.03;
 	if(speed < floor_speed){
	  speed = floor_speed;
        } 
        wait_time = 100;
      }
      else if(currentPosition == setpoint){
        nHome++;
      }
      else{
        nHome = 0;
      }
  
      if(flip){
        digitalWrite(motorPin1, LOW);
        digitalWrite(motorPin2, LOW);
      }
      else if(nHome > 499){
        digitalWrite(motorPin1, LOW);
        digitalWrite(motorPin2, LOW);
        last_time_home = time;
      }
      else if (forward) {
        digitalWrite(motorPin1, HIGH);
        digitalWrite(motorPin2, LOW);
        analogWrite(pwmPin, speed);
      }
      else if (!forward) {
        digitalWrite(motorPin1, LOW);
        digitalWrite(motorPin2, HIGH);
        analogWrite(pwmPin, speed);
      } 
    }
  } //end if(nHome < 500)
  else if( (time - last_time_home) > wait_time_home){
    reset();
    nRun++;
    Serial.print("nRun: ");
    Serial.println(nRun);
  }
  // Add code to change the setpoint based on desired position
  // For example, you could read a new setpoint from Serial or a potentiometer.
}

void reset() {
  last_time_home = 0;
  myEncoder.write(0);
  nHome = 0;
  speed = base_speed;
  setpoint *= -1;
}

void print() {
  //  Debugging output
  Serial.print("Setpoint: ");
  Serial.println(setpoint);
  Serial.print(" Current Position: ");
  Serial.println(currentPosition);
  Serial.print(" Counter: ");
  Serial.println(counter);
  Serial.print(" nHome: ");
  Serial.println(nHome);
  Serial.print(" Speed: ");
  Serial.println(speed);
  Serial.print(" nSame: ");
  Serial.println(nSame);
  Serial.println("------------");

}