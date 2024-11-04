#define ENCODER_USE_INTERRUPTS

// Need custom version of Encoder library https://forum.arduino.cc/t/uno-r4-wifi-error-in-encoder-library/1215256/10
#include <Encoder.h>


// ---- All the parameters a user might want to change are in this block ---- 
bool move_leftright = true; // if false we will move up/down
double setpoint = -56000; // Desired position.
                          // (+ve) = towards us / up
                          // (-ve) = towards wall / down
int wait_time_base = 1; // Wait time in ms before we try update the motor
int wait_time_flip = 100; // Wait time in ms before we try update the motor after overshooting
int wait_time_print = 1000; // Wait time in ms before printing again
int wait_time_home = 1000; // Once we reach our destination, how long do we wait before resetting and moving again
double base_speed = 30.; // default motor speed
double floor_speed = 2.; // minimum speed of motor
int home_limit = 500; // if we have been at the destination for this many checks, we are done
bool debug = false; // should be false for normal operation, otherwise operation will be degraded
// --------------------------------------------------------------------------

int wait_time = wait_time_base;
double speed = base_speed; // FYI technically this gets cast to an int when we write to a pwm pin
double currentPosition = 0;  // Encoder position
double last_pos = 0; // What was the encoder position last time we checked
bool forward = true; // motor direction. true = forward, false = backward
bool flip = false; // if the motor has just overshot, this will be true for 100ms

int nHome = 0; // number of consecutive times we have checked and the motor is at the destination
int nSame = 0; // number of consecutive times we have checked and the motor is at the same location (not destination)
int nRun = 0; // how many times have we reset and started again
int counter = 0; // number of times loop() has executed. just for debugging


// Which pin you use for what matters, don't change without doing some research.
// Intialise with pins for left/right movement
int motorPin1 = 13;
int motorPin2 = 12;
int pwmPin = 11; 
int encoderPinA = 6;
int encoderPinB = 7;


// Initialise all these times to current time as placeholder
unsigned long last_time = millis(); // timestamp of time we most recently checked on the motor
unsigned long last_time_home = millis(); // timestamp when we reached our destination ("home") 
unsigned long last_time_print = millis(); // timestamp when we last printed out values. Warning - printing degrades encoder performance, only do it for debugging	

Encoder myEncoder; // This guy will keep track of the motor position

void setup() {

  // switch to pins for moving up/down
  if(!move_leftright){
    motorPin1 = 8;
    motorPin2 = 10;
    pwmPin = 9; 
    encoderPinA = 3;
    encoderPinB = 4;
  }

  myEncoder.begin(encoderPinA, encoderPinB);

  // Initialize motor pins
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(pwmPin, OUTPUT);

  if(currentPosition > setpoint){
    forward = false;
  }

  last_pos = currentPosition;

  Serial.begin(9600);  // for printouts
  if(debug){
    Serial.println("********************************");
    Serial.println("******    WARNING!!!!    *******");
    Serial.println("********************************");
    Serial.println("Debug mode will severerely degrade encoder performance");
  }
  
}

void loop() {
  // Read the current encoder position
  currentPosition = myEncoder.read();

  unsigned long time = millis();
  counter++;

  if( debug && (time - last_time_print) > wait_time_print){
    last_time_print = time;
    print();
  }  

  if(nHome < home_limit){
    if( (time - last_time > wait_time)  ){ 
      last_time = time;
      
      if(flip){
        wait_time = wait_time_base; // go back to default wait_time
      }

      if(currentPosition == last_pos){ // motor might be stuck
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
      if(currentPosition == setpoint){
        nHome++;
      }
      else if( ( (currentPosition < setpoint) && !forward) || // If we have overshot
	       ( (currentPosition > setpoint) && forward) ){
  	forward = !forward;
  	nHome = 0;
  	flip = true;
  	speed /= 1.03; 
        wait_time = wait_time_flip; // temporarily increase wait time, so the motor can finish drifting
      }

      if(speed < floor_speed){
	speed = floor_speed;
      } 
      else if(speed > base_speed){
	speed = base_speed;
      }
      
      if(flip){ // we have just overshot, let's stop moving temporarily
        digitalWrite(motorPin1, LOW);
        digitalWrite(motorPin2, LOW);
      }
      else if(nHome >= (home_limit-1)){ // we are done, stop moving the motor
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
    }  // end if( (time - last_time > wait_time) )
  } //end if(nHome < 500)
  else if( (time - last_time_home) > wait_time_home){ // after some amount of time, start again. Useful if you want to move the motor back and forward a bunch
    reset();
    nRun++;
    Serial.print("nRun: ");
    Serial.println(nRun);
  }
}

void reset() {
  last_time_home = 0;
  myEncoder.write(0); // reset the current position to zero
  nHome = 0;
  speed = base_speed;
  setpoint *= -1; // reverse the direction
}

void print() {
  // Warning: do not print out too often, it will
  // degrade performance of encoder. Every 1s is too often!
  //  Debugging output
  Serial.println("********************************");
  Serial.println("******    WARNING!!!!    *******");
  Serial.println("********************************");
  Serial.println("Debug mode will severerely degrade encoder performance");
  Serial.println("");
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
  Serial.print(" Forward: ");
  Serial.println(forward);
  Serial.println("------------");

}
