#include <StateMachine.h>
#include <ADCTouch.h>
#include <MPU6050_tockn.h>
#include <math.h>

#include<Wire.h>

// [-1121,-1120] --> [0,18]  [-67,-67] --> [0,1] [3649,3650] --> [16364,16386] [91,91] --> [0,1] [12,12] --> [0,1] [-28,-27] --> [-1,2]


const int STATE_DELAY = 500;

MPU6050 mpu6050(Wire);


long timer = 0;
int mean, old_mean;

int ref0, ref1;
static int value_off = 0;
bool flag_first_time = true;

const int soundPin = A0;
int big_sound_th = 90;

StateMachine machine = StateMachine();


State* calibration = machine.addState(&state_calibration); //state 0
State* still_neutral = machine.addState(&state_still_neutral); //state 1
State* shoulder = machine.addState(&state_shoulder); //state 2
State* fear = machine.addState(&state_fear); //state 3


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Wire.begin();

  pinMode(soundPin,INPUT);  
  
  //mpu6050.begin();
  //mpu6050.calcGyroOffsets(true);

  ref1 = ADCTouch.read(A1, 500);    //account for the capacitance of the pad
  
  calibration->addTransition(&transition_calib_shoulder, shoulder);
  calibration->addTransition(&transition_calib_still_neutral, still_neutral);

  shoulder->addTransition(&transition_shoulder_still, calibration);
  shoulder->addTransition(&transition_shoulder_big_noise, fear);
  
  still_neutral->addTransition(&transition_still_calib, calibration);

}

void getMeanProximity(){

  int count = 0;
  int total_value = 0;
  int value0, value1;

  mean = 0;
  
  while(count < 30){
  
    value1 = ADCTouch.read(A1);   //   --> 100 samples
    
    total_value = total_value + value1;
  
    count++;
    
    delay(30);
  }
  
  mean = total_value / count;

  Serial.print("MEAN: ");Serial.print(mean);
  Serial.println('\n');
}

void loop(){
  // put your main code here, to run repeatedly:
  if(!flag_first_time){
    getMeanProximity();
  }
  machine.run();
  delay(STATE_DELAY);

  
}

void state_calibration(){

  int value1 = ADCTouch.read(A1);

  if(value1 > 600){
    machine.transitionTo(shoulder);
  }
  else{
    machine.transitionTo(still_neutral);
  }
  
  flag_first_time = false;
  
}

void state_still_neutral(){

  Serial.println("I AM ON THE TABLE");

  
  
}

void state_shoulder(){

  Serial.println("I AM ON THE SHOULDER");
  
}

void state_fear(){

  Serial.println("FEAR");

  
}

bool transition_shoulder_big_noise(){

  int val = analogRead(soundPin);

  val = val - 65;

  if(val > big_sound_th){
    return true;
  }

  return false;

  
}

bool transition_calib_shoulder(){

  if(mean > old_mean + 30){
    old_mean = mean;
    return true;
  }

  return false;
  
}

bool transition_calib_still_neutral(){

  if(mean < old_mean - 30){
    old_mean = mean;
    return true;
  }

  return false;
  
}

bool transition_shoulder_still(){

  if(mean < old_mean - 30){
    old_mean = mean;
    return true;
  }

  return false;
  
}

bool transition_still_calib(){

  if(mean > old_mean + 30){
    old_mean = mean;
    return true;
  }

  return false;
  
}
