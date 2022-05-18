#include <StateMachine.h>
#include <ADCTouch.h>
#include <MPU6050_tockn.h>
#include <math.h>

#include<Wire.h>

// [-1121,-1120] --> [0,18]  [-67,-67] --> [0,1] [3649,3650] --> [16364,16386] [91,91] --> [0,1] [12,12] --> [0,1] [-28,-27] --> [-1,2]


const int STATE_DELAY = 500;

MPU6050 mpu6050(Wire);


long timer = 0;

// Capacitive sensor params
int ref0, ref1;
static int value_off = 0;
bool flag_first_time = true;
int mean, old_mean;
int mean_offset = 30;

// Big noise sensor params
const int soundPin = A0;
int Digital_Pin = 3;

// Sad still state parameters
long sad_still_timer = 0;
struct states{
  bool happy;
  bool sad;
  bool angry;
  bool fear; 
  };


states state = {0,0,0,0};

StateMachine machine = StateMachine();


// States declarations
State* calibration = machine.addState(&state_calibration); //state 0
State* still_neutral = machine.addState(&state_still_neutral); //state 1
State* shoulder = machine.addState(&state_shoulder); //state 2
State* fear = machine.addState(&state_fear); //state 3
State* sad_still = machine.addState(&state_sad_still); //stato 4


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Wire.begin();

  pinMode(soundPin,INPUT);
  pinMode(Digital_Pin, INPUT);
  
  //mpu6050.begin();
  //mpu6050.calcGyroOffsets(true);

  ref1 = ADCTouch.read(A1, 500);    //account for the capacitance of the pad

  // Transitions exiting calibration state
  calibration->addTransition(&transition_calib_shoulder, shoulder);
  calibration->addTransition(&transition_calib_still_neutral, still_neutral);

  // Transitions exiting shoulder state
  shoulder->addTransition(&transition_shoulder_still, still_neutral);
  shoulder->addTransition(&transition_shoulder_big_noise, fear);

  // Transitions exiting still_neutral state
  still_neutral->addTransition(&transition_still_shoulder, shoulder);
  still_neutral->addTransition(&transition_still_sad_still, sad_still);

}

// Function that sets the mean value of the capacitive-proximity sensor over "samples_number" samples
void getMeanProximity(){

  int count = 0;
  int total_value = 0;
  int value0, value1;

  // Parameters
  int samples_number = 30;
  int sample_delay = 30;
  
  mean = 0;
  
  while(count < samples_number){
  
    value1 = ADCTouch.read(A1);   //   --> 100 samples
    
    total_value = total_value + value1;
  
    count++;
    
    delay(sample_delay);
  }
  
  mean = total_value / count;

  if(old_mean == 0){
    old_mean = mean;
  }

  Serial.print("MEAN: ");Serial.print(mean);
  Serial.println('\n');
}

// Function that checks if mean is above threshold
bool checkOverMeanProximity(){
    
  if(mean > old_mean + mean_offset){
    old_mean = mean;
    Serial.print("STILL-SHOULDER - ");Serial.print("OLD MEAN: ");Serial.print(old_mean);Serial.print(" MEAN: ");Serial.println(mean);
    return true;
  }

  return false;

}

// Function that checks if mean is below threshold
bool checkBelowMeanProximity(){
    
  if(mean < old_mean - mean_offset){
    old_mean = mean;
    Serial.print("SHOULDER-STILL - ");Serial.print("OLD MEAN: ");Serial.print(old_mean);Serial.print(" MEAN: ");Serial.println(mean);
    return true;
  }

  return false;

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

  if(flag_first_time){
    old_mean = mean;
  }
  
  flag_first_time = false;
  
}

void state_still_neutral(){

  sad_still_timer++;
  Serial.println("I AM ON THE TABLE");
}

void state_shoulder(){

  Serial.println("I AM ON THE SHOULDER");
}

void state_fear(){

  Serial.println("FEAR");
}

void state_sad_still(){
  Serial.println("WAITING TO BE PICKED UP");
}

bool transition_shoulder_big_noise(){

  int val = analogRead(soundPin);

  // Parameters
  int sound_offset = 65;
  int big_sound_th = 90;

  val = val - sound_offset;

  int Digital = digitalRead(Digital_Pin);

  Serial.print("NOISE VAL: ");Serial.println(val);
  Serial.print("Limit :");Serial.println(Digital);

  

  if(val > big_sound_th){
    return false;
  }

  return false;
}


bool transition_calib_shoulder(){
  
  return checkOverMeanProximity();

}

bool transition_calib_still_neutral(){
  
  return checkBelowMeanProximity();
  
}

bool transition_shoulder_still(){

  return checkBelowMeanProximity();
  
}

bool transition_still_shoulder(){

  return checkOverMeanProximity();
  
}

bool transition_still_sad_still(){

  if(sad_still_timer > 10){
    Serial.println("Sono passati 5 secondi");
    set_sad();
    display_state();
    //é triste per sempre fin quando non viene preso
    sad_still_timer = 0;
    return true;
  }

  return false;
  
}

void display_state(){

  if(state.happy){
    Serial.println("Happy");
  }

  else if(state.sad){
    Serial.println("Sad");
  }

  else if(state.angry){
    Serial.println("Angry");
  }

  else if(state.fear){
    Serial.println("Fear");
  }

  else{
    Serial.println("Neutral");
    return;      
  }

  delay(5000);

   
}


void set_neutral(){ 
  state ={0,0,0,0};
}

void set_happy(){
  state={1,0,0,0};
}

void set_angry(){
  state={0,0,1,0};
}

void set_fear(){
  state={0,0,0,1};
}

void set_sad(){
  state={0,1,0,0};
}
