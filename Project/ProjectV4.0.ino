#include <StateMachine.h>
#include <ADCTouch.h>
#include <MPU6050_tockn.h>
#include <math.h>
#include <Servo.h>

//include for microphone libraries
#include <SoftwareSerial.h>
#include "VoiceRecognitionV3.h"

#include<Wire.h>

// [-1121,-1120] --> [0,18]  [-67,-67] --> [0,1] [3649,3650] --> [16364,16386] [91,91] --> [0,1] [12,12] --> [0,1] [-28,-27] --> [-1,2]


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

const int STATE_DELAY = 500;

MPU6050 mpu6050(Wire);


long timer = 0;

// Servo motors params
int servo_vert = 5;
int servo_horiz = 6;
Servo servo_head_vertical;
Servo servo_head_horizontal;

// Vibration motor params
int vibr_pin = 4;

// Multicolor LEDs params
int red_pin = 11;
int green_pin = 10;
int blue_pin = 9;

// No movement params
float refX, accX;
int no_mov_count = 0;
int threshold_counter = 10;
float threshold_movement = 0.1;

// Sudden movement params
float threshold_sudden_mov = 0.3;

// Capacitive sensor params
int ref1;
static int value_off = 0;
bool flag_first_time = true;
int mean, old_mean;
int mean_offset = 30;

// Big noise sensor params
const int soundPin = A0;
int Digital_Pin = 3;



VR myVR(3,2);    // 2:RX 3:TX, you can choose your favourite pins.
uint8_t records[7]; // save record
uint8_t buf[64];
#define COCO    (1)
#define STUPID   (2) 
#define HEY_COCO   (3) 
#define BEAUTIFUL   (4) 


// Sad still state parameters
long sad_still_timer = 0;
struct states{
  bool happy;
  bool sad;
  bool angry;
  bool fear; 
  };

//storing variable of the petting touch value
int old_value_pet; //value read by the capacitive sensor for detecting petting
int ref_pet; //initial value of the capacitive sensor for moving the mean to 0

states state = {0,0,0,0};

StateMachine machine = StateMachine();



// States declarations
State* calibration = machine.addState(&state_calibration); //state 0
State* still_neutral = machine.addState(&state_still_neutral); //state 1
State* shoulder = machine.addState(&state_shoulder); //state 2
State* fear = machine.addState(&state_fear); //state 3
State* sad_still = machine.addState(&state_sad_still); //stato 4
State* angry_on_shoulder = machine.addState(&state_angry_on_shoulder);





////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  myVR.begin(9600);
  Wire.begin();

  pinMode(soundPin,INPUT);
  pinMode(Digital_Pin, INPUT);

  pinMode(red_pin, OUTPUT);
  pinMode(green_pin, OUTPUT);
  pinMode(blue_pin, OUTPUT);

  pinMode(vibr_pin, OUTPUT);

    
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);

  mpu6050.update();

  refX = mpu6050.getAccX();



  if(myVR.load((uint8_t)COCO) >= 0){
    Serial.println("COCO loaded");
  }
  
  if(myVR.load((uint8_t)STUPID) >= 0){
    Serial.println("STUPID loaded");
  }

    if(myVR.load((uint8_t)HEY_COCO) >= 0){
    Serial.println("HEYCOCO loaded");
  }
  
  if(myVR.load((uint8_t)BEAUTIFUL) >= 0){
    Serial.println("BEAUTIFUL loaded");
  }
  

  ref1 = ADCTouch.read(A1, 500);//account for the capacitance of the pad
  ref_pet = ADCTouch.read(A1, 500); //account for the capacitance of petting sensor
  old_value_pet = ADCTouch.read(A2)-ref_pet; //initializing the starting value of petting capacitor

  // Transitions exiting calibration state
  calibration->addTransition(&transition_calib_shoulder, shoulder);
  calibration->addTransition(&transition_calib_still_neutral, still_neutral);

  // Transitions exiting shoulder state
  shoulder->addTransition(&transition_shoulder_still, still_neutral);
  //shoulder->addTransition(&transition_shoulder_big_noise, fear);
  shoulder->addTransition(&transition_shoulder_happy_petting_shoulder, shoulder);
  shoulder->addTransition(&transition_shoulder_word_recognition, shoulder);
  shoulder->addTransition(&transition_shoulder_no_movement, angry_on_shoulder);
  //shoulder->addTransition(&transition_shoulder_sudden_movement, angry);
  //shoulder->addTransition(&transition_shoulder_no_movement, shoulder);
  shoulder->addTransition(&transition_shoulder_sudden_movement, shoulder);

  // Transitions exiting still_neutral state
  still_neutral->addTransition(&transition_still_shoulder, shoulder);
  still_neutral->addTransition(&transition_still_sad_still, sad_still);


  // Transitions exiting sad_still state
  sad_still->addTransition(&transition_sad_still_shoulder, shoulder);

  // Transitions exiting angry state
  angry_on_shoulder->addTransition(&transition_angry_on_shoulder_shoulder, shoulder);


  //Servo1.attach(servoPin);
  //Servo1.write(90);

  set_neutral();
  display_state();

}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

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

  Serial.print("OLD MEAN: ");Serial.print(old_mean);Serial.print(" MEAN: ");Serial.println(mean);
  
  if(mean > old_mean + mean_offset){
    old_mean = mean;
    //Serial.print("STILL-SHOULDER - ");Serial.print("OLD MEAN: ");Serial.print(old_mean);Serial.print(" MEAN: ");Serial.println(mean);
    return true;
  }

  return false;

}

// Function that checks if mean is below threshold
bool checkBelowMeanProximity(){

  Serial.print("OLD MEAN: ");Serial.print(old_mean);Serial.print(" MEAN: ");Serial.println(mean);
  
  if(mean < old_mean - mean_offset){
    old_mean = mean;
    //Serial.print("SHOULDER-STILL - ");Serial.print("OLD MEAN: ");Serial.print(old_mean);Serial.print(" MEAN: ");Serial.println(mean);
    return true;
  }

  return false;

}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void loop(){
  // put your main code here, to run repeatedly:
  if(!flag_first_time){
    getMeanProximity();
  }
  machine.run();
  delay(STATE_DELAY);

  
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

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

  if(machine.executeOnce){
    sad_still_timer = 0;
  }
  //sad_still_timer++;
  Serial.println("I AM ON THE TABLE");
}

void state_shoulder(){

  mpu6050.update();
  accX = mpu6050.getAccX();

  if(machine.executeOnce){
    no_mov_count = 0;
  }
  
  Serial.println("I AM ON THE SHOULDER");
}

void state_fear(){

  Serial.println("FEAR");
}

void state_sad_still(){
  Serial.println("WAITING TO BE PICKED UP");
}

void state_angry_on_shoulder(){
  //display_state();
  Serial.println("IN ANGRY STATE-ON-SHOULDER");
}




////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool transition_angry_on_shoulder_shoulder(){
  //delay(3000);
  mpu6050.update();
  float accX = mpu6050.getAccX();

  if(abs(accX - refX) > 0.2){
    Serial.println("MI SONO MOSSO");
    set_neutral();
    display_state();
    return true;
  }

  Serial.println("HERE");
  return false;
}

bool transition_shoulder_sudden_movement(){

  Serial.println("Checking for sudden movement.");
  
  if(abs(accX - refX) > threshold_sudden_mov){
    Serial.println("SUDDEN MOVEMENT");
    set_fear();
    display_state();
    set_neutral();
    display_state();
    return true;
  }
  else{
    return false;
  }
}

bool transition_shoulder_no_movement(){

  bool return_value = false;

  if(no_mov_count >= threshold_counter){
    no_mov_count = 0;
  }
  else{
    no_mov_count = no_mov_count + 1;
  }
  
  if(no_mov_count < threshold_counter){ //per fare 10 iterazioni e raggiungere lo stato "angry" ci mette circa 20 secondi
    if(abs(accX - refX) > threshold_movement){
      Serial.println("MOVING");
      return_value = false;
      no_mov_count = 0;
    }
  }
  else{
    if(abs(accX - refX) > threshold_movement){
      Serial.println("MOVING");
      return_value = false;
      no_mov_count = 0;
    }
    else{
      Serial.print("NOT MOVING ");
      set_angry();
      display_state();
      //set_neutral();
      //display_state();
      return_value = true;
    }
  }

  Serial.print("COUNTER: ");Serial.println(no_mov_count);
  return return_value;
  
}

bool transition_shoulder_big_noise(){

  int val = analogRead(soundPin);

  // Parameters
  int sound_offset = 65;
  int big_sound_th = 90;

  val = val - sound_offset;

  //int Digital = digitalRead(Digital_Pin);

  Serial.print("NOISE VAL: ");Serial.println(val);
  //Serial.print("Limit :");Serial.println(Digital);

  

  if(val > big_sound_th){
    return false;
  }

  return false;
}



bool transition_calib_shoulder(){
  
  return checkOverMeanProximity();

}

bool transition_sad_still_shoulder(){

  bool mean_over = checkOverMeanProximity();

  if(mean_over){
    set_happy();
    display_state();
    set_neutral();
    display_state();
    return true;
  }

  return false;
  
}

bool transition_calib_still_neutral(){
  
  return checkBelowMeanProximity();
  
}

bool transition_shoulder_still(){

  return checkBelowMeanProximity();
  
}

bool transition_shoulder_happy_petting_shoulder(){
  int new_value_pet = ADCTouch.read(A2) - ref_pet;
  Serial.print("Value read by the petting capacitive sensor: ");Serial.println(new_value_pet);
  if(new_value_pet > old_value_pet + 30){
    Serial.println("Qualcuno mi tocca");
    set_happy();
    display_state();
    set_neutral();
    display_state();
    return true;

  }

  old_value_pet = new_value_pet;

  return false;
  
}

bool transition_shoulder_word_recognition(){
  int ret;
  Serial.print("Checking for some words...");
  ret = myVR.recognize(buf, 50);
  
  if(ret>0){
    switch(buf[1]){
      case COCO:
        Serial.println("COCO");
        break;
      case STUPID:
        Serial.println("STUPID");
        set_sad();
        display_state();
        set_neutral();
        display_state();
        break;
      case HEY_COCO:
        Serial.println("HEY_COCO");
        break;
      case BEAUTIFUL:
        Serial.println("BEAUTIFUL");
        set_happy();
        display_state();
        set_neutral();
        display_state();
        break;
      default:
        Serial.println("Record function undefined");
        return false;
        break;

        
    }

    return true;
  }
    Serial.println("No words");
    return false;
}

bool transition_still_shoulder(){

  return checkOverMeanProximity();
  
}

bool transition_still_sad_still(){

  if(sad_still_timer > 10){
    //Serial.println("Sono passati 5 secondi");
    set_sad();
    display_state();
    //é triste per sempre fin quando non viene preso
    //sad_still_timer = 0;
    return true;
  }
  else{
    sad_still_timer++;
  }

  return false;
  
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void display_state(){

  if(state.happy){
    Serial.println("Happy");
    RGB_color(255, 255, 0);

    servo_head_vertical.attach(servo_vert);
    servo_head_vertical.write(20);
    
    for(int j = 0; j < 4; j++){
      for(int i = 90; i < 150; i++){
        servo_head_horizontal.write(i);
        delay(10);
      }

      for(int i = 90; i > 30; i--){
        servo_head_horizontal.write(i);
        delay(10);
      }
    }

    delay(1000);
    
  }

  else if(state.sad){
    Serial.println("Sad");
    RGB_color(0, 0, 255);

    /*for(int i = 90; i < 120; i++){
      servo_head_vertical.write(i);
      delay(10);
    }*/

    for(int i = 0; i < 30; i++){
      servo_head_vertical.write(i);
      delay(10);
    }

    delay(3000);
    
  }

  else if(state.angry){
    Serial.println("Angry");
    RGB_color(255, 0, 0);
    
    digitalWrite(vibr_pin, HIGH);
    delay(2000);
    digitalWrite(vibr_pin, LOW);

    delay(3000);
  }

  else if(state.fear){
    Serial.println("Fear");
    RGB_color(255, 0, 255);

    //digitalWrite(vibr_pin, HIGH);

    int starting_position = 90;

    servo_head_horizontal.write(starting_position + 10);

    servo_head_horizontal.write(starting_position - 10);

    servo_head_horizontal.write(starting_position + 10);

    servo_head_horizontal.write(starting_position - 10);

    /*for(int j = 0; j < 4; j++){
      for(int i = 0; i < 5; i++){
        servo_head_horizontal.write(starting_position + 10);
        servo_head_horizontal.write(starting_position - 10);
        //Serial.println("FIRST I - FOR");
        delay(10);
      }
    }*/
    
    
    //delay(2000);
    //digitalWrite(vibr_pin, LOW);

    delay(3000);
  }

  else{
    Serial.println("Neutral");
    RGB_color(0, 0, 0);
    
    servo_head_vertical.attach(servo_vert);
    servo_head_vertical.write(20);

    servo_head_horizontal.attach(servo_horiz);
    servo_head_horizontal.write(85);
    
    return;      
  }

  //delay(5000);

   
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

void RGB_color(int red_light_value, int green_light_value, int blue_light_value)
 {
  analogWrite(red_pin, red_light_value);
  analogWrite(green_pin, green_light_value);
  analogWrite(blue_pin, blue_light_value);
}
