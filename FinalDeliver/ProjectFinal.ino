// State Machine library
#include <StateMachine.h>


// Accelerometer library
#include <MPU6050_tockn.h>

// Servo Motors library
#include <Servo.h>

// Microphone libraries
#include <SoftwareSerial.h>
#include "VoiceRecognitionV3.h"

// Speaker library
#include "DFRobotDFPlayerMini.h"
// MP3 songs indexes
//1 angry
//2 fear best
//3 fear
//4 happy
//5 normal
//6 sad

// Generic libraries
#include <math.h>
#include <ADCTouch.h>
#include <Wire.h>

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

const int STATE_DELAY = 500;
long timer = 0;

MPU6050 mpu6050(Wire);


// Speaker setup
SoftwareSerial mySoftwareSerial(12, 13); // RX, TX
DFRobotDFPlayerMini myDFPlayer;

// Servo motors params
int servo_vert = 5;
int servo_horiz = 6;
int servo_ears = 3;
Servo servo_head_ears;
Servo servo_head_vertical;
Servo servo_head_horizontal;

// Vibration motor params
int vibr_pin = 4;

// Multicolor LEDs params
int red_pin = 11;
int green_pin = 10;
int blue_pin = 9;

// No movement params
float refX,refY,refZ, accX,accY,accZ;
int no_mov_count = 0;
int threshold_counter = 10;
float threshold_movement = 0.4;

// Sudden movement params
float threshold_sudden_mov = 0.9;

// Capacitive sensor params
static int value_off = 0;
bool flag_first_time = true;
int mean, old_mean;
int mean_offset = 30;//30



VR myVR(8, 7); // 7:RX 8:TX, you can choose your favourite pins.
uint8_t records[7]; // save record
uint8_t buf[64];
#define CIAO    (0)
#define CUTE   (1) 
#define HORRIBLE   (2) 
#define STUPID   (3) 
#define GOOD   (4) 
#define COCO (5)


// Sad still state parameters
long sad_still_timer = 0;
struct states{
  bool happy;
  bool sad;
  bool angry;
  bool fear; 
  };

//A2 petting
// storing variable of the petting touch value
int old_value_pet; // value read by the capacitive sensor for detecting petting
int ref_pet; // initial value of the capacitive sensor for moving the mean to 0

states state = {0,0,0,0};

// Declaration of State Machine
StateMachine machine = StateMachine();

// States declarations
State* calibration = machine.addState(&state_calibration); //state 0
State* still_neutral = machine.addState(&state_still_neutral); //state 1
State* shoulder = machine.addState(&state_shoulder); //state 2
State* fear = machine.addState(&state_fear); //state 3
State* sad_still = machine.addState(&state_sad_still); //stato 4
State* angry_on_shoulder = machine.addState(&state_angry_on_shoulder); //stato 5

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup() {

  // Check if SD card is inside MP3 player
  mySoftwareSerial.begin(9600);
  if (!myDFPlayer.begin(mySoftwareSerial)) {  //Use softwareSerial to communicate with mp3.
    Serial.println(F("Unable to begin:"));
    Serial.println(F("1.Please recheck the connection!"));
    Serial.println(F("2.Please insert the SD card!"));
    while(true){
      delay(0); // Code to compatible with ESP8266 watch dog.
    }
  }
  myDFPlayer.volume(25);  //Set volume value. From 0 to 30

  // Begin communication to Serial Monitor
  Serial.begin(9600);
  myVR.begin(9600);
  Wire.begin();

  // Setup LEDs pins
  pinMode(red_pin, OUTPUT);
  pinMode(green_pin, OUTPUT);
  pinMode(blue_pin, OUTPUT);

  RGB_color(255, 255, 255);

  // Setup vibration motor pin
  pinMode(vibr_pin, OUTPUT);

  // Setup accelerometer and get reference values
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
  mpu6050.update();

  refX = mpu6050.getAccX();
  refY = mpu6050.getAccY();
  refZ = mpu6050.getAccZ();

  Serial.print("accX");Serial.println(refX);
  Serial.print("accY");Serial.println(refY);
  Serial.print("accZ");Serial.println(refZ);

  
  // Setup Voice Recognition
  if(myVR.load((uint8_t)CIAO) >= 0){
    Serial.println("COCO loaded");
  }
  
  if(myVR.load((uint8_t)CUTE) >= 0){
    Serial.println("STUPID loaded");
  }

    if(myVR.load((uint8_t)HORRIBLE) >= 0){
    Serial.println("HEYCOCO loaded");
  }

    
  ref_pet = analogRead(A2); // account for the capacitance of petting sensor
  old_value_pet = analogRead(A2)-ref_pet; // initializing the starting value of petting capacitor
  
  // State Machine transitions
  // Transitions exiting calibration state
  calibration->addTransition(&transition_calib_shoulder, shoulder);
  calibration->addTransition(&transition_calib_still_neutral, still_neutral);

  // Transitions exiting shoulder state
  shoulder->addTransition(&transition_shoulder_still, still_neutral);
  shoulder->addTransition(&transition_shoulder_happy_petting_shoulder, shoulder);
  shoulder->addTransition(&transition_shoulder_word_recognition, shoulder);
  shoulder->addTransition(&transition_shoulder_no_movement, angry_on_shoulder);
  shoulder->addTransition(&transition_shoulder_sudden_movement, shoulder);

  // Transitions exiting still_neutral state
  still_neutral->addTransition(&transition_still_shoulder, shoulder);
  still_neutral->addTransition(&transition_still_sad_still, sad_still);


  // Transitions exiting sad_still state
  sad_still->addTransition(&transition_sad_still_shoulder, shoulder);

  // Transitions exiting angry state
  angry_on_shoulder->addTransition(&transition_angry_on_shoulder_shoulder, shoulder);

  // Start by setting Coco to neutral state
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
  
    value1 = analogRead(A1); //   --> 100 samples
    
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
  
  if(mean > old_mean + mean_offset and mean >300){
    old_mean = mean;
    return true;
  }

  return false;

}

// Function that checks if mean is below threshold
bool checkBelowMeanProximity(){

  Serial.print("OLD MEAN: ");Serial.print(old_mean);Serial.print(" MEAN: ");Serial.println(mean);
  
  if(mean < old_mean - mean_offset and mean<100){
    old_mean = mean;
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

  // Code to run the state machine once per loop
  machine.run();
  delay(STATE_DELAY);

  
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// STATES

// Calibration state
void state_calibration(){

  int value1 = analogRead(A1);

  if(value1 > 400){
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

// Neutral state, when Coco is on the table
void state_still_neutral(){

  if(machine.executeOnce){
    sad_still_timer = 0;
  }
  Serial.println("I AM ON THE TABLE");
}

// Neutral state, when Coco is on the shoulder
void state_shoulder(){
  
  mpu6050.update();
  accX = mpu6050.getAccX();

  if(machine.executeOnce){
    no_mov_count = 0;
  }
  
  Serial.println("I AM ON THE SHOULDER");
}

// Scared state, when Coco expresses fear
void state_fear(){
  Serial.println("FEAR");
}

// Sad state, when Coco wants to be picked up from the table
void state_sad_still(){
  
  Serial.println("WAITING TO BE PICKED UP");
}

// Angry state, when Coco warns you to move
void state_angry_on_shoulder(){
  Serial.println("IN ANGRY STATE-ON-SHOULDER");
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//TRANSITIONS

// Checks if Coco is moving while on the shoulder
bool transition_angry_on_shoulder_shoulder(){
  //delay(3000);
  mpu6050.update();
  accX = mpu6050.getAccX();
  accY = mpu6050.getAccY();
  accZ = mpu6050.getAccZ();

  if(abs(accX - refX) + abs(accY - refY) + abs(accZ - refZ)> threshold_movement){
    Serial.println("I'm Moving");
    set_neutral();
    display_state();
    return true;
  }

  Serial.println("HERE");
  return false;
}

// Checks if a sudden movement has occurred
bool transition_shoulder_sudden_movement(){
  
  if(abs(accX - refX) + abs(accY - refY) + abs(accZ - refZ) > threshold_sudden_mov){
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

// Checks if there was no movement after some time
bool transition_shoulder_no_movement(){

  bool return_value = false;

  accX = mpu6050.getAccX();
  accY = mpu6050.getAccY();
  accZ = mpu6050.getAccZ();
  
  Serial.print("Acc Error");Serial.println(abs(accX - refX) + abs(accY - refY) + abs(accZ - refZ));
  

  if(no_mov_count >= threshold_counter){
    no_mov_count = 0;
  }
  else{
    no_mov_count = no_mov_count + 1;
  }
  
  if(no_mov_count < threshold_counter){
    if(abs(accX - refX) + abs(accY - refY) + abs(accZ - refZ) > threshold_movement){
      Serial.println("MOVING");
      return_value = false;
      no_mov_count = 0;
    }
  }
  else{
    if(abs(accX - refX) + abs(accY - refY) + abs(accZ - refZ) > threshold_movement){
      Serial.println("MOVING");
      return_value = false;
      no_mov_count = 0;
    }
    else{
      Serial.print("NOT MOVING ");
      set_angry();
      display_state();
      return_value = true;
    }
  }

  Serial.print("COUNTER MOVEMENT ON SHOULDER: ");Serial.println(no_mov_count);
  return return_value;
  
}

// Calibration transition
bool transition_calib_shoulder(){
  
  return checkOverMeanProximity();

}

// Check if Coco has been picked up after having been sad
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

// Move ears down if Coco has been on the table for too long
bool transition_shoulder_still(){

  if(checkBelowMeanProximity()){
    for(int i = 90; i > 60; i--){
      servo_head_ears.write(i);
      delay(10);
    }

    return 1;
  }
  else{
    return 0;
  }
  
}

// Checks if Coco has been pet
bool transition_shoulder_happy_petting_shoulder(){
  int new_value_pet = analogRead(A2);
  
  if(new_value_pet > 200){
    Serial.println("PETTING");
    set_happy();
    display_state();
    set_neutral();
    display_state();
    return true;

  }

  old_value_pet = new_value_pet;

  return false;
  
}

// Checks if any word has been recognized
bool transition_shoulder_word_recognition(){
  int ret;
  Serial.print("Checking for some words...");
  ret = myVR.recognize(buf, 50);
  
  if(ret>0){
    switch(buf[1]){
      case CIAO:
        Serial.println("CIAO");
        myDFPlayer.play(5);
        break;
      case CUTE:
        Serial.println("CUTE");
        set_happy();
        display_state();
        set_neutral();
        display_state();
        break;
      case HORRIBLE:
        Serial.println("HORRIBLE");       
        set_sad();
        display_state();
        set_neutral();
        display_state();
        break;
      case STUPID:
        Serial.println("STUPID");       
        set_sad();
        display_state();
        set_neutral();
        display_state();
        break;
      case GOOD:
        Serial.println("GOOD");       
        set_happy();
        display_state();
        set_neutral();
        display_state();
        break;
      case COCO:
        Serial.println("GATTO");       
        myDFPlayer.play(5);
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

  //return checkOverMeanProximity();
    if(checkOverMeanProximity()){
    for(int i = 90; i < 120; i++){
      servo_head_ears.write(i);
      delay(10);
    }

    return 1;
  }
  else{
    return 0;
  }
  
}

// Makes Coco sad
bool transition_still_sad_still(){

  if(sad_still_timer > 10){
    set_sad();
    display_state();
    return true;
  }
  else{
    sad_still_timer++;
  }

  return false;
  
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Sets the expression of Coco
void display_state(){

  if(state.happy){
    Serial.println("Happy");
    RGB_color(255, 255, 0);

    servo_head_vertical.attach(servo_vert);
    servo_head_vertical.write(90);

    servo_head_ears.write(90);

    myDFPlayer.play(4);
    delay(2000);
    
    for(int j = 0; j < 4; j++){
      for(int i = 90; i < 150; i++){
        //Serial.println(i);
        servo_head_horizontal.write(i);
        servo_head_ears.write(i);
        delay(10);
      }   

      for(int i = 90; i > 30; i--){
        servo_head_horizontal.write(i);
        servo_head_ears.write(i);
        delay(10);
      }
    }

    

    delay(1000);
    
  }

  else if(state.sad){
    Serial.println("Sad");
    RGB_color(0, 0, 255);


    Serial.println("SAD sound");
    delay(500);
    myDFPlayer.play(6);
    delay(2000);

    for(int i = 90; i < 130; i++){
      servo_head_vertical.write(i);
      delay(10);
    }
    
     for(int i = 90; i > 30; i--){
      servo_head_ears.write(i);
      delay(10);
    }
    

    delay(1000);
   
  }

  else if(state.angry){
    Serial.println("Angry");
    RGB_color(255, 0, 0);
    
    digitalWrite(vibr_pin, HIGH);

    servo_head_ears.write(30);

    delay(100);

    servo_head_horizontal.write(50);

   
    delay(500);
    myDFPlayer.play(1);
    
    delay(2000);

    servo_head_ears.write(120);

    delay(1000);

    digitalWrite(vibr_pin, LOW);
  }

  else if(state.fear){ 
    Serial.println("Fear");
    RGB_color(255, 0, 255);
    myDFPlayer.play(3);
    delay(2000);
    digitalWrite(vibr_pin, HIGH);

    int starting_position = 90;
    int ang = 10;
    int del = 100;

    int ang_incr = 5; //10;
    int del_incr = 100;

    servo_head_ears.write(150);

    for(int i = 0; i < 4; i++){
      for(int j = 0; j < 6; j++){
        
        servo_head_horizontal.write(starting_position + ang);
        delay(del);
        servo_head_horizontal.write(starting_position - ang);
        delay(del); 
        ang = ang + ang_incr;
      }
      ang = 10;
    }
    
    digitalWrite(vibr_pin, LOW);   

  }

  else{
    Serial.println("Neutral");
    
    RGB_color(0, 0, 0);
    
    servo_head_vertical.attach(servo_vert);
    servo_head_vertical.write(90);

    
    servo_head_horizontal.attach(servo_horiz);
    servo_head_horizontal.write(90);
    
    
    servo_head_ears.attach(servo_ears);
    servo_head_ears.write(90);
    
    return ;
  }
}

// Functions used to set the state of Coco
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

// Sets the color of each LED
void RGB_color(int red_light_value, int green_light_value, int blue_light_value)
 {
  analogWrite(red_pin, red_light_value);
  analogWrite(green_pin, green_light_value);
  analogWrite(blue_pin, blue_light_value);
}
