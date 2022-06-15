#include <StateMachine.h>
#include <ADCTouch.h>
#include <MPU6050_tockn.h>
#include <math.h>
#include <Servo.h>


//include for microphone libraries
#include <SoftwareSerial.h>
#include "VoiceRecognitionV3.h"

#include<Wire.h>

#include "DFRobotDFPlayerMini.h"



//1 angry
//2 fear best
//3 fear
//4 happy
//5 normal
//6 sad




// [-1121,-1120] --> [0,18]  [-67,-67] --> [0,1] [3649,3650] --> [16364,16386] [91,91] --> [0,1] [12,12] --> [0,1] [-28,-27] --> [-1,2]


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

const int STATE_DELAY = 500;

MPU6050 mpu6050(Wire);


//Speaker
SoftwareSerial mySoftwareSerial(12, 13); // RX, TX
DFRobotDFPlayerMini myDFPlayer;


long timer = 0;

// Servo motors params
int servo_vert = 5;
int servo_horiz = 6; //era 2, ora mettiamo 6
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
float refX, accX;
int no_mov_count = 0;
int threshold_counter = 10;
float threshold_movement = 0.25;

// Sudden movement params
float threshold_sudden_mov = 0.4;

// Capacitive sensor params
int ref1;
static int value_off = 0;
bool flag_first_time = true;
int mean, old_mean;
int mean_offset = 30;


// Sound detector sensor params
const int soundPin = A0;
int sound_th = 180; //con 200 va, forse è meglio abbassarla un po



VR myVR(8, 7);//VR myVR(3,2);    // 2:RX 3:TX, you can choose your favourite pins.
uint8_t records[7]; // save record
uint8_t buf[64];
#define CIAO    (0)
#define CUTE   (1) 
#define HORRIBLE   (2) 
//#define BEAUTIFUL   (4) 


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
State* angry_on_shoulder = machine.addState(&state_angry_on_shoulder); //stato 5





////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {
  

  mySoftwareSerial.begin(9600);

  if (!myDFPlayer.begin(mySoftwareSerial)) {  //Use softwareSerial to communicate with mp3.
    Serial.println(F("Unable to begin:"));
    Serial.println(F("1.Please recheck the connection!"));
    Serial.println(F("2.Please insert the SD card!"));
    while(true){
      delay(0); // Code to compatible with ESP8266 watch dog.
    }
  }
  myDFPlayer.volume(20);  //Set volume value. From 0 to 30 //best is 30


  
  // put your setup code here, to run once:
  Serial.begin(9600);
  myVR.begin(9600);
  Wire.begin();

  pinMode(red_pin, OUTPUT);
  pinMode(green_pin, OUTPUT);
  pinMode(blue_pin, OUTPUT);

  RGB_color(255, 255, 255);

  pinMode(vibr_pin, OUTPUT);




    
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);

  mpu6050.update();

  refX = mpu6050.getAccX();



  if(myVR.load((uint8_t)CIAO) >= 0){
    Serial.println("COCO loaded");
  }
  
  if(myVR.load((uint8_t)CUTE) >= 0){
    Serial.println("STUPID loaded");
  }

    if(myVR.load((uint8_t)HORRIBLE) >= 0){
    Serial.println("HEYCOCO loaded");
  }

  /*
  if(myVR.load((uint8_t)BEAUTIFUL) >= 0){
    Serial.println("BEAUTIFUL loaded");
  }
  */
  
  //MODIFICA
  //ref1 = ADCTouch.read(A1, 500);//account for the capacitance of the pad
  ref1 = ADCTouch.read(A1, 500); //account for the capacitance of the pad
  //
  
  ref_pet = ADCTouch.read(A1, 500); //account for the capacitance of petting sensor
  old_value_pet = ADCTouch.read(A2)-ref_pet; //initializing the starting value of petting capacitor
  
  
  // Transitions exiting calibration state
  calibration->addTransition(&transition_calib_shoulder, shoulder);
  calibration->addTransition(&transition_calib_still_neutral, still_neutral);

  // Transitions exiting shoulder state
  shoulder->addTransition(&transition_shoulder_still, still_neutral);
  shoulder->addTransition(&transition_shoulder_big_noise, shoulder);
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

  servo_head_vertical.attach(servo_vert);
  servo_head_horizontal.attach(servo_horiz);
  servo_head_ears.attach(servo_ears);

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

    //MODIFICA
    //value1 = ADCTouch.read(A1);   //   --> 100 samples
    value1 = ADCTouch.read(A1);   //   --> 100 samples
    //
    
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

  //Serial.print("OLD MEAN: ");Serial.print(old_mean);Serial.print(" MEAN: ");Serial.println(mean);
  
  if(mean > old_mean + mean_offset){
    old_mean = mean;
    //Serial.print("STILL-SHOULDER - ");Serial.print("OLD MEAN: ");Serial.print(old_mean);Serial.print(" MEAN: ");Serial.println(mean);
    return true;
  }

  return false;

}

// Function that checks if mean is below threshold
bool checkBelowMeanProximity(){

  //Serial.print("OLD MEAN: ");Serial.print(old_mean);Serial.print(" MEAN: ");Serial.println(mean);
  
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
  //Serial.println(sound);
  delay(STATE_DELAY);

/*
  int ret;

  Serial.print("Checking for some words...");
  ret = myVR.recognize(buf, 50);
  
  if(ret>0){
    switch(buf[1]){
      case CIAO:
        Serial.println("CIAO");
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
      default:
        Serial.println("Record function undefined");
        break;   
    }
  }

*/
  
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void state_calibration(){

  //MODIFICA
  //int value1 = ADCTouch.read(A1);
  int value1 = ADCTouch.read(A1);
  //
  
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

  //Serial.println("Checking for sudden movement.");
  
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

  Serial.print("COUNTER MOVEMENT ON SHOULDER: ");Serial.println(no_mov_count);
  return return_value;
  
}

bool transition_shoulder_big_noise(){

  int sound;

  sound = analogRead(soundPin);

  if(sound > sound_th){
    Serial.print("I'M SCARED -> SOUND RECORDED: ");Serial.println(sound);
    set_fear();
    display_state();
    set_neutral();
    display_state();
    return true;
  }
  else{
    Serial.print("SOUND RECORDED:");Serial.println(sound);
    return false;
  }
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

    //servo_head_vertical.attach(servo_vert);
    servo_head_vertical.write(90);

    delay(100);

    servo_head_ears.write(90);

    delay(100);

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

    for(int i = 90; i < 120; i++){
      servo_head_vertical.write(i);
      delay(10);
    }

    delay(100);
    
    Serial.println("SAD sound");
    myDFPlayer.play(6);

    delay(100);

    /*for(int i = 90; i < 150; i++){
      servo_head_ears.write(i);
      delay(10);
    }*/

    for(int i = 90; i > 30; i--){
      servo_head_ears.write(i);
      delay(10);
    }

    delay(3000);
    
  }

  else if(state.angry){
    Serial.println("Angry");
    RGB_color(255, 0, 0);
    
    digitalWrite(vibr_pin, HIGH);

    delay(500);

    digitalWrite(vibr_pin, LOW);    

    delay(100);

    servo_head_ears.write(30);

    //servo_head_horizontal.write(30);

    delay(100);

    myDFPlayer.play(1);

    delay(100);

    servo_head_vertical.write(30);

    delay(4000);
  }

  else if(state.fear){
    Serial.println("Fear");
    RGB_color(255, 0, 255);

    digitalWrite(vibr_pin, HIGH);

    int starting_position = 90;
    int ang = 10;
    int del = 100;

    int ang_incr = 5; //10;
    int del_incr = 100;

    servo_head_ears.write(150);

    for(int i = 0; i < 3; i++){
      for(int j = 0; j < 5; j++){
        
        servo_head_horizontal.write(starting_position + ang);
        delay(del);
        servo_head_horizontal.write(starting_position - ang);
        delay(del);
  
        ang = ang + ang_incr;
        //del = del + del_incr;
      }
      delay(del);
      myDFPlayer.play(3);
      delay(del);
      ang = 10;
      //del = 100;
    }
    
    digitalWrite(vibr_pin, LOW);

  }

  else{
    Serial.println("Neutral");
    
    RGB_color(0, 0, 0);

    delay(100);
    
    //servo_head_vertical.attach(servo_vert);
    servo_head_vertical.write(90);

    delay(100);
    
    //servo_head_horizontal.attach(servo_horiz);
    servo_head_horizontal.write(90);
    
    delay(100);
    
    //servo_head_ears.attach(servo_ears);
    servo_head_ears.write(90);
    
    return ;
  }
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
