#include <StateMachine.h>
#include <ADCTouch.h>
#include <MPU6050_tockn.h>

#include<Wire.h>

// [-1121,-1120] --> [0,18]  [-67,-67] --> [0,1] [3649,3650] --> [16364,16386] [91,91] --> [0,1] [12,12] --> [0,1] [-28,-27] --> [-1,2]


const int STATE_DELAY = 500;
const int LED = 13;

MPU6050 mpu6050(Wire);

long timer = 0;

int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;

StateMachine machine = StateMachine();

//State* S0 = machine.addState(&state0);
//State* S1 = machine.addState(&state1);

State* calibration = machine.addState(&state_calibration); //state 0
State* still = machine.addState(&state_still); //state 1
State* shoulder = machine.addState(&state_shoulder); //state 2

static float variation = 0.0;


//int ref0, ref1;
//static int value_off = 0;


void setup() {
  // put your setup code here, to run once:
 Serial.begin(9600);
  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
  

  //pinMode(LED,OUTPUT);
  //randomSeed(A0);

  //S0->addTransition(&transitionS0,S0);
  //S0->addTransition(&transitionS0S1,S1);
  //S1->addTransition(&transitionS1S0,S0);

  //ref0 = ADCTouch.read(A0, 500);    //create reference values to 
  //ref1 = ADCTouch.read(A1, 500);    //account for the capacitance of the pad

}

void loop() {
  // put your main code here, to run repeatedly:
  machine.run();
  delay(STATE_DELAY);
}

void state_calibration(){
  mpu6050.update();

  if(millis() - timer > 200){
    
    Serial.println("=======================================================");
    //Serial.print("temp : ");Serial.println(mpu6050.getTemp());
    Serial.print("accX : ");Serial.print(mpu6050.getAccX());
    Serial.print("\taccY : ");Serial.print(mpu6050.getAccY());
    Serial.print("\taccZ : ");Serial.println(mpu6050.getAccZ());

    /*
    Serial.print("gyroX : ");Serial.print(mpu6050.getGyroX());
    Serial.print("\tgyroY : ");Serial.print(mpu6050.getGyroY());
    Serial.print("\tgyroZ : ");Serial.println(mpu6050.getGyroZ());
  
    Serial.print("accAngleX : ");Serial.print(mpu6050.getAccAngleX());
    Serial.print("\taccAngleY : ");Serial.println(mpu6050.getAccAngleY());
  
    Serial.print("gyroAngleX : ");Serial.print(mpu6050.getGyroAngleX());
    Serial.print("\tgyroAngleY : ");Serial.print(mpu6050.getGyroAngleY());
    Serial.print("\tgyroAngleZ : ");Serial.println(mpu6050.getGyroAngleZ());
    
    Serial.print("angleX : ");Serial.print(mpu6050.getAngleX());
    Serial.print("\tangleY : ");Serial.print(mpu6050.getAngleY());
    Serial.print("\tangleZ : ");Serial.println(mpu6050.getAngleZ());
    */
    
    Serial.println("=======================================================\n");
    
    variation += 
    
    timer = millis();
    
  }
}

void state_still(){
  
}

void state_shoulder(){
  
}


/*
void state0(){
  Serial.println("State 0");
  if(machine.executeOnce){
    Serial.println("Execute Once");
    digitalWrite(LED,LOW);
  }
  Serial.println("\n");
}

void state1(){
  Serial.println("State 1");
  if(machine.executeOnce){
    Serial.println("Execute Once");
    digitalWrite(LED,HIGH);
    machine.transitionTo(S0);
  }
  Serial.println("\n");
}


bool transitionS0S1(){

  int value0 = ADCTouch.read(A0);   //no second parameter
  int value1 = ADCTouch.read(A1);   //   --> 100 samples

  value0 -= ref0;       //remove offset
  value1 -= ref1;

  if(value1 - value_off > 10){
    value_off = value1;
    Serial.println("RETURN TRUE");
    return true;
  }
  else{
    value_off = value1;
    Serial.println("RETURN FALSE");
    return false;    
  }

}

bool transitionS1S0(){

  
}
*/
