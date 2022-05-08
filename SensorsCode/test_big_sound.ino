

// EXAMPLE 1: LED lights up when noise is above the threshold and immediately lights off.
/*
//**********************************************************************
const int ledPin = 12;    //LED attached to digital pin 12
const int soundPin = A0;  //Sound sensor attached to analog pin A0
int val = 0;              //the value of sound sensor
//*********************************************************************
void setup()
{
  pinMode(soundPin,INPUT);         //set soundPin as INPUT
  pinMode(ledPin,OUTPUT);          //set ledPin as OUTPUT
  Serial.begin(9600);              //initialize serial communications at 9600 bps
}

void loop()
{
  val = analogRead(soundPin);      //read the value of sound sensor
  Serial.println(val);             //print the value of sound sensor on serial monitor
  if(val > 100)                    //if the value is greater than 27
  {
    Serial.println("Reached ");
    digitalWrite(ledPin,HIGH);     //turn the LED on
  }
  else
  {
    Serial.println("Not reached ");
    digitalWrite(ledPin,LOW);      //turn the LED off
  }
  //delay(500);
}
*/



// EXAMPLE 2: LED lights up when noise is above the threshold and stays up until another noise is above the threshold.
/*
//**********************************************************************
const int ledPin = 12;    //LED attached to digital pin 12
const int soundPin = A0;  //Sound sensor attached to analog pin A0
int val = 0;              //the value of sound sensor
boolean state = true;
int threshold = 80;
//*********************************************************************
void setup()
{
  pinMode(soundPin,INPUT);         //set soundPin as INPUT
  pinMode(ledPin,OUTPUT);          //set ledPin as OUTPUT
  Serial.begin(9600);              //initialize serial communications at 9600 bps
}
void loop(){
  val = analogRead(soundPin);

  val = val - 85; // correct the error (it seems that when there is silence the noise level is 85 more or less)

  if(val > threshold && state == false){
    digitalWrite(ledPin, HIGH);
    Serial.print("State FALSE, val: ");
    Serial.println(val);
    state = true;
    delay(1000);
  }
  else if(val > threshold && state == true){
    digitalWrite(ledPin, LOW);
    Serial.print("State TRUE, val: ");
    Serial.println(val);
    state = false;
    delay(1000);
  }
}
*/

// EXAMPLE 3: servo motor moves when noise threshold is exceeded.
#include <Servo.h> 

//**********************************************************************
//const int ledPin = 12;    //LED attached to digital pin 12
const int soundPin = A0;  //Sound sensor attached to analog pin A0
int val = 0;              //the value of sound sensor
boolean state = true;
int threshold = 90;
int servoPin = 4; //control pin of the motor
Servo my_servo;  
//*********************************************************************
void setup()
{
  pinMode(soundPin,INPUT);         //set soundPin as INPUT
  //pinMode(ledPin,OUTPUT);          //set ledPin as OUTPUT

  my_servo.attach(servoPin); //setup the servo motor
  
  Serial.begin(9600);              //initialize serial communications at 9600 bps
}
void loop(){
  val = analogRead(soundPin);

  val = val - 65; // correct the error (it seems that when there is silence the noise level is 85 more or less)

  if(val > threshold /*&& state == false*/){
    //digitalWrite(ledPin, HIGH);
    my_servo.write(90);
    delay(500);
    my_servo.write(0);
    Serial.print("Motor moved, sound val: ");
    Serial.println(val);
    //state = true;
    delay(5000);
    Serial.println("FINE DELAY");
  }
  //delay(5000);
  /*
  else if(val > threshold && state == true){
    //digitalWrite(ledPin, LOW);
    Serial.print("State TRUE, val: ");
    Serial.println(val);
    state = false;
    delay(1000);
  }*/
}
