#include <ADCTouch.h>

// FILO CAPACITIVO + VIBRAZIONE

int ref0, ref1;     //reference values to remove offset

int vibr_pin = 10;

static int value_off = 0;

void setup() 
{
    // No pins to setup, pins can still be used regularly, although it will affect readings

    Serial.begin(9600);

    ref0 = ADCTouch.read(A0, 500);    //create reference values to 
    ref1 = ADCTouch.read(A1, 500);    //account for the capacitance of the pad

    pinMode(vibr_pin, OUTPUT);
} 

void loop() 
{
    int value0 = ADCTouch.read(A0);   //no second parameter
    int value1 = ADCTouch.read(A1);   //   --> 100 samples

    value0 -= ref0;       //remove offset
    value1 -= ref1;

    

    Serial.print(value0 > 40);    //send (boolean) pressed or not pressed
    Serial.print("\t");           //use if(value > threshold) to get the state of a button

    Serial.print(value1 > 10);
    Serial.print("\t\t");

    Serial.print(value0);             //send actual reading
    Serial.print("\t");
  
    Serial.println(value1);

    if(value1 - value_off > 10){
        digitalWrite(vibr_pin, HIGH);

        delay(2000);
      
        digitalWrite(vibr_pin, LOW);
    }

    value_off = value1;
    
    delay(100);
}
