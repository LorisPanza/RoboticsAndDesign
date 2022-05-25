#include "Arduino.h"
#include "SoftwareSerial.h"
#include "DFRobotDFPlayerMini.h"

SoftwareSerial mySoftwareSerial(12, 13); // RX, TX
DFRobotDFPlayerMini myDFPlayer;

void setup() {
  mySoftwareSerial.begin(9600);
  Serial.begin(115200);

  if (!myDFPlayer.begin(mySoftwareSerial)) {  //Use softwareSerial to communicate with mp3.
    Serial.println(F("Unable to begin:"));
    Serial.println(F("1.Please recheck the connection!"));
    Serial.println(F("2.Please insert the SD card!"));
    while(true){
      delay(0); // Code to compatible with ESP8266 watch dog.
    }
  }
  myDFPlayer.volume(4);  //Set volume value. From 0 to 30
}

void loop() {
  Serial.println("sound1");
  myDFPlayer.play(1);  //Play the first mp3
  delay(3000);
  Serial.println("sound3");
  myDFPlayer.play(3);
  delay(3000);
  Serial.println("sound2");
  myDFPlayer.play(2);
  delay(3000);
}
