#include <Wire.h>

// CONFIGURATION SETUP
/*
void setup() {
  Wire.begin();

  Serial.begin(9600);
  Serial.println("\nI2C Scanner");
}


void loop() {
  byte error, address;
  int nDevices;

  Serial.println("Scanning...");

  nDevices = 0;
  for (address = 1; address < 127; address++ ) {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.print(address, HEX);
      Serial.println("  !");

      nDevices++;
    }

    else if (error == 4) {
      Serial.print("Unknow error at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.println(address, HEX);
    }
  }

  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");

  delay(5000);           // wait 5 seconds for next scan
}
*/

// MPU-6050 Short Example Sketch
#include<Wire.h>

const int MPU = 0x68; // I2C address of the MPU-6050
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
int led_pin = 13;

int16_t threshold_min = 16000;
int16_t threshold_max = 18000;
int stand_counter=0;


void setup() {

  pinMode(led_pin, OUTPUT);
  
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  Serial.begin(9600);
}
void loop() {
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 14, true); // request a total of 14 registers
  AcX = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  AcY = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ = Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp = Wire.read() << 8 | Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX = Wire.read() << 8 | Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY = Wire.read() << 8 | Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ = Wire.read() << 8 | Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)


/*&& -650 < AcY < -100 && -1800 < AcZ < -1400*/
  
  if(threshold_min < AcX && AcX < threshold_max && -650 < AcY && AcY < -100 && -2300 < AcZ && AcZ < -1700){
    //Serial.println("FERMO");
    //Serial.print("X dentro = "); Serial.println(AcX);
    stand_counter = stand_counter+1;

    if(stand_counter==10){

       digitalWrite(led_pin, HIGH);
       delay(500);      
       stand_counter=0;
      
      }
     //delay(500);
    //digitalWrite(led_pin, LOW);
  }
  else{
    //Serial.println("MOVIMENTO");
    stand_counter=0;
    digitalWrite(led_pin, LOW);
    delay(2000);
  }
  
  Serial.print("Accelerometer: ");
  Serial.print("X  = "); Serial.println(AcX);
  Serial.print("Y = "); Serial.println(AcY);
  Serial.print("Z = "); Serial.println(AcZ);
  
  //equation for temperature in degrees C from datasheet
  /*Serial.print("Temperature: "); Serial.print(Tmp / 340.00 + 36.53); Serial.println(" C ");

  Serial.print("Gyroscope: ");
  Serial.print("X = "); Serial.print(GyX);
  Serial.print(" | Y = "); Serial.print(GyY);
  Serial.print(" | Z = "); Serial.println(GyZ);
  */
  Serial.println(" ");
  delay(333);
}
