int fsrPin = 0;     // the FSR and 10K pulldown are connected to a0
int fsrReading;     // the analog reading from the FSR resistor divider

int red_pin = 9;
int green_pin = 10;
int blue_pin = 11;
 
void setup(void) {
  Serial.begin(9600);

  pinMode(red_pin, OUTPUT);
  pinMode(green_pin, OUTPUT);
  pinMode(blue_pin, OUTPUT);
}
 
void loop(void) {
  fsrReading = analogRead(fsrPin);  
 
  Serial.print("Analog reading = ");
  Serial.print(fsrReading);     // print the raw analog reading
 
  if (fsrReading < 10) {
    Serial.println(" - No pressure -- RED");
    RGB_color(255, 0, 0); //Red
    delay(500);
  } else if (fsrReading < 200) {
    Serial.println(" - Light touch -- GREEN");
    RGB_color(0, 255, 0); //Green
    delay(500);
  } else if (fsrReading < 500) {
    Serial.println(" - Light squeeze -- BLUE");
    RGB_color(0, 0, 255); //Blue
    delay(500);
  } else if (fsrReading < 800) {
    Serial.println(" - Medium squeeze -- MAGENTA");
    RGB_color(255, 0, 255); // Magenta
    delay(500);
  } else {
    Serial.println(" - Big squeeze -- YELLOW");
    RGB_color(255, 255, 0); //Yellow
    delay(500);
  }
  //delay(1000);
}

void RGB_color(int red_light_value, int green_light_value, int blue_light_value)
 {
  analogWrite(red_pin, red_light_value);
  analogWrite(green_pin, green_light_value);
  analogWrite(blue_pin, blue_light_value);
}
