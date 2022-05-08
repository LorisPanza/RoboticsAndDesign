int vibr_pin = 4;

void setup() {
  // put your setup code here, to run once:
  pinMode(vibr_pin, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:

  

  digitalWrite(vibr_pin, HIGH);

  delay(2000);

  digitalWrite(vibr_pin, LOW);

  delay(2000);

}
