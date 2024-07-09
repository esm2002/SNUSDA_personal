int analogPin = A0;

void setup() {
  Serial.begin(9600);
}

void loop() {
  int analog_value = analogRead(analogPin);
  Serial.println(analog_value/4);
  delay(50);
}
