void setup() {
  // put your setup code here, to run once:

}

void loop() {
  toggleLED(2);
  toggleLED(4);
  toggleLED(3);
  delay(2000);
}

void toggleLED(int pin) {
  pinMode(pin, OUTPUT);
  if (digitalRead(pin) == HIGH) {
    digitalWrite(pin, LOW);
  } else {
    digitalWrite(pin, HIGH);
  }
}

