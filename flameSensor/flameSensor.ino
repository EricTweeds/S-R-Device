const int minThreshold = 1000;
const int flameThreshold = 100;


void setup() {
  Serial.begin(9600);
}

void loop() {
  int sensorValue = getFlameValue(A0);
  Serial.println(sensorValue);
  Serial.print(" ");
}

int getFlameValue(int pin) {
  int sensorSum = 0;
  for (int i = 0; i < 5; i++) {
    sensorSum += analogRead(pin);
    delay(500);
  }
  return (sensorSum/5);
}
