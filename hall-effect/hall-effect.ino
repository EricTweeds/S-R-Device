

void setup() {
    Serial.begin(9600);
    attachInterrupt(0, magnet_detect, RISING);
}

void loop() {
   
}

void magnet_detect() {
    Serial.println("detect");
}
