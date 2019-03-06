volatile byte half_revolutions;
unsigned int rpm;
unsigned long timeold;

void setup() {
    Serial.begin(115200);
    attachInterrupt(0, magnet_detect, RISING);
    half_revolutions = 0;
    rpm = 0;
    timeold = 0;
}

void loop() {
   
}

void magnet_detect() {
    half_revolutions++;
    Serial.println("detect");
}