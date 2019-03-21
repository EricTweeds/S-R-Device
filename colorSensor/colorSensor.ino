#define S0 50
#define S1 52
#define S2 53
#define S3 51
#define CSLights 49
#define sensorOut 12

#define UNKNOWN '?'
#define WATER 'W'
#define PERSON 'P'
#define GROUP 'A'
#define SAND 'S'
#define ROCK 'R'
#define CANDLE 'C'

struct RGB {
  int R;
  int G;
  int B;
  int C;
  float sum;
};

void setup() {
  Serial.begin(9600);
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(sensorOut, INPUT);
  digitalWrite(S0,HIGH);
  digitalWrite(S1,LOW);
  setCSLights(true);
}

void loop() {
  RGB colour = getColour();
  logColourRatios(colour);
}

char determineObject(RGB colour) {
  
}

void logColourVal(RGB colour) {
  Serial.print(colour.B);
  Serial.print(" ");
  Serial.print(colour.R);
  Serial.print(" ");
  Serial.print(colour.G);
  Serial.print(" ");
  Serial.print(colour.C);
  Serial.print(" ");
  Serial.println();
}

void logColourRatios(RGB colour) {
  Serial.print((colour.B/colour.sum)*100);
  Serial.print(" ");
  Serial.print((colour.R/colour.sum)*100);
  Serial.print(" ");
  Serial.print((colour.G/colour.sum)*100);
  Serial.print(" ");
  Serial.print((colour.C/colour.sum)*100);
  Serial.print(" ");
  Serial.println();
}

RGB getColour() {
  
  RGB colour;
  //red
  digitalWrite(S2,LOW);
  digitalWrite(S3,LOW);
  int red = pulseIn(sensorOut, LOW);
  colour.R = red;

  delay(100);

  //Green
  digitalWrite(S2,HIGH);
  digitalWrite(S3,HIGH);
  // Reading the output frequency
  int green = pulseIn(sensorOut, LOW);
  colour.G = green;
  delay(100);

  //Blue
  digitalWrite(S2,LOW);
  digitalWrite(S3,HIGH);
  // Reading the output frequency
  int blue = pulseIn(sensorOut, LOW);
  colour.B = blue;
  delay(100);

  digitalWrite(S2, HIGH);
  digitalWrite(S3, LOW);
  int c = pulseIn(sensorOut, LOW);
  colour.C = c;
  int sum = (red + blue + c + green);
  colour.sum = sum*1.0;
  return colour;
}

void setCSLights(bool on) {
  pinMode(CSLights, OUTPUT);
  if (on) {
    digitalWrite(CSLights, HIGH);
  } else {
    digitalWrite(CSLights, LOW);
  }
}
