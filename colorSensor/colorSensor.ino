#define S0 3
#define S1 4
#define S2 5
#define S3 6
#define CSLights 7
#define sensorOut 2

#define UNKNOWN '?'
#define WATER 'W'
#define PERSON 'P'
#define GROUP 'GP'
#define SAND 'S'
#define ROCK 'R'
#define CANDLE 'C'

struct RGB {
  int R;
  int G;
  int B;
  int C;
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
  logColourVal(colour);
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

RGB getColour() {
  
  RGB colour;
  //red
  digitalWrite(S2,LOW);
  digitalWrite(S3,LOW);
  colour.R = pulseIn(sensorOut, LOW);

  delay(100);

  //Green
  digitalWrite(S2,HIGH);
  digitalWrite(S3,HIGH);
  // Reading the output frequency
  colour.G = pulseIn(sensorOut, LOW);

  delay(100);

  //Blue
  digitalWrite(S2,LOW);
  digitalWrite(S3,HIGH);
  // Reading the output frequency
  colour.B = pulseIn(sensorOut, LOW);

  delay(100);

  digitalWrite(S2, HIGH);
  digitalWrite(S3, LOW);
  colour.C = pulseIn(sensorOut, LOW);


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
