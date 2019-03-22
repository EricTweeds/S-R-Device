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
  float R;
  float G;
  float B;
  float C;
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
//  pinMode(45, OUTPUT);
//  digitalWrite(45, HIGH);
  setCSLights(false);
}

void loop() {
  RGB colour = getColour();
  logColourVal(colour);
  //char object = determineObject(colour);
  //Serial.println(object);
  //delay(500);
}

char determineObject(RGB colour) {
  if ((colour.B > 41.5 && colour.G < 32) || colour.G < 31) {
    return PERSON;
  } else if ((colour.B < 36 && colour.G > 35.5 && colour.R < 21) || colour.R < 16.5) {
    return GROUP;
  } else if (colour.C > 11) {
    return CANDLE; 
  } else {
    return UNKNOWN;
  }
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
  
  float sum = (red + blue + c + green)*1.0;

  colour.R = red/sum*100;
  colour.G = green/sum*100;
  colour.B = blue/sum*100;
  colour.C = c/sum*100;
  
  colour.sum = sum;
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
