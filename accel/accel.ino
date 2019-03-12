double dt = 0.01;
double kp = 1.0, ki = 1.0;
double integral = 0.0;
double error[20] = {0};
int index = 0;
double finalSP;
double here;
bool setDt = false;
double startTime;

void setup() {
    finalSP = 120;
    here = 120;
}

double tempTime;

void loop() {
    // if (!setDt) {
    //     startTime = millis();
    // }
    double output = pi(finalSP, here);

    double ratio = map(output, -1, 1, 2, 0.5);
    if (output > 1) {
        analogWrite(enA, 255 / ratio);
        analogWrite(enB, 255);
    } else {
        analogWrite(enA, 255);
        analogWrite(enB, 255 * ratio);
    }
    // if (!setDt) {
    //     dt = millis() - startTime;
    //     setDt = true;
    // }
    here += accel * dt * dt;
}

double pi(double SP, double PV) {
    double output;
    integral -= error[index] * dt;
    error[index] = SP - PV;
    integral += error[index] * dt;
    output = kp * error[index] + ki * integral;
    index = (index + 1) % 20;
}
