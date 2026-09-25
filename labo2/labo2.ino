//2411160 Mon Numéro de DA est Pair

#include <MeAuriga.h>

// ------------------ Capteurs && LEDS ------------------
#define LEDNUM 12
#define LEDPIN 44
MeRGBLed led(PORT0, LEDNUM);
MeUltrasonicSensor capteurDistance(PORT_10);

// ------------------ États ------------------
enum State {FORWARD, SLOWDOWN, AVOID_STOP, AVOID_BACK, AVOID_TURN, RESUME};
State currentState = FORWARD;

// ------------------ Temps ------------------
unsigned long currentTime = 0;
unsigned long previousTime = 0;
unsigned long distancePrevious = 0;
unsigned long printPrevious = 0;

// ------------------ Durées ------------------
const long stopDuration = 2000;        
const long backDuration = 1000;        
const long turnDuration = 1150;       
const long distanceInterval = 100;     
const long printInterval = 250;        

// ------------------ Vitesses ------------------
int cruisePwm = 180;                   
int slowPwm   = cruisePwm / 2;

// ------------------ Moteurs variables ------------------
const int m1_pwm = 11;
const int m1_in1 = 48;
const int m1_in2 = 49;

const int m2_pwm = 10;
const int m2_in1 = 47;
const int m2_in2 = 46;

// ------------------ Setup ------------------
void setup() {
  Serial.begin(115200);

  pinMode(m1_pwm, OUTPUT);
  pinMode(m1_in1, OUTPUT);
  pinMode(m1_in2, OUTPUT);

  pinMode(m2_pwm, OUTPUT);
  pinMode(m2_in1, OUTPUT);
  pinMode(m2_in2, OUTPUT);

  led.setpin(LEDPIN);

  previousTime = millis();
  distancePrevious = millis();
  printPrevious = millis();
}

// ------------------ Loop ------------------
void loop() {
  currentTime = millis();

  // Mesure distance toutes les 100 ms
  static int distance = 999;
  if (currentTime - distancePrevious >= distanceInterval) {
    distancePrevious = currentTime;
    distance = capteurDistance.distanceCm();
  }

  // Affichage la distance toutes les 250 ms
  if (currentTime - printPrevious >= printInterval) {
    printPrevious = currentTime;
    Serial.print("Distance = ");
    Serial.println(distance);
  }

  stateManager(distance);
  ledTask(currentState);
}

// ------------------ Machine à états ------------------
void stateManager(int distance) {
  switch (currentState) {

    case FORWARD:
      DriveForward(cruisePwm);

      if (distance <= 30) {
        currentState = AVOID_STOP;
        previousTime = currentTime;
        Stop();
      }
      else if (distance <= 100) {
        currentState = SLOWDOWN;
      }
      break;

    case SLOWDOWN:
      DriveForward(slowPwm);

      if (distance <= 30) {
        currentState = AVOID_STOP;
        previousTime = currentTime;
        Stop();
      }
      else if (distance >= 100) {
        currentState = FORWARD;
      }
      break;

    case AVOID_STOP:
      Stop();
      if (currentTime - previousTime >= stopDuration) {
        currentState = AVOID_BACK;
        previousTime = currentTime;
      }
      break;

    case AVOID_BACK:
      DriveBackward(cruisePwm);
      if (currentTime - previousTime >= backDuration) {
        currentState = AVOID_TURN;
        previousTime = currentTime;
      }
      break;

    case AVOID_TURN:
      Turn180();
      if (currentTime - previousTime >= turnDuration) {
        currentState = RESUME;
      }
      break;

    case RESUME:
      currentState = FORWARD;
      break;
  }
}

// ------------------ LED Task ------------------
void ledTask(State s) {

  led.setColor(0, 0, 0);

  switch (s) {

    case FORWARD:
      for (int i = 1; i <= 6; i++)
        led.setColor(i, 0, 255, 0);   // vert
      break;

    case SLOWDOWN:
      for (int i = 7; i <= 12; i++)
        led.setColor(i, 255, 255, 0); // jaune
      break;

    case AVOID_STOP:
      for (int i = 1; i <= 12; i++)
        led.setColor(i, 255, 0, 0);   // rouge
      break;

    case AVOID_BACK:
      for (int i = 1; i <= 12; i++)
        led.setColor(i, 255, 0, 0); 
      break;

    case AVOID_TURN:
      for (int i = 1; i <= 12; i++)
        led.setColor(i, 255, 0, 0); 
      break;

    case RESUME:
      for (int i = 1; i <= 6; i++)
        led.setColor(i, 0, 255, 0); 
      break;
  }

  led.show();
}

// ------------------ Fonctions moteurs ------------------

void DriveForward(int pwmValue) {
  digitalWrite(m1_in2, LOW);
  digitalWrite(m1_in1, HIGH);
  analogWrite(m1_pwm, pwmValue);

  digitalWrite(m2_in2, LOW);
  digitalWrite(m2_in1, HIGH);
  analogWrite(m2_pwm, pwmValue);
}

void DriveBackward(int pwmValue) {
  digitalWrite(m1_in2, HIGH);
  digitalWrite(m1_in1, LOW);
  analogWrite(m1_pwm, pwmValue);

  digitalWrite(m2_in2, HIGH);
  digitalWrite(m2_in1, LOW);
  analogWrite(m2_pwm, pwmValue);
}

void Turn180() {
  digitalWrite(m1_in2, HIGH);
  digitalWrite(m1_in1, LOW);
  analogWrite(m1_pwm, cruisePwm);

  digitalWrite(m2_in2, LOW);
  digitalWrite(m2_in1, HIGH);
  analogWrite(m2_pwm, cruisePwm);
}

void Stop() {
  analogWrite(m1_pwm, 0);
  analogWrite(m2_pwm, 0);
}
