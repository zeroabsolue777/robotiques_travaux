#include <MeAuriga.h>

MeUltrasonicSensor ultraSensor(PORT_10);

#define LEDNUM 12
#define LED_PIN 44
#define PULSE 9
#define RATIO 39.267
#define CIRC_WHEEL 202.6

MeRGBLed led(0, LEDNUM);
MeGyro gyro(0, 0x69);
MeEncoderOnBoard encoderRight(SLOT1);
MeEncoderOnBoard encoderLeft(SLOT2);

// ---------- CONFIG ETUDIANT ----------
const int CHIFFRE_DA = 6;
const int SENS_PIVOT = (CHIFFRE_DA % 2 == 0) ? -1 : 1;  // pair = gauche

// ---------- PARAMETRES PARCOURS ----------
float distanceX = 150.0;  // distance prof le jour J
float distanceY = 30.0;
const float anticipation = 1.5;
short vitesse = 90;
short vitesseLente = 55;
short vitesseApproche = 35;
short vitessePivot = 60;
short tolerance = 2;

// Distances réelles pour retour précis
float distanceSeg1 = 0;
float distanceSeg2 = 0;
float distanceSeg3 = 0;

// ---------- FSM ----------
enum State { DEPART,
             SEGMENT1,
             PIVOT,
             SEGMENT2,
             SEGMENT3,
             RETOUR_SEG3,
             RETOUR_SEG2,
             RETOUR_SEG1,
             FIN };
State state = DEPART;
State precedentState = DEPART;

// ---------- HARDWARE ----------
void rightEncoderInterrupt() {
  if (digitalRead(encoderRight.getPortB()) == 0) encoderRight.pulsePosMinus();
  else encoderRight.pulsePosPlus();
}
void leftEncoderInterrupt() {
  if (digitalRead(encoderLeft.getPortB()) == 0) encoderLeft.pulsePosMinus();
  else encoderLeft.pulsePosPlus();
}

void encoderConfig() {
  attachInterrupt(encoderRight.getIntNum(), rightEncoderInterrupt, RISING);
  attachInterrupt(encoderLeft.getIntNum(), leftEncoderInterrupt, RISING);
  encoderRight.setPulse(PULSE);
  encoderLeft.setPulse(PULSE);
  encoderRight.setRatio(RATIO);
  encoderLeft.setRatio(RATIO);
  encoderRight.setPosPid(1.8, 0, 1.2);
  encoderLeft.setPosPid(1.8, 0, 1.2);
  encoderRight.setSpeedPid(0.18, 0, 0);
  encoderLeft.setSpeedPid(0.18, 0, 0);
  TCCR1A = _BV(WGM10);
  TCCR1B = _BV(CS11) | _BV(WGM12);
  TCCR2A = _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS21);
}
void gyroTask() {
  gyro.update();
}
void encodersTask() {
  encoderRight.loop();
  encoderLeft.loop();
}
void offMotors() {
  encoderLeft.setMotorPwm(0);
  encoderRight.setMotorPwm(0);
}
void offLeds() {
  led.setColor(0, 0, 0);
  led.show();
}
void arret(unsigned long ms = 300) {
  offMotors();
  unsigned long t = millis();
  while (millis() - t < ms) gyro.update();
}

// ---------- MESURE DISTANCE ENCODEURS ----------
long posDebutG = 0, posDebutD = 0;
void debuterMesure() {
  posDebutG = encoderLeft.getCurPos();
  posDebutD = encoderRight.getCurPos();
}
float distanceFaite() {
  long ecartG = encoderLeft.getCurPos() - posDebutG;
  long ecartD = encoderRight.getCurPos() - posDebutD;
  float posMoy = (abs(ecartG) + abs(ecartD)) / 2.0;
  return (posMoy / 360.0) * (CIRC_WHEEL / 10.0);
}

// ---------- ULTRASON ----------
int retournerDistance(unsigned long ct) {
  static unsigned long lastTime = 0;
  static int last = 100;
  if (ct - lastTime < 60) return last;
  lastTime = ct;
  int d = ultraSensor.distanceCm(150);
  if (d > 0) last = d;
  return last;
}

// ---------- GYROSCOPE ----------
double ecartAngle(double a, double b) {
  double e = a - b;
  while (e > 180) e -= 360;
  while (e < -180) e += 360;
  return e;
}

void goStraight(short speed = 100, bool firstRun = false) {
  static double zAngleGoal = 0, previousError = 0, integrale = 0;
  static unsigned long lastTime = 0;
  const double kp = 6.5, ki = 0.05, kd = 1.1;
  if (firstRun) {
    gyro.resetData();
    zAngleGoal = gyro.getAngleZ();
    previousError = 0;
    integrale = 0;
    lastTime = millis();
    encoderLeft.setMotorPwm(20);
    encoderRight.setMotorPwm(-20);
    return;
  }
  if (millis() - lastTime < 10) return;
  lastTime = millis();
  double error = ecartAngle(gyro.getAngleZ(), zAngleGoal);
  integrale = constrain(integrale + error, -100, 100);
  double output = kp * error + ki * integrale + kd * (error - previousError);
  output = constrain(output, -60, 60);
  previousError = error;
  encoderLeft.setMotorPwm(speed - output);
  encoderRight.setMotorPwm(-speed - output);
}

// PIVOT AVEC INTEGRALE - ne bloque plus
bool spin(short vitesseMax, int targetAngle, bool firstRun = false) {
  static double zAngleGoal = 0, integrale = 0, lastAngle = 0;
  static unsigned long debutStable = 0, lastTime = 0, sansMouvement = 0;
  const double kpPivot = 1.8, kiPivot = 0.20;
  const short pwmMin = 40, pwmBoost = 80;

  if (firstRun) {
    gyro.resetData();
    delay(80);
    zAngleGoal = gyro.getAngleZ() + targetAngle;
    integrale = 0;
    debutStable = 0;
    lastTime = millis();
    lastAngle = gyro.getAngleZ();
    sansMouvement = millis();
    return false;
  }
  unsigned long now = millis();
  double dt = (now - lastTime) / 1000.0;
  if (dt <= 0) dt = 0.01;
  lastTime = now;
  double reste = ecartAngle(zAngleGoal, gyro.getAngleZ());

  if (abs(gyro.getAngleZ() - lastAngle) < 0.5) {
    if (now - sansMouvement > 250) integrale += 20;
  } else {
    sansMouvement = now;
    lastAngle = gyro.getAngleZ();
  }

  if (abs(reste) <= tolerance) {
    offMotors();
    if (debutStable == 0) debutStable = now;
    return (now - debutStable >= 150);
  }
  debutStable = 0;
  integrale += reste * dt;
  integrale = constrain(integrale, -80, 80);
  double pwm = kpPivot * abs(reste) + kiPivot * abs(integrale);
  if (now - sansMouvement > 400) pwm = pwmBoost;
  pwm = constrain(pwm, pwmMin, abs(vitesseMax));
  if (reste < 0) pwm = -pwm;
  encoderLeft.setMotorPwm(pwm);
  encoderRight.setMotorPwm(pwm);
  return false;
}

// ---------- LEDS ----------
int ledsFixes = 0;
int barre(float faite, float total) {
  float r = faite / total;
  if (r > 1) r = 1;
  return constrain((int)ceil(r * 7.0f), 0, 7);
}

void majLeds(int nb) {
  static int derBarre = -1, derFixes = -1;
  if (nb == derBarre && ledsFixes == derFixes) return;
  derBarre = nb;
  derFixes = ledsFixes;
  led.setColor(0, 0, 0);
  for (int i = 1; i <= nb; i++) led.setColor(i, 0, 255, 0);
  if (ledsFixes >= 1) led.setColor(12, 0, 255, 0);
  if (ledsFixes >= 2) led.setColor(11, 0, 255, 0);
  if (ledsFixes >= 3) led.setColor(10, 0, 255, 0);
  led.show();
}

void rainbowLeds(unsigned long ct) {
  for (int i = 1; i <= 12; i++) {
    int hue = (ct / 10 + i * 30) % 360;
    int r, g, b;
    int h = hue / 60;
    int f = hue % 60;
    if (h == 0) {
      r = 255;
      g = f * 255 / 60;
      b = 0;
    } else if (h == 1) {
      r = 255 - (f * 255 / 60);
      g = 255;
      b = 0;
    } else if (h == 2) {
      r = 0;
      g = 255;
      b = f * 255 / 60;
    } else if (h == 3) {
      r = 0;
      g = 255 - (f * 255 / 60);
      b = 255;
    } else if (h == 4) {
      r = f * 255 / 60;
      g = 0;
      b = 255;
    } else {
      r = 255;
      g = 0;
      b = 255 - (f * 255 / 60);
    }
    led.setColor(i, r, g, b);
  }
  led.show();
}

// ---------- ETATS ----------
void gererDepart(unsigned long ct) {
  static bool first = true;
  static unsigned long debut = 0;
  static int dernierNb = -1;
  if (first) {
    first = false;
    debut = ct;
  }
  unsigned long ecoule = ct - debut;
  int nb = constrain(ecoule / 250, 0, 12);
  if (nb != dernierNb) {
    dernierNb = nb;
    led.setColor(0, 0, 0);
    for (int i = 1; i <= nb; i++) led.setColor(i, 0, 0, 255);
    led.show();
  }
  if (ecoule >= 3000) {
    offLeds();
    first = true;
    precedentState = state;
    state = SEGMENT1;
  }
}

void gererSegment1(unsigned long ct) {
  static bool first = true;
  if (first) {
    first = false;
    ledsFixes = 1;
    debuterMesure();
    goStraight(vitesse, true);
  }
  float faite = distanceFaite();
  majLeds(barre(faite, distanceX));
  goStraight(vitesse);
  if (faite >= distanceX - anticipation) {
    distanceSeg1 = faite;
    arret();
    first = true;
    precedentState = state;
    state = PIVOT;
  }
}

void gererSegment2(unsigned long ct) {
  static bool first = true;
  static float dDepart = 100;
  if (first) {
    first = false;
    ledsFixes = 2;
    dDepart = retournerDistance(ct);
    debuterMesure();
    goStraight(vitesse, true);
  }
  int d = retournerDistance(ct);
  float total = max(1.0f, dDepart - distanceY);
  majLeds(barre(dDepart - d, total));
  if (d < 40) goStraight(vitesseApproche);
  else if (d < 65) goStraight(vitesseLente);
  else goStraight(vitesse);
  if (d <= distanceY) {
    distanceSeg2 = distanceFaite();
    arret();
    first = true;
    precedentState = state;
    state = PIVOT;
  }
}

void gererSegment3(unsigned long ct) {
  static bool first = true;
  if (first) {
    first = false;
    ledsFixes = 3;
    debuterMesure();
    goStraight(vitesse, true);
  }
  float faite = distanceFaite();
  majLeds(barre(faite, distanceX));
  goStraight(vitesse);
  if (faite >= distanceX - anticipation) {
    distanceSeg3 = faite;
    arret();
    first = true;
    precedentState = state;
    state = PIVOT;
  }
}

void gererPivot(unsigned long ct) {
  static bool first = true;
  static int angle = 0;
  static State suivant = FIN;
  if (first) {
    first = false;
    if (precedentState == SEGMENT1) {
      angle = SENS_PIVOT * 90;
      suivant = SEGMENT2;
    } else if (precedentState == SEGMENT2) {
      angle = -SENS_PIVOT * 90;
      suivant = SEGMENT3;
    } else if (precedentState == SEGMENT3) {
      angle = 180;
      suivant = RETOUR_SEG3;
    } else if (precedentState == RETOUR_SEG3) {
      angle = SENS_PIVOT * 90;
      suivant = RETOUR_SEG2;
    } else if (precedentState == RETOUR_SEG2) {
      angle = -SENS_PIVOT * 90;
      suivant = RETOUR_SEG1;
    } else {
      angle = 180;
      suivant = FIN;
    }
    spin(vitessePivot, angle, true);
  }
  majLeds(0);
  if (spin(vitessePivot, angle)) {
    arret(300);
    first = true;
    precedentState = state;
    state = suivant;
  }
}

void gererRetourSeg3(unsigned long ct) {
  static bool first = true;
  if (first) {
    first = false;
    debuterMesure();
    goStraight(vitesse, true);
  }
  rainbowLeds(ct);
  goStraight(vitesse);
  if (distanceFaite() >= distanceSeg3 - anticipation) {
    arret();
    first = true;
    precedentState = RETOUR_SEG3;
    state = PIVOT;
  }
}

void gererRetourSeg2(unsigned long ct) {
  static bool first = true;
  if (first) {
    first = false;
    debuterMesure();
    goStraight(vitesse, true);
  }
  rainbowLeds(ct);
  goStraight(vitesse);
  if (distanceFaite() >= distanceSeg2 - anticipation) {
    arret();
    first = true;
    precedentState = RETOUR_SEG2;
    state = PIVOT;
  }
}

void gererRetourSeg1(unsigned long ct) {
  static bool first = true;
  if (first) {
    first = false;
    debuterMesure();
    goStraight(vitesse, true);
  }
  rainbowLeds(ct);
  goStraight(vitesse);
  if (distanceFaite() >= distanceSeg1 - anticipation) {
    arret();
    first = true;
    state = FIN;
  }
}

void gererFin(unsigned long ct) {
  static bool first = true;
  static unsigned long der = 0;
  static bool allume = false;
  if (first) {
    first = false;
    offMotors();
    offLeds();
  }
  if (ct - der >= 50) {
    der = ct;
    allume = !allume;
    for (int i = 8; i <= 12; i++) led.setColor(i, 0, allume ? 255 : 0, 0);
    led.show();
  }
}

void manageState(unsigned long ct) {
  switch (state) {
    case DEPART: gererDepart(ct); 
    break;
    case SEGMENT1: gererSegment1(ct); 
    break;
    case PIVOT: gererPivot(ct); 
    break;
    case SEGMENT2: gererSegment2(ct); 
    break;
    case SEGMENT3: gererSegment3(ct); 
    break;
    case RETOUR_SEG3: gererRetourSeg3(ct); 
    break;
    case RETOUR_SEG2: gererRetourSeg2(ct); 
    break;
    case RETOUR_SEG1: gererRetourSeg1(ct); 
    break;
    case FIN: gererFin(ct); 
    break;
  }
}

void setup() {
  Serial.begin(115200);
  led.setpin(LED_PIN);
  encoderConfig();
  gyro.begin();
  offLeds();
}
void loop() {
  unsigned long ct = millis();
  gyroTask();
  encodersTask();
  manageState(ct);
}