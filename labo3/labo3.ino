#include <MeAuriga.h>

// 1. CONSTANTES
#define LEDNUM 12
#define LED_PIN 44
#define PULSE 9
#define RATIO 39.267
#define CIRC_WHEEL 202.6

const int DA = 2411160;
const float distanceX = 180.0;
const short tolerance = 2;
short vitesse = 100;
short vitessePivot = 80;

// 2. OBJETS
MeUltrasonicSensor ultraSensor(PORT_10);
MeRGBLed led(0, LEDNUM);
MeGyro gyro(0, 0x69);
MeEncoderOnBoard encoderRight(SLOT1);
MeEncoderOnBoard encoderLeft(SLOT2);

// 3. VARIABLES
float distanceParcourue = 0;
float distanceDebutSegment = 0;
unsigned long currentTime = 0;

// 4. ÉTATS
enum State { SEGMENT1,
             PIVOT1,
             SEGMENT2,
             PIVOT2,
             SEGMENT3,
             PIVOT3,
             FIN,
             RETOUR };
State state = SEGMENT1;
State precedentState = SEGMENT1;

// 5. ENCODEURS / GYRO
void rightEncoderInterrupt(void) {
  if (digitalRead(encoderRight.getPortB()) == 0) encoderRight.pulsePosMinus();
  else encoderRight.pulsePosPlus();
}
void leftEncoderInterrupt(void) {
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
void gyroTask(unsigned long ct) {
  gyro.update();
}
void encodersTask(unsigned long ct) {
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

// 6. DISTANCES
float calculDistancePar(unsigned long ct) {
  static unsigned long lastTime = 0;
  static float last = 0;
  if (ct - lastTime < 100) return last;
  lastTime = ct;
  long posL = encoderLeft.getCurPos();
  long posR = encoderRight.getCurPos();
  float posMoy = (abs(posL) + abs(posR)) / 2.0;
  float local = (posMoy / 360.0) * (CIRC_WHEEL / 10.0);
  last = local;
  return local;
}
float retournerDistance(unsigned long ct) {
  static unsigned long lastTime = 0;
  static int last = 0;
  if (ct - lastTime < 250) return last;
  lastTime = ct;
  last = ultraSensor.distanceCm();
  return last;
}

// 7. DEPLACEMENT - TES FONCTIONS D'ORIGINE
void goStraight(short speed = 100, bool firstRun = false) {
  static double zAngleGoal = 0, error = 0, previousError = 0, output = 0;
  const double kp = 6.5, kd = 1.1;
  if (firstRun) {
    gyro.resetData();
    zAngleGoal = gyro.getAngleZ();
    encoderLeft.setMotorPwm(speed);
    encoderRight.setMotorPwm(-speed);
    return;
  }
  error = gyro.getAngleZ() - zAngleGoal;
  output = kp * error + kd * (error - previousError);
  previousError = error;
  encoderLeft.setMotorPwm(speed - output);
  encoderRight.setMotorPwm(-speed - output);
}

bool spin(short speed, int targetAngle, bool firstRun = false) {
  static double zAngleGoal = 0;
  if (firstRun) {
    gyro.resetData();
    zAngleGoal = gyro.getAngleZ() + targetAngle;
    return false;
  }
  encoderLeft.setMotorPwm(speed);
  encoderRight.setMotorPwm(speed);
  // Version d'origine avec abs
  return abs(gyro.getAngleZ() - zAngleGoal) <= tolerance;
}

// 8. DEL
void afficherProgressionSeg1(float faite) {
  led.setColor(0, 0, 0);
  int nb = map(faite, 0, distanceX, 0, 7);
  nb = constrain(nb, 0, 7);
  for (int i = 1; i <= nb; i++) led.setColor(i, 0, 255, 0);
  led.setColor(12, 0, 255, 0);
  led.show();
}

// 9. ETAPES
void gererSegment1(unsigned long ct) {
  static bool firstTime = true;
  if (firstTime) {
    Serial.println("Entrée: SEGMENT1");
    firstTime = false;
    distanceDebutSegment = distanceParcourue;
    goStraight(vitesse, true);
  }
  float faite = distanceParcourue - distanceDebutSegment;
  afficherProgressionSeg1(faite);
  goStraight(vitesse);
  if (faite >= distanceX) {
    Serial.println("Sortie: SEGMENT1 -> PIVOT1");
    offMotors();
    firstTime = true;
    precedentState = SEGMENT1;
    state = PIVOT1;
  }
}

void gererPivot1(unsigned long ct) {
  static bool firstTime = true;
  const int angleCible = -90;
  const short sens = -80;  

  if (firstTime) {
    Serial.println("Entrée: PIVOT1 GAUCHE -90");
    firstTime = false;
    spin(sens, angleCible, true);
  }
  led.setColor(0, 0, 0);
  led.setColor(12, 0, 255, 0);
  led.show();

  if (spin(sens, angleCible)) {
    Serial.println("Sortie: PIVOT1 -> SEGMENT2");
    offMotors();
    firstTime = true;
    precedentState = PIVOT1;
    state = SEGMENT2;
  }
}

void gererSegment2(unsigned long ct) {
  offMotors();
  Serial.println("SEGMENT2 pas encore codé");
  state = FIN;
}
void gererPivot2(unsigned long ct) {}
void gererSegment3(unsigned long ct) {}
void gererPivot3(unsigned long ct) {}
void gererFin(unsigned long ct) {
  offMotors();
  offLeds();
}
void gererRetour(unsigned long ct) {}

// 10. MACHINE À ÉTATS
void manageState(unsigned long ct) {
  switch (state) {
    case SEGMENT1: gererSegment1(ct); break;
    case PIVOT1: gererPivot1(ct); break;
    case SEGMENT2: gererSegment2(ct); break;
    case PIVOT2: gererPivot2(ct); break;
    case SEGMENT3: gererSegment3(ct); break;
    case PIVOT3: gererPivot3(ct); break;
    case FIN: gererFin(ct); break;
    case RETOUR: gererRetour(ct); break;
  }
}

void setup() {
  Serial.begin(115200);
  led.setpin(LED_PIN);
  encoderConfig();
  gyro.begin();
  offLeds();
  Serial.println("Labo03 DA 2411160 - Test SEG1+PIVOT1 GAUCHE");
  delay(3000);
}

void loop() {
  currentTime = millis();
  distanceParcourue = calculDistancePar(currentTime);
  manageState(currentTime);
  gyroTask(currentTime);
  encodersTask(currentTime);
}