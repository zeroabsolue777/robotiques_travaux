#include <MeAuriga.h>

// 1. CONSTANTES
#define LEDNUM 12
#define LED_PIN 44
#define PULSE 9
#define RATIO 39.267
#define CIRC_WHEEL 202.6

// DA 2411160 -> DA[6]=0 PAIR -> GAUCHE
const float distanceX = 180.0; // X donné par le prof le jour J
const int LED_FIXE_SEG1 = 12;

// 2. OBJETS
MeUltrasonicSensor ultraSensor(PORT_10);
MeRGBLed led(0, LEDNUM);
MeGyro gyro(0, 0x69);
MeEncoderOnBoard encoderRight(SLOT1);
MeEncoderOnBoard encoderLeft(SLOT2);

// 3. VARIABLES GLOBALES
float distanceParcourue = 0;
float distanceDebutSegment1 = 0;
short vitesse = 100;
short vitessePivot = 80;
const short tolerance = 2;

// 4. ÉTATS
enum State { SEGMENT1, PIVOT1, SEGMENT2, PIVOT2, SEGMENT3, PIVOT3, FIN, RETOUR };
State state = SEGMENT1;
State precedentState = SEGMENT1;

// 5. ENCODEURS / GYRO - Repris de ton code qui marche
void rightEncoderInterrupt(void){ if(digitalRead(encoderRight.getPortB())==0) encoderRight.pulsePosMinus(); else encoderRight.pulsePosPlus(); }
void leftEncoderInterrupt(void){ if(digitalRead(encoderLeft.getPortB())==0) encoderLeft.pulsePosMinus(); else encoderLeft.pulsePosPlus(); }

void encoderConfig() {
  attachInterrupt(encoderRight.getIntNum(), rightEncoderInterrupt, RISING);
  attachInterrupt(encoderLeft.getIntNum(), leftEncoderInterrupt, RISING);
  encoderRight.setPulse(PULSE);
  encoderLeft.setPulse(PULSE);
  encoderRight.setRatio(RATIO);
  encoderLeft.setRatio(RATIO);
  encoderRight.setPosPid(1.8,0,1.2);
  encoderLeft.setPosPid(1.8,0,1.2);
  encoderRight.setSpeedPid(0.18,0,0);
  encoderLeft.setSpeedPid(0.18,0,0);
  TCCR1A = _BV(WGM10);
  TCCR1B = _BV(CS11) | _BV(WGM12);
  TCCR2A = _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS21);
}
void gyroTask(unsigned long ct){ gyro.update(); }
void encodersTask(unsigned long ct){ encoderRight.loop(); encoderLeft.loop(); }
void offMotors(){ encoderLeft.setMotorPwm(0); encoderRight.setMotorPwm(0); }
void offLeds(){ led.setColor(0,0,0); led.show(); }

float calculDistancePar(unsigned long ct){
  static unsigned long lastTime=0; static float last=0;
  if(ct-lastTime<100) return last; lastTime=ct;
  long posL=encoderLeft.getCurPos(); long posR=encoderRight.getCurPos();
  float posMoy=(abs(posL)+abs(posR))/2.0;
  float local=(posMoy/360.0)*(CIRC_WHEEL/10.0);
  last=local; return local;
}

// 7. DEPLACEMENT - Ton goStraight avec PID
void goStraight(short speed = 100, bool firstRun = false){
  static double zAngleGoal=0, error=0, previousError=0, output=0;
  const double kp=6.5, kd=1.1;

    if(firstRun){ gyro.resetData();
        zAngleGoal=gyro.getAngleZ();
        encoderLeft.setMotorPwm(speed);
        encoderRight.setMotorPwm(-speed);
        return;
    }

  error=gyro.getAngleZ()-zAngleGoal; output=kp*error+kd*(error-previousError);
   previousError=error;
  encoderLeft.setMotorPwm(speed-output);
  encoderRight.setMotorPwm(-speed-output);
}

// 8. DEL - VERSION CONFORME LABO 03
void afficherProgressionSeg1(float faite){
  led.setColor(0,0,0);
  // Barre 1 à 7 qui progresse selon distance
  int nb = map(faite, 0, distanceX, 0, 7);
  nb = constrain(nb, 0, 7);
  for(int i=1; i<=nb; i++){
    led.setColor(i, 0, 255, 0); // VERT
  }
  // LED 12 verte fixe
  led.setColor(12, 0, 255, 0);
  led.show();
}

// 9. SEGMENT1 - À COMPLÉTER
void gererSegment1(unsigned long ct){
  static bool firstTime = true;

  if(firstTime){
    Serial.println("Entrée etat: SEGMENT1 - DA 2411160");
    firstTime = false;
    distanceDebutSegment1 = distanceParcourue; // on mémorise le départ
    goStraight(vitesse, true); // firstRun = true pour reset gyro
  }

  float faite = distanceParcourue - distanceDebutSegment1;

  // Affichage DEL conforme
  afficherProgressionSeg1(faite);

  // Maintien trajectoire droite avec gyro
  goStraight(vitesse);

  // Condition de transition
  bool transition = faite >= distanceX;

  if(transition){
    Serial.println("Sortie etat: SEGMENT1 -> PIVOT1 GAUCHE");
    offMotors();
    firstTime = true; // reset pour la prochaine fois
    precedentState = SEGMENT1;
    state = PIVOT1;
  }
}

// Les autres restent vides pour l'instant
void gererPivot1(unsigned long ct){}
void gererSegment2(unsigned long ct){}
void gererPivot2(unsigned long ct){}
void gererSegment3(unsigned long ct){}
void gererPivot3(unsigned long ct){}
void gererFin(unsigned long ct){}
void gererRetour(unsigned long ct){}
float retournerDistance(unsigned long ct){ return 0; }
void spin(){}
void manageState(unsigned long ct){
  if(state==SEGMENT1) gererSegment1(ct);
}

void setup(){
  Serial.begin(115200);
  led.setpin(LED_PIN);
  encoderConfig();
  gyro.begin();
  offLeds();
  delay(3000);
}

void loop(){
  unsigned long currentTime = millis();
  distanceParcourue = calculDistancePar(currentTime);
  manageState(currentTime);
  gyroTask(currentTime);
  encodersTask(currentTime);
}
