
#include "Lab3.h"

Lab3* Lab3::instance = nullptr;

// CONSTRUCTEUR

Lab3::Lab3()
    : ultraSensor(PORT_10),
      led(0, LEDNUM),
      gyro(0, 0x69),
      encoderRight(SLOT1),
      encoderLeft(SLOT2),
      SENS_PIVOT((CHIFFRE_DA % 2 == 0) ? -1 : 1),
      distanceX(150.0),
      distanceY(30.0),
      anticipation(1.5),
      vitesse(90),
      vitesseLente(55),
      vitesseApproche(35),
      vitessePivot(60),
      tolerance(2) {

    instance = this;

    state = DEPART;
    precedentState = DEPART;
    prochainEtatPivot = FIN;

    distanceSeg1 = 0;
    distanceSeg2 = 0;
    distanceSeg3 = 0;

    premierDepart = true;
    premierSegment1 = true;
    premierSegment2 = true;
    premierSegment3 = true;
    premierPivot = true;

    premierRetourSeg3 = true;
    premierRetourSeg2 = true;
    premierRetourSeg1 = true;

    premierFin = true;

    posDebutG = 0;
    posDebutD = 0;

    dernierTempsUltrason = 0;
    derniereDistance = 100;

    zAngleGoal = 0;
    previousError = 0;
    integrale = 0;
    dernierTempsPID = 0;

    angleCible = 0;
    integralePivot = 0;
    dernierAngle = 0;

    debutStable = 0;
    dernierTempsPivot = 0;
    sansMouvement = 0;

    ledsFixes = 0;
    derniereBarre = -1;
    derniersLedsFixes = -1;

    debutDepart = 0;
    dernierNbDepart = -1;

    dernierClignotement = 0;
    ledFinAllumee = false;
}

// INITIALISATION

void Lab3::initialiser() {

    // Reinitialiser la machine interne
    state = DEPART;
    precedentState = DEPART;

    premierDepart = true;
    premierSegment1 = true;
    premierSegment2 = true;
    premierSegment3 = true;
    premierPivot = true;

    premierRetourSeg3 = true;
    premierRetourSeg2 = true;
    premierRetourSeg1 = true;

    premierFin = true;

    distanceSeg1 = 0;
    distanceSeg2 = 0;
    distanceSeg3 = 0;

    ledsFixes = 0;

    // Configuration du materiel
    led.setpin(LED_PIN);

    encoderConfig();

    gyro.begin();

    offMotors();
    offLeds();
}

// INTERRUPTIONS DES ENCODEURS

void Lab3::rightEncoderInterrupt() {

    if (instance == nullptr) return;

    if (digitalRead(instance->encoderRight.getPortB()) == 0) {
        instance->encoderRight.pulsePosMinus();
    }
    else {
        instance->encoderRight.pulsePosPlus();
    }
}


void Lab3::leftEncoderInterrupt() {

    if (instance == nullptr) return;

    if (digitalRead(instance->encoderLeft.getPortB()) == 0) {
        instance->encoderLeft.pulsePosMinus();
    }
    else {
        instance->encoderLeft.pulsePosPlus();
    }
}


// ==========================================
// CONFIGURATION DES ENCODEURS
// ==========================================

void Lab3::encoderConfig() {

    attachInterrupt(
        encoderRight.getIntNum(),
        rightEncoderInterrupt,
        RISING
    );

    attachInterrupt(
        encoderLeft.getIntNum(),
        leftEncoderInterrupt,
        RISING
    );

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


// ==========================================
// TACHES MATERIELLES
// ==========================================

void Lab3::gyroTask() {

    gyro.update();
}


void Lab3::encodersTask() {

    encoderRight.loop();
    encoderLeft.loop();
}


// ==========================================
// ARRET DES MOTEURS ET LEDS
// ==========================================

void Lab3::offMotors() {

    encoderLeft.setMotorPwm(0);
    encoderRight.setMotorPwm(0);
}


void Lab3::offLeds() {

    led.setColor(0, 0, 0);
    led.show();
}


void Lab3::arret() {

    // Arret non bloquant
    offMotors();
}


// ==========================================
// MESURE DES DISTANCES AVEC ENCODEURS
// ==========================================

void Lab3::debuterMesure() {

    posDebutG = encoderLeft.getCurPos();
    posDebutD = encoderRight.getCurPos();
}


float Lab3::distanceFaite() {

    long ecartG = encoderLeft.getCurPos() - posDebutG;
    long ecartD = encoderRight.getCurPos() - posDebutD;

    float posMoy = (abs(ecartG) + abs(ecartD)) / 2.0;

    return (posMoy / 360.0) * (CIRC_WHEEL / 10.0);
}


// ==========================================
// CAPTEUR ULTRASON
// ==========================================

int Lab3::retournerDistance(unsigned long ct) {

    if (ct - dernierTempsUltrason < 60) {
        return derniereDistance;
    }

    dernierTempsUltrason = ct;

    int d = ultraSensor.distanceCm(150);

    if (d > 0) {
        derniereDistance = d;
    }

    return derniereDistance;
}


// ==========================================
// GYROSCOPE
// ==========================================

double Lab3::ecartAngle(double a, double b) {

    double e = a - b;

    while (e > 180) e -= 360;
    while (e < -180) e += 360;

    return e;
}


// ==========================================
// DEPLACEMENT EN LIGNE DROITE AVEC PID
// ==========================================

void Lab3::goStraight(short speed, bool firstRun) {

    const double kp = 6.5;
    const double ki = 0.05;
    const double kd = 1.1;

    if (firstRun) {

        gyro.resetData();

        zAngleGoal = gyro.getAngleZ();

        previousError = 0;
        integrale = 0;

        dernierTempsPID = millis();

        encoderLeft.setMotorPwm(20);
        encoderRight.setMotorPwm(-20);

        return;
    }

    if (millis() - dernierTempsPID < 10) {
        return;
    }

    dernierTempsPID = millis();

    double error = ecartAngle(
        gyro.getAngleZ(),
        zAngleGoal
    );

    integrale = constrain(
        integrale + error,
        -100,
        100
    );

    double output =
        kp * error +
        ki * integrale +
        kd * (error - previousError);

    output = constrain(output, -60, 60);

    previousError = error;

    encoderLeft.setMotorPwm(speed - output);
    encoderRight.setMotorPwm(-speed - output);
}


// ==========================================
// PIVOT AVEC PID
// ==========================================

bool Lab3::spin(short vitesseMax, int targetAngle, bool firstRun) {

    const double kpPivot = 1.8;
    const double kiPivot = 0.20;

    const short pwmMin = 40;
    const short pwmBoost = 80;

    if (firstRun) {

        gyro.resetData();

        angleCible = gyro.getAngleZ() + targetAngle;

        integralePivot = 0;
        debutStable = 0;

        dernierTempsPivot = millis();

        dernierAngle = gyro.getAngleZ();

        sansMouvement = millis();

        return false;
    }

    unsigned long now = millis();

    double dt = (now - dernierTempsPivot) / 1000.0;

    if (dt <= 0) dt = 0.01;

    dernierTempsPivot = now;

    double reste = ecartAngle(
        angleCible,
        gyro.getAngleZ()
    );

    if (fabs(gyro.getAngleZ() - dernierAngle) < 0.5) {

        if (now - sansMouvement > 250) {
            integralePivot += 20;
        }

    }
    else {

        sansMouvement = now;
        dernierAngle = gyro.getAngleZ();
    }

    if (fabs(reste) <= tolerance) {

        offMotors();

        if (debutStable == 0) {
            debutStable = now;
        }

        return (now - debutStable >= 150);
    }

    debutStable = 0;

    integralePivot += reste * dt;

    integralePivot = constrain(
        integralePivot,
        -80,
        80
    );

    double pwm =
        kpPivot * fabs(reste) +
        kiPivot * fabs(integralePivot);

    if (now - sansMouvement > 400) {
        pwm = pwmBoost;
    }

    pwm = constrain(pwm, pwmMin, abs(vitesseMax));

    if (reste < 0) {
        pwm = -pwm;
    }

    encoderLeft.setMotorPwm(pwm);
    encoderRight.setMotorPwm(pwm);

    return false;
}


// ==========================================
// BARRE DE PROGRESSION LED
// ==========================================

int Lab3::barre(float faite, float total) {

    float r = faite / total;

    if (r > 1) r = 1;

    return constrain(
        (int)ceil(r * 7.0f),
        0,
        7
    );
}


// ==========================================
// MISE A JOUR DES LEDS
// ==========================================

void Lab3::majLeds(int nb) {

    if (nb == derniereBarre &&
        ledsFixes == derniersLedsFixes) {
        return;
    }

    derniereBarre = nb;
    derniersLedsFixes = ledsFixes;

    led.setColor(0, 0, 0);

    for (int i = 1; i <= nb; i++) {
        led.setColor(i, 0, 255, 0);
    }

    if (ledsFixes >= 1) {
        led.setColor(12, 0, 255, 0);
    }

    if (ledsFixes >= 2) {
        led.setColor(11, 0, 255, 0);
    }

    if (ledsFixes >= 3) {
        led.setColor(10, 0, 255, 0);
    }

    led.show();
}


// ==========================================
// LEDS ARC-EN-CIEL
// ==========================================

void Lab3::rainbowLeds(unsigned long ct) {

    for (int i = 1; i <= 12; i++) {

        int hue = (ct / 10 + i * 30) % 360;

        int r, g, b;

        int h = hue / 60;
        int f = hue % 60;

        if (h == 0) {
            r = 255;
            g = f * 255 / 60;
            b = 0;
        }
        else if (h == 1) {
            r = 255 - (f * 255 / 60);
            g = 255;
            b = 0;
        }
        else if (h == 2) {
            r = 0;
            g = 255;
            b = f * 255 / 60;
        }
        else if (h == 3) {
            r = 0;
            g = 255 - (f * 255 / 60);
            b = 255;
        }
        else if (h == 4) {
            r = f * 255 / 60;
            g = 0;
            b = 255;
        }
        else {
            r = 255;
            g = 0;
            b = 255 - (f * 255 / 60);
        }

        led.setColor(i, r, g, b);
    }

    led.show();
}


// ==========================================
// ETAT DEPART
// ==========================================

void Lab3::gererDepart(unsigned long ct) {

    if (premierDepart) {

        premierDepart = false;

        debutDepart = ct;
        dernierNbDepart = -1;
    }

    unsigned long ecoule = ct - debutDepart;

    int nb = constrain(ecoule / 250, 0, 12);

    if (nb != dernierNbDepart) {

        dernierNbDepart = nb;

        led.setColor(0, 0, 0);

        for (int i = 1; i <= nb; i++) {
            led.setColor(i, 0, 0, 255);
        }

        led.show();
    }

    if (ecoule >= 3000) {

        offLeds();

        premierDepart = true;

        precedentState = state;
        state = SEGMENT1;
    }
}


// ==========================================
// SEGMENT 1
// ==========================================

void Lab3::gererSegment1(unsigned long ct) {

    if (premierSegment1) {

        premierSegment1 = false;

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

        premierSegment1 = true;

        precedentState = state;
        state = PIVOT;
    }
}


// ==========================================
// SEGMENT 2
// ==========================================

void Lab3::gererSegment2(unsigned long ct) {

    if (premierSegment2) {

        premierSegment2 = false;

        ledsFixes = 2;

        distanceDepartUltrason = retournerDistance(ct);

        debuterMesure();

        goStraight(vitesse, true);
    }

    int d = retournerDistance(ct);

    float total = max(
        1.0f,
        distanceDepartUltrason - distanceY
    );

    majLeds(barre(distanceDepartUltrason - d, total));

    if (d < 40) {
        goStraight(vitesseApproche);
    }
    else if (d < 65) {
        goStraight(vitesseLente);
    }
    else {
        goStraight(vitesse);
    }

    if (d <= distanceY) {

        distanceSeg2 = distanceFaite();

        arret();

        premierSegment2 = true;

        precedentState = state;
        state = PIVOT;
    }
}


// ==========================================
// SEGMENT 3
// ==========================================

void Lab3::gererSegment3(unsigned long ct) {

    if (premierSegment3) {

        premierSegment3 = false;

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

        premierSegment3 = true;

        precedentState = state;
        state = PIVOT;
    }
}


// ==========================================
// PIVOT
// ==========================================

void Lab3::gererPivot(unsigned long ct) {

    if (premierPivot) {

        premierPivot = false;

        if (precedentState == SEGMENT1) {

            anglePivot = SENS_PIVOT * 90;
            prochainEtatPivot = SEGMENT2;
        }
        else if (precedentState == SEGMENT2) {

            anglePivot = -SENS_PIVOT * 90;
            prochainEtatPivot = SEGMENT3;
        }
        else if (precedentState == SEGMENT3) {

            anglePivot = 180;
            prochainEtatPivot = RETOUR_SEG3;
        }
        else if (precedentState == RETOUR_SEG3) {

            anglePivot = SENS_PIVOT * 90;
            prochainEtatPivot = RETOUR_SEG2;
        }
        else if (precedentState == RETOUR_SEG2) {

            anglePivot = -SENS_PIVOT * 90;
            prochainEtatPivot = RETOUR_SEG1;
        }
        else {

            anglePivot = 180;
            prochainEtatPivot = FIN;
        }

        spin(vitessePivot, anglePivot, true);
    }

    majLeds(0);

    if (spin(vitessePivot, anglePivot)) {

        arret();

        premierPivot = true;

        precedentState = state;
        state = prochainEtatPivot;
    }
}


// ==========================================
// RETOUR SEGMENT 3
// ==========================================

void Lab3::gererRetourSeg3(unsigned long ct) {

    if (premierRetourSeg3) {

        premierRetourSeg3 = false;

        debuterMesure();

        goStraight(vitesse, true);
    }

    rainbowLeds(ct);

    goStraight(vitesse);

    if (distanceFaite() >= distanceSeg3 - anticipation) {

        arret();

        premierRetourSeg3 = true;

        precedentState = RETOUR_SEG3;
        state = PIVOT;
    }
}


// ==========================================
// RETOUR SEGMENT 2
// ==========================================

void Lab3::gererRetourSeg2(unsigned long ct) {

    if (premierRetourSeg2) {

        premierRetourSeg2 = false;

        debuterMesure();

        goStraight(vitesse, true);
    }

    rainbowLeds(ct);

    goStraight(vitesse);

    if (distanceFaite() >= distanceSeg2 - anticipation) {

        arret();

        premierRetourSeg2 = true;

        precedentState = RETOUR_SEG2;
        state = PIVOT;
    }
}


// ==========================================
// RETOUR SEGMENT 1
// ==========================================

void Lab3::gererRetourSeg1(unsigned long ct) {

    if (premierRetourSeg1) {

        premierRetourSeg1 = false;

        debuterMesure();

        goStraight(vitesse, true);
    }

    rainbowLeds(ct);

    goStraight(vitesse);

    if (distanceFaite() >= distanceSeg1 - anticipation) {

        arret();

        premierRetourSeg1 = true;

        state = FIN;
    }
}


// ==========================================
// FIN
// ==========================================

void Lab3::gererFin(unsigned long ct) {

    if (premierFin) {

        premierFin = false;

        offMotors();
        offLeds();

        dernierClignotement = ct;
        ledFinAllumee = false;
    }

    if (ct - dernierClignotement >= 50) {

        dernierClignotement = ct;

        ledFinAllumee = !ledFinAllumee;

        for (int i = 8; i <= 12; i++) {

            led.setColor(
                i,
                0,
                ledFinAllumee ? 255 : 0,
                0
            );
        }

        led.show();
    }
}


// ==========================================
// MACHINE A ETATS INTERNE DU LAB3
// ==========================================

void Lab3::manageState(unsigned long ct) {

    switch (state) {

        case DEPART:
            gererDepart(ct);
            break;

        case SEGMENT1:
            gererSegment1(ct);
            break;

        case PIVOT:
            gererPivot(ct);
            break;

        case SEGMENT2:
            gererSegment2(ct);
            break;

        case SEGMENT3:
            gererSegment3(ct);
            break;

        case RETOUR_SEG3:
            gererRetourSeg3(ct);
            break;

        case RETOUR_SEG2:
            gererRetourSeg2(ct);
            break;

        case RETOUR_SEG1:
            gererRetourSeg1(ct);
            break;

        case FIN:
            gererFin(ct);
            break;
    }
}


// ==========================================
// ACTUALISATION DU LAB3
// ==========================================

void Lab3::actualiser() {

    unsigned long ct = millis();

    gyroTask();

    encodersTask();

    manageState(ct);
}

bool Lab3::estTermine() {

    return state == FIN;
}

static Lab3 instanceLab3;

void initialiserLab3() {
    instanceLab3.initialiser();
}

void actualiserLab3() {
    instanceLab3.actualiser();
}

bool lab3EstTermine() {
    return instanceLab3.estTermine();
}
