
#ifndef LAB3_H
#define LAB3_H

#include <Arduino.h>
#include <MeAuriga.h>
#include <math.h>
#include "Configuration.h"

class Lab3 {

private:

    // MATERIEL

    MeUltrasonicSensor ultraSensor;
    MeRGBLed led;
    MeGyro gyro;

    MeEncoderOnBoard encoderRight;
    MeEncoderOnBoard encoderLeft;

    static Lab3* instance;
    static void rightEncoderInterrupt();
    static void leftEncoderInterrupt();

    // PARAMETRES DU PARCOURS

     static const int CHIFFRE_DA = 6;
    const int SENS_PIVOT;
    unsigned long debutRotationCalibration;

    float distanceX;
    float distanceY;

    const float anticipation;

    short vitesse;
    short vitesseLente;
    short vitesseApproche;
    short vitessePivot;
    short tolerance;

    float distanceSeg1;
    float distanceSeg2;
    float distanceSeg3;

    float distanceDepartUltrason;
    float angleDebutCalibration;
    float anglePrecedentCalibration;
    float angleCumuleCalibration;
    bool rotationCalibrationActive;
    bool rotationCalibrationReussie;

    // ETATS INTERNES DU LAB3

    enum State {
        DEPART,
        SEGMENT1,
        PIVOT,
        SEGMENT2,
        SEGMENT3,
        RETOUR_SEG3,
        RETOUR_SEG2,
        RETOUR_SEG1,
        FIN
    };
    
    int anglePivot;
    
    State state;
    State precedentState;
    State prochainEtatPivot;

    // INITIALISATION DES ETATS

    bool premierDepart;
    bool premierSegment1;
    bool premierSegment2;
    bool premierSegment3;

    bool premierPivot;

    bool premierRetourSeg3;
    bool premierRetourSeg2;
    bool premierRetourSeg1;

    bool premierFin;

    unsigned long debutDepart;
    int dernierNbDepart;

    // ENCODEURS

    long posDebutG;
    long posDebutD;

    // ULTRASON

    unsigned long dernierTempsUltrason;
    int derniereDistance;

    // PID DE DEPLACEMENT

    double zAngleGoal;
    double previousError;
    double integrale;
    unsigned long dernierTempsPID;

    // PID DE ROTATION

    double angleCible;
    double integralePivot;
    double dernierAngle;

    unsigned long debutStable;
    unsigned long dernierTempsPivot;
    unsigned long sansMouvement;

    // LEDS

    int ledsFixes;

    int derniereBarre;
    int derniersLedsFixes;

    unsigned long dernierClignotement;
    bool ledFinAllumee;

    // FONCTIONS MATERIELLES

    void encoderConfig();

    void gyroTask();
    void encodersTask();

    void offMotors();
    void offLeds();
    void arret();

    // MESURE DES DISTANCES

    void debuterMesure();
    float distanceFaite();
    int retournerDistance(unsigned long ct);

    // GYROSCOPE ET MOUVEMENTS

    double ecartAngle(double a, double b);

    void goStraight(short speed = 100,
                    bool firstRun = false);

    bool spin(short vitesseMax,
              int targetAngle,
              bool firstRun = false);

    // LEDS

    int barre(float faite, float total);

    void majLeds(int nb);
    void rainbowLeds(unsigned long ct);

    // GESTION DES ETATS

    void gererDepart(unsigned long ct);

    void gererSegment1(unsigned long ct);
    void gererSegment2(unsigned long ct);
    void gererSegment3(unsigned long ct);

    void gererPivot(unsigned long ct);

    void gererRetourSeg3(unsigned long ct);
    void gererRetourSeg2(unsigned long ct);
    void gererRetourSeg1(unsigned long ct);

    void gererFin(unsigned long ct);

    void manageState(unsigned long ct);

public:

    Lab3();

    void initialiser();
    void actualiser();
    bool estTermine();
    void commencerRotationCalibration();
    bool actualiserRotationCalibration();
    void arreterRotationCalibration();
    bool calibrationRotationReussie();

};

#endif
