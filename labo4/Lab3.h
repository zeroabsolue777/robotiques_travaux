#ifndef LAB3_H
#define LAB3_H

#include <Arduino.h>
#include <MeAuriga.h>
#include <math.h>
#include "Configuration.h"

class Lab3 {
private:
    MeUltrasonicSensor ultraSensor;
    MeRGBLed led;
    MeGyro gyro;
    MeEncoderOnBoard encoderRight;
    MeEncoderOnBoard encoderLeft;

    static Lab3* instance;
    static void rightEncoderInterrupt();
    static void leftEncoderInterrupt();

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

    enum State {
        DEPART, SEGMENT1, PIVOT, SEGMENT2, SEGMENT3,
        RETOUR_SEG3, RETOUR_SEG2, RETOUR_SEG1, FIN
    };

    int anglePivot;
    State state;
    State precedentState;
    State prochainEtatPivot;

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
    long posDebutG;
    long posDebutD;
    unsigned long dernierTempsUltrason;
    int derniereDistance;

    double zAngleGoal;
    double previousError;
    double integrale;
    unsigned long dernierTempsPID;
    double angleCible;
    double integralePivot;
    double dernierAngle;
    unsigned long debutStable;
    unsigned long dernierTempsPivot;
    unsigned long sansMouvement;

    float angleDebutPivot180;
    bool pivot180Actif;

    int ledsFixes;
    int derniereBarre;
    int derniersLedsFixes;
    unsigned long dernierClignotement;
    bool ledFinAllumee;

    void encoderConfig();
    void gyroTask();
    void encodersTask();
    void offMotors();
    void offLeds();
    void arret();
    void debuterMesure();
    float distanceFaite();
    int retournerDistance(unsigned long ct);
    double ecartAngle(double a, double b);
    void goStraight(short speed = 100, bool firstRun = false);
    bool spin(short vitesseMax, int targetAngle, bool firstRun = false);
    int barre(float faite, float total);
    void majLeds(int nb);
    void rainbowLeds(unsigned long ct);
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
    void arreterLab3();
    void commencerRotationCalibration();
    bool actualiserRotationCalibration();
    void arreterRotationCalibration();
    bool calibrationRotationReussie();
    void commanderMoteursLigne(int vitesseGauche, int vitesseDroite);
    void arreterMoteursLigne();
    void commencerPivot180();
    bool actualiserPivot180();
    void arreterPivot180();
    void afficherCouleurEtat(int rouge, int vert, int bleu);
    void eteindreLedsEtat();
    void initialiserLedsEtat();
};

#endif
