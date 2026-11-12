
#ifndef CALIBRATION_H
#define CALIBRATION_H

#include <Arduino.h>
#include <Adafruit_seesaw.h>

#define NB_IR 5

class Calibration {
private:
    Adafruit_seesaw ss;

    int valeurs[NB_IR];
    int valeursMin[NB_IR];
    int valeursMax[NB_IR];

    bool connecte;
    bool calibre;

public:
    Calibration();

    bool initialiser();

    void commencer();
    void lireCapteurs();
    void calibrer();
    void terminer();

    void afficherCapteurs();
    void afficherCalibration();

    bool estConnecte();
    bool estCalibre();

    int getMin(int index);
    int getMax(int index);
    int getValeur(int index);
};

#endif
