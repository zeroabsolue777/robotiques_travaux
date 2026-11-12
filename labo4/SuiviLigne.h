
#ifndef SUIVI_LIGNE_H
#define SUIVI_LIGNE_H

#include <Arduino.h>
#include "Calibration.h"

class SuiviLigne {
private:
    Calibration* calibration;

    // Coefficients PID
    float kp;
    float ki;
    float kd;

    // Variables PID
    float integrale;
    float erreurPrecedente;
    unsigned long dernierPID;

    // Position et correction
    float positionLigne;
    float correction;

    // Filtrage
    float positionFiltree;
    float deriveeFiltree;
    bool premiereLecture;

    // Détection
    bool ligneDetectee;

public:
    SuiviLigne();

    void initialiser(Calibration* capteur);
    void commencer();
    void actualiser();

    float getPosition();
    float getCorrection();
    bool estLigneDetectee();
};

#endif
