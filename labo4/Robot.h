
#ifndef ROBOT_H
#define ROBOT_H

#include <Arduino.h>
#include "Configuration.h"

void initialiserLab3();
void actualiserLab3();
bool lab3EstTermine();

// Les 7 états 
enum EtatRobot {
    BOOT,
    ATTENTE,
    LAB3,
    CALIB,
    SUIVRE_LIGNE,
    CMD_INVALIDE,
    NON_CALIB
};

class Robot {

private:

    EtatRobot etatActuel;
    EtatRobot etatPrecedent;

    unsigned long tempsDebut;

    bool estCalibre;

    void changerEtat(EtatRobot nouvelEtat);
    

    // Exécution des états
    void executerBoot();
    void executerAttente();
    void executerLab3();
    void executerCalibration();
    void executerSuiviLigne();
    void executerCmdInvalide();
    void executerNonCalib();

public:

    Robot();

    void initialiser();
    void actualiser();

};

#endif
