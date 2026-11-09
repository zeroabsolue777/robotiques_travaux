
#include "Robot.h"

// CONSTRUCTEUR
Robot::Robot()
    : etatActuel(ATTENTE),
      etatPrecedent(ATTENTE),
      tempsDebut(0),
      estCalibre(false),
      calibrationEnCours(false){
}

// INITIALISATION

void Robot::initialiser() {

    Serial.begin(BAUD_RATE);
    calibration.initialiser();
    changerEtat(BOOT);
}

void Robot::actualiser() {
    switch (etatActuel) {
        case BOOT:
            executerBoot();
            break;

        case ATTENTE:
            executerAttente();
            break;

        case LAB3:
            executerLab3();
            break;

        case CALIB:
            executerCalibration();
            break;

        case SUIVRE_LIGNE:
            executerSuiviLigne();
            break;

        case CMD_INVALIDE:
            executerCmdInvalide();
            break;

        case NON_CALIB:
            executerNonCalib();
            break;
    }
}

void Robot::changerEtat(EtatRobot nouvelEtat) {
    etatPrecedent = etatActuel;
    etatActuel = nouvelEtat;

    tempsDebut = millis();

    switch (etatActuel) {
        case BOOT:
            Serial.println("État : BOOT");
            break;

        case ATTENTE:
            Serial.println("État : ATTENTE");
            break;

        case LAB3:
            Serial.println("État : LAB3");
            initialiserLab3();
            break;

        case CALIB:
            Serial.println("État : CALIB");

            calibrationEnCours = calibration.estConnecte();

            if (calibrationEnCours) {
                calibration.commencer();
                commencerRotationCalibration();
            } else {
                Serial.println("Calibration impossible : LyneTracker absent");
            }
            break;

        case SUIVRE_LIGNE:
            Serial.println("État : SUIVRE_LIGNE");
            break;

        case CMD_INVALIDE:
            Serial.println("État : CMD_INVALIDE");
            break;

        case NON_CALIB:
            Serial.println("État : NON_CALIB");
            break;
    }
}

void Robot::executerBoot() {

    if (millis() - tempsDebut >= TEMPS_BOOT) {

        changerEtat(ATTENTE);
    }
}

void Robot::executerAttente() {
    static bool testCalibrationEffectue = false;

    if (!testCalibrationEffectue) {
        testCalibrationEffectue = true;
        changerEtat(CALIB);
    }
}

void Robot::executerLab3() {
    actualiserLab3();

    if (lab3EstTermine()) {
        changerEtat(ATTENTE);
    }
}

void Robot::executerCalibration() {
    if (!calibrationEnCours) {
        changerEtat(ATTENTE);
        return;
    }

    // Lire les capteurs pendant la rotation
    calibration.calibrer();

    // Continuer la rotation jusqu'à 360 degrés
    if (actualiserRotationCalibration()) {
        calibration.calibrer();
        calibration.terminer();

        estCalibre = calibration.estCalibre();
        calibrationEnCours = false;

        changerEtat(ATTENTE);
    }
}

void Robot::executerSuiviLigne() {
    // Suivi de ligne PID plus tard.
}

void Robot::executerCmdInvalide() {
    // Clignotement rouge plus tard.
}

void Robot::executerNonCalib() {
    // Clignotement jaune plus tard.
}
