
#include "Robot.h"

// CONSTRUCTEUR
Robot::Robot()
    : etatActuel(ATTENTE),
      etatPrecedent(ATTENTE),
      tempsDebut(0),
      estCalibre(false){
}

// INITIALISATION

void Robot::initialiser() {

    Serial.begin(BAUD_RATE);
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

    // Le robot attend une commande Bluetooth.
    // Nous ajouterons la réception Bluetooth
    // dans une prochaine étape.

}
// void Robot::executerAttente() {
//     static bool testLab3Effectue = false;

//     if (!testLab3Effectue) {
//         testLab3Effectue = true;
//         changerEtat(LAB3);
//     }
// }

void Robot::executerLab3() {
    actualiserLab3();

    if (lab3EstTermine()) {
        changerEtat(ATTENTE);
    }
}

void Robot::executerCalibration() {
    // Calibration des capteurs plus tard.
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
