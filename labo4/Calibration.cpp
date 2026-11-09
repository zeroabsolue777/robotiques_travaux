
#include "Calibration.h"

Calibration::Calibration() {
    connecte = false;
    calibre = false;

    for (int i = 0; i < NB_IR; i++) {
        valeurs[i] = 0;
        valeursMin[i] = 1023;
        valeursMax[i] = 0;
    }
}

bool Calibration::initialiser() {
    connecte = ss.begin();

    if (connecte) {
        Serial.println("LyneTracker connecte !");
    } else {
        Serial.println("Erreur de connexion au LyneTracker");
    }

    return connecte;
}

void Calibration::commencer() {
    calibre = false;

    for (int i = 0; i < NB_IR; i++) {
        valeursMin[i] = 1023;
        valeursMax[i] = 0;
    }

    Serial.println("Debut calibration");
}

void Calibration::lireCapteurs() {
    if (!connecte) {
        return;
    }

    for (int i = 0; i < NB_IR; i++) {
        valeurs[i] = ss.analogRead(i);
    }
}

void Calibration::calibrer() {
    if (!connecte) {
        return;
    }

    lireCapteurs();

    for (int i = 0; i < NB_IR; i++) {
        if (valeurs[i] < valeursMin[i]) {
            valeursMin[i] = valeurs[i];
        }

        if (valeurs[i] > valeursMax[i]) {
            valeursMax[i] = valeurs[i];
        }
    }
}

void Calibration::terminer() {
    if (!connecte) {
        return;
    }

    calibre = true;

    Serial.println("Calibration terminee !");
    afficherCalibration();
}

void Calibration::afficherCapteurs() {
    if (!connecte) {
        return;
    }

    for (int i = 0; i < NB_IR; i++) {
        Serial.print("IR");
        Serial.print(i);
        Serial.print(": ");
        Serial.print(valeurs[i]);
        Serial.print("\t");
    }

    Serial.println();
}

void Calibration::afficherCalibration() {
    for (int i = 0; i < NB_IR; i++) {
        Serial.print("IR");
        Serial.print(i);
        Serial.print(" MIN: ");
        Serial.print(valeursMin[i]);
        Serial.print(" MAX: ");
        Serial.println(valeursMax[i]);
    }
}

bool Calibration::estConnecte() {
    return connecte;
}

bool Calibration::estCalibre() {
    return calibre;
}

int Calibration::getMin(int index) {
    if (index < 0 || index >= NB_IR) {
        return 0;
    }

    return valeursMin[index];
}

int Calibration::getMax(int index) {
    if (index < 0 || index >= NB_IR) {
        return 0;
    }

    return valeursMax[index];
}
