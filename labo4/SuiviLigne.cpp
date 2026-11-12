
#include "SuiviLigne.h"

SuiviLigne::SuiviLigne() {
    calibration = nullptr;

    // Coefficients PID actuels
    kp = 0.025f;
    ki = 0.0f;
    kd = 0.002f;

    integrale = 0;
    erreurPrecedente = 0;
    dernierPID = 0;

    positionLigne = 0;
    correction = 0;

    positionFiltree = 0;
    deriveeFiltree = 0;
    premiereLecture = true;

    ligneDetectee = false;
}

void SuiviLigne::initialiser(Calibration* capteur) {
    calibration = capteur;
}

void SuiviLigne::commencer() {
    integrale = 0;
    erreurPrecedente = 0;
    dernierPID = millis();

    positionLigne = 0;
    correction = 0;

    positionFiltree = 0;
    deriveeFiltree = 0;
    premiereLecture = true;

    ligneDetectee = false;
}

void SuiviLigne::actualiser() {
    if (calibration == nullptr || !calibration->estCalibre()) {
        correction = 0;
        ligneDetectee = false;
        return;
    }

    // Lire les cinq capteurs
    calibration->lireCapteurs();

    float numerateur = 0;
    float denominateur = 0;

    float intensiteMax = 0;

    for (int i = 0; i < NB_IR; i++) {
        int minimum = calibration->getMin(i);
        int maximum = calibration->getMax(i);

        // Ignorer une calibration insuffisante
        if (maximum - minimum < 50) {
            continue;
        }

        float valeur = calibration->getValeur(i);

        // Normalisation entre 0 et 1000
        float normalisee =
            (valeur - minimum) * 1000.0f /
            (maximum - minimum);

        normalisee = constrain(
            normalisee, 0.0f, 1000.0f
        );

        // Noir = 1000, blanc = 0
        float noir = 1000.0f - normalisee;

        // Supprimer les faibles détections
        if (noir < 150.0f) {
            noir = 0;
        }

        if (noir > intensiteMax) {
            intensiteMax = noir;
        }

        // Position des capteurs :
        // IR0 = -2000
        // IR1 = -1000
        // IR2 =     0
        // IR3 = +1000
        // IR4 = +2000
        numerateur += noir * (i - 2) * 1000.0f;
        denominateur += noir;
    }

    // Vérifier si la ligne est détectée
    if (intensiteMax < 250.0f ||
        denominateur < 250.0f) {

        ligneDetectee = false;
        correction = 0;

        integrale = 0;
        deriveeFiltree = 0;
        premiereLecture = true;
        dernierPID = millis();

        return;
    }

    ligneDetectee = true;

    // Calcul de la position
    float positionBrute =
        numerateur / denominateur;

    // Filtrage de la position
    const float alpha = 0.65f;

    if (premiereLecture) {
        positionFiltree = positionBrute;
    } else {
        positionFiltree =
            alpha * positionBrute +
            (1.0f - alpha) * positionFiltree;
    }

    positionLigne = positionFiltree;

    // Calcul du temps écoulé
    unsigned long maintenant = millis();

    float dt =
        (maintenant - dernierPID) / 1000.0f;

    if (dt <= 0.0f) {
        return;
    }

    // Erreur par rapport au centre
    float erreur = positionLigne;

    // Terme intégral avec anti-windup
    integrale += erreur * dt;

    integrale = constrain(
        integrale, -1000.0f, 1000.0f
    );

    // Terme dérivé
    float derivee = 0;

    if (!premiereLecture) {
        derivee =
            (erreur - erreurPrecedente) / dt;
    }

    // Filtrage de la dérivée
    const float alphaD = 0.3f;

    deriveeFiltree =
        alphaD * derivee +
        (1.0f - alphaD) * deriveeFiltree;

    // Calcul PID
    correction =
        kp * erreur +
        ki * integrale +
        kd * deriveeFiltree;

    // Limiter la correction
    correction = constrain(
        correction, -50.0f, 50.0f
    );

    erreurPrecedente = erreur;
    dernierPID = maintenant;
    premiereLecture = false;
}

float SuiviLigne::getPosition() {
    return positionLigne;
}

float SuiviLigne::getCorrection() {
    return correction;
}

bool SuiviLigne::estLigneDetectee() {
    return ligneDetectee;
}
