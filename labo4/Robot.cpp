
#include "Robot.h"

// CONSTRUCTEUR

Robot::Robot()
    : etatActuel(ATTENTE),
      etatPrecedent(ATTENTE),
      tempsDebut(0),
      calibrationEnCours(false),
      estCalibre(false),
      dernierAffichageLigne(0),
      phaseSuivi(AVANCER_LIGNE),
      debutPerteLigne(0),
      dernierClignotement(0),
      ledsAllumees(false) {
}

void Robot::initialiser() {
    Serial.begin(BAUD_RATE);

    initialiserLedsEtat();
    eteindreLedsEtat();

    calibration.initialiser();
    suiviLigne.initialiser(&calibration);

    dernierAffichageLigne = 0;

    changerEtat(BOOT);
}

void Robot::actualiser() {
    lireCommandes();
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

    if (etatActuel == nouvelEtat && nouvelEtat != BOOT) {
        return;
    }

    if (etatActuel == LAB3) {
        arreterLab3();
    }

    // Arrêter le suivi de ligne avant de le quitter.
    if (etatActuel == SUIVRE_LIGNE) {
        if (phaseSuivi == PIVOTER_180) {
            arreterPivot180();
        }

        arreterMoteursLigne();
    }

    if (etatActuel == CALIB) {
        arreterRotationCalibration();
        calibrationEnCours = false;
    }

    etatPrecedent = etatActuel;
    etatActuel = nouvelEtat;
    tempsDebut = millis();

    switch (etatActuel) {
        case BOOT:
            Serial.println("État : BOOT");
            eteindreLedsEtat();
            break;

        case ATTENTE:
            Serial.println("État : ATTENTE");
            arreterMoteursLigne();
            eteindreLedsEtat();
            break;

        case LAB3:
            Serial.println("État : LAB3");
            eteindreLedsEtat();
            initialiserLab3();
            break;

        case CALIB:
            Serial.println("État : CALIB");
            eteindreLedsEtat();

            calibrationEnCours = calibration.estConnecte();

            if (calibrationEnCours) {
                calibration.commencer();
                estCalibre = false;
                commencerRotationCalibration();
            } else {
                Serial.println(
                    "Calibration impossible : LyneTracker absent"
                );
            }
            break;

        case SUIVRE_LIGNE:
            Serial.println("État : SUIVRE_LIGNE");
            eteindreLedsEtat();

            suiviLigne.commencer();

            phaseSuivi = AVANCER_LIGNE;
            debutPerteLigne = 0;
            dernierAffichageLigne = 0;
            break;

        case CMD_INVALIDE:
            Serial.println("État : CMD_INVALIDE");

            arreterMoteursLigne();

            dernierClignotement = millis() - 250UL;
            ledsAllumees = false;
            eteindreLedsEtat();
            break;

        case NON_CALIB:
            Serial.println("État : NON_CALIB");

            arreterMoteursLigne();

            dernierClignotement = millis() - 250UL;
            ledsAllumees = false;
            eteindreLedsEtat();
            break;
    }
}

void Robot::executerBoot() {
    if (millis() - tempsDebut >= TEMPS_BOOT) {
        changerEtat(ATTENTE);
    }
}

void Robot::executerAttente() {
    // rester en ATTENTE.
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

    // Relever les MIN/MAX pendant la rotation.
    calibration.calibrer();

    // La fonction retourne true lorsque
    // la rotation est terminée ou interrompue.
    if (actualiserRotationCalibration()) {
        calibrationEnCours = false;

        if (rotationCalibrationReussie()) {
            calibration.calibrer();
            calibration.terminer();

            estCalibre = calibration.estCalibre();

            Serial.println("Calibration reussie !");
        } else {
            estCalibre = false;

            Serial.println("ERREUR : calibration echouee");
        }

        changerEtat(ATTENTE);
    }
}

void Robot::executerSuiviLigne() {
    static unsigned long dernierCalcul = 0;

    // Phase du pivot.
    if (phaseSuivi == PIVOTER_180) {
        if (actualiserPivot180()) {
            phaseSuivi = AVANCER_LIGNE;
            debutPerteLigne = 0;

            suiviLigne.commencer();

            Serial.println("Reprise du suivi de ligne");
        }

        return;
    }

    // Actualiser le PID toutes les 20 ms.
    if (millis() - dernierCalcul < 20UL) {
        return;
    }

    dernierCalcul = millis();

    suiviLigne.actualiser();

    // Détecter la fin de la ligne.
    if (!suiviLigne.estLigneDetectee()) {
        arreterMoteursLigne();

        if (debutPerteLigne == 0) {
            debutPerteLigne = millis();
        }

        // Absence de ligne pendant 300 ms.
        if (millis() - debutPerteLigne >= 300UL) {
            Serial.println("Fin de ligne detectee");

            phaseSuivi = PIVOTER_180;
            commencerPivot180();
        }

        return;
    }

    debutPerteLigne = 0;

    // Vitesse de base du suivi.
    const int vitesseBase = 55;

    float correction = suiviLigne.getCorrection();

    int vitesseGauche = constrain(
        (int)(vitesseBase + correction),
        -100,
        100
    );

    int vitesseDroite = constrain(
        (int)(vitesseBase - correction),
        -100,
        100
    );

    commanderMoteursLigne(
        vitesseGauche,
        vitesseDroite
    );

    // Affichage périodique.
    if (millis() - dernierAffichageLigne >= 200UL) {
        dernierAffichageLigne = millis();

        Serial.print("Position : ");
        Serial.print(suiviLigne.getPosition());

        Serial.print(" | Correction : ");
        Serial.println(correction);
    }
}

void Robot::executerCmdInvalide() {
    actualiserClignotement(255, 0, 0);

    if (millis() - tempsDebut >= TEMPS_AVERTISSEMENT) {
        eteindreLedsEtat();
        changerEtat(ATTENTE);
    }
}

void Robot::executerNonCalib() {
    actualiserClignotement(255, 255, 0);

    if (millis() - tempsDebut >= TEMPS_AVERTISSEMENT) {
        eteindreLedsEtat();
        changerEtat(ATTENTE);
    }
}

void Robot::actualiserClignotement(
    int rouge,
    int vert,
    int bleu
) {
    if (millis() - dernierClignotement >= 250UL) {
        dernierClignotement = millis();
        ledsAllumees = !ledsAllumees;

        if (ledsAllumees) {
            afficherCouleurEtat(rouge, vert, bleu);
        } else {
            eteindreLedsEtat();
        }
    }
}


void Robot::lireCommandes() {
    while (Serial.available() > 0) {
        char caractere = Serial.read();

        if (caractere == '\n' || caractere == '\r') {
            if (commandeRecue.length() > 0) {
                String commande = commandeRecue;
                commandeRecue = "";

                traiterCommande(commande);
            }
        }
        else if (commandeRecue.length() < 32) {
            commandeRecue += caractere;
        }
        else {
            // Message trop long : réinitialiser.
            commandeRecue = "";
        }
    }
}

void Robot::traiterCommande(String commande) {
    commande.trim();
    commande.toLowerCase();

    if (commande.length() == 0) {
        return;
    }

    Serial.print("cmd reçu : ");
    Serial.println(commande);

    if (commande == "stop") {
        changerEtat(ATTENTE);
    }
    else if (commande == "lab3") {
        changerEtat(LAB3);
    }
    else if (commande == "cal") {
        changerEtat(CALIB);
    }
    else if (commande == "ligne") {
        if (estCalibre) {
            changerEtat(SUIVRE_LIGNE);
        } else {
            changerEtat(NON_CALIB);
        }
    }
    else {
        Serial.println("cmd INCONNUE");
        changerEtat(CMD_INVALIDE);
    }
}
