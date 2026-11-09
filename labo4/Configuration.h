
#ifndef CONFIGURATION_H
#define CONFIGURATION_H


// 1. CONFIGURATION GENERALE

#define BAUD_RATE 115200

// 2. CONFIGURATION DU ROBOT

#define LEDNUM 12
#define LED_PIN 44

#define PULSE 9
#define RATIO 39.267
#define CIRC_WHEEL 202.6

// 3. VITESSES DU ROBOT

const int VITESSE = 100;
const int VITESSE_PIVOT = 50;

// 4. PARAMETRES DES MOUVEMENTS

const float TOLERANCE = 2.0;
const float ANGLE_CALIBRATION = 360.0;
const float ANGLE_DEMI_TOUR = 180.0;

// 5. TEMPS DES ETATS (millisecondes)

const unsigned long TEMPS_BOOT = 3000;
const unsigned long TEMPS_AVERTISSEMENT = 3000;

// 6. PARAMETRES DU PID

// Valeurs provisoires à ajuster pendant les tests
const float KP = 6.5;
const float KI = 0.0;
const float KD = 1.1;

// 7. COULEURS

#define COULEUR_ROUGE 1
#define COULEUR_JAUNE 2
#define COULEUR_VERT 3

#endif
