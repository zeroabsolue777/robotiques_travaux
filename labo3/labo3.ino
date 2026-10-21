#include <MeAuriga.h>

// 1. CONSTANTES ET CONFIGURATION

#define LEDNUM 12

// 2. OBJETS DES CAPTEURS ET DES ACTIONNEURS

MeUltrasonicSensor ultraSensor(PORT_10);
MeRGBLed led(0, LEDNUM);
MeGyro gyro(0, 0x69);
MeEncoderOnBoard encoderRight(SLOT1);
MeEncoderOnBoard encoderLeft(SLOT2);

// 3. VARIABLES GLOBALES


// 4. ÉTATS DU ROBOT

enum State {
    SEGMENT1,
    PIVOT1,
    SEGMENT2,
    PIVOT2,
    SEGMENT3,
    PIVOT3,
    FIN,
    RETOUR
};

State state = SEGMENT1;
State precedentState = SEGMENT1;

// 5. FONCTIONS DU GYROSCOPE ET DES ENCODEURS

void gyroTask(unsigned long ct){};
void encodersTask(unsigned long ct){};

// 6. FONCTIONS DE CALCUL DES DISTANCES

float retournerDistance(unsigned long ct);
float calculDistancePar(unsigned long ct){};

// 7. FONCTIONS DE DÉPLACEMENT

void goStraight(){};
void spin(){};

// 8. FONCTIONS DE CONTRÔLE DES DEL

// 9. FONCTIONS DE CHAQUE ÉTAPE DU LABORATOIRE

void gererSegment1(unsigned long ct){};
void gererPivot1(unsigned long ct){};
void gererSegment2(unsigned long ct){};
void gererPivot2(unsigned long ct){};
void gererSegment3(unsigned long ct){};
void gererPivot3(unsigned long ct){};
void gererFin(unsigned long ct){};

// Bonus 
void gererRetour(unsigned long ct){};

// 10. MACHINE À ÉTATS

void manageState(unsigned long ct){};


void setup() {
    Serial.begin(115200);

    // Conserver ici l'initialisation originale :
    // configuration des DEL, encodeurs et gyroscope.

    // Attendre 3 secondes au démarrage.
    delay(3000);
}

// 12. BOUCLE PRINCIPALE

void loop() {
    unsigned long currentTime = millis();

    // Calculer la distance parcourue.
    // Mettre à jour le gyroscope et les encodeurs.
    // Gérer l'état actuel du robot.

    double distanceParcourue = calculDistancePar(currentTime);

    manageState(currentTime);

    gyroTask(currentTime);
    encodersTask(currentTime);
}