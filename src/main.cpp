#include <Arduino.h>
#include "board.h"
#include "locomotion.h"
#include "binserial.h"

const uint32_t sample_time = 10;
uint32_t time; // Temps de la dernière période d'échantillonnage

Locomotion locomotion(sample_time);

void setup() {
    // Serial.begin(9600); // Initialise Serial communication
    // while (!Serial); // Attend que la liaison soit établie

    // Initialise LED de debug
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);

    locomotion.setSpeeds(-1, 1);

    time = millis() - sample_time; // Initialise le temps
}

void loop() {
    // Éxécute les instruction toutes les périodes d'échantillonnages
    if (millis() - time >= sample_time) {
        time = millis();
        locomotion.run();
    }
}
