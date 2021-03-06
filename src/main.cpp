#include <Arduino.h>
#include "board.h"
#include "locomotion.h"
#include "binserial.h"

const uint32_t sample_time = 10;
uint32_t time; // Temps de la dernière période d'échantillonnage
float distance;

Locomotion locomotion(sample_time/1000.);

void setup() {
    Serial.begin(9600); // Initialise Serial communication
    while (!Serial); // Attend que la liaison soit établie

    // Initialise LED de debug
    pinMode(LED_BUILTIN, OUTPUT);

    time = millis() - sample_time; // Initialise le temps
}

void loop() {
    // Éxécute les instruction toutes les périodes d'échantillonnages
    if (millis() - time >= sample_time) {
        time = millis();

        // Run the locomotion
        if (STOP == locomotion.run())
            digitalWriteFast(LED_BUILTIN, HIGH);
        else
            digitalWriteFast(LED_BUILTIN, LOW);

        writeData(locomotion.getPosition(), sizeof(position_t));
        if (Serial.available()) {
            readData(&distance, sizeof(distance));
            locomotion.translateFrom(distance);
        }
    }
}
