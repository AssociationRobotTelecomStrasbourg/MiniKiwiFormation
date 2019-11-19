#include <Arduino.h>
#include <Encoder.h>
#include "board.h"
#include "motor.h"
#include "pid.h"
#include "binserial.h"

// Réglages du PID
typedef struct {
	float kp;
	float ki;
	float kd;
	float reference;
} settings_t;

settings_t settings; // Réglages du PID
float input; // Position du codeur
uint32_t time; // Temps de la dernière période d'échantillonnage

Motor motor(IN1_1, IN2_1); // Initialise motor
Encoder encoder(A_1, B_1); // Initialise encoder

const uint32_t sample_time = 10; // Période d'échantillonnage
PID pid(0, 0, 0, sample_time); // Initialise pid

void setup() {
	Serial.begin(9600); // Initialise Serial communication
	while (!Serial); // Attend que la liaison soit établie

	// Initialise LED de debug
	pinMode(LED_BUILTIN, OUTPUT);
	digitalWrite(LED_BUILTIN, HIGH);

	// Règle le PID
	pid.setOutputLimits(-255, 255); // Sature la sortie entre [-255, 255]
	pid.setInput(encoder.read()); // Règle l'entrée par la position du codeur
	pid.setReference(0); // Règle la référence sur 0

	while(!Serial.available()); // Attend une consigne de pid_interface.py

	// Active le pid
	pid.setMode(true);

	time = millis() - sample_time; // Initialise le temps
}

void loop() {
	// Éxécute les instruction toutes les périodes d'échantillonnages
	if (millis() - time > sample_time) {
		time = millis();

		// Calcul et applique le PID
		input = encoder.read();
		pid.setInput(input);
		pid.compute();
		motor.setPwm(pid.getOutput());

		// Envoie la positon du codeur à pid_interface.py
		writeData(&input, sizeof(input));

		// Met à jour les réglages du PID si réception de nouveaux réglages
		if (Serial.available()) {
			digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); // Change l'état de la led
			readData(&settings, sizeof(settings)); // Reçoit les réglages
			// Applique les réglages
			pid.setTunings(settings.kp, settings.ki, settings.kd);
			pid.setReference(settings.reference);
		}
	}
}
