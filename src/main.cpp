#include <Arduino.h>
#include <Encoder.h>
#include "board.h"
#include "motor.h"
#include "pid.h"
#include "binserial.h"

// Réglages du PID
typedef struct {
	uint32_t sample_time;
	float kp;
	float ki;
	float kd;
	float reference;
	bool mode;
	bool anti_windup;
} settings_t;

settings_t settings = {10, 0, 0, 0, 0, false, true}; // Réglages du PID
size_t settings_size = 22;

float input; // Position du codeur
uint32_t time; // Temps de la dernière période d'échantillonnage

Motor motor(IN1_1, IN2_1); // Initialise motor
Encoder encoder(A_1, B_1); // Initialise encoder

PID pid(0, 0, 0); // Initialise pid

void setup() {
	Serial.begin(9600); // Initialise Serial communication
	while (!Serial); // Attend que la liaison soit établie

	// Initialise LED de debug
	pinMode(LED_BUILTIN, OUTPUT);
	digitalWrite(LED_BUILTIN, HIGH);

	while(!Serial.available()); // Attend une consigne de pid_interface.py

	time = millis() - settings.sample_time; // Initialise le temps
}

void loop() {
	// Éxécute les instruction toutes les périodes d'échantillonnages
	if (millis() - time > settings.sample_time) {
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
			readData(&settings, settings_size); // Reçoit les réglages
			// Applique les réglages
			pid.setMode(settings.mode);
			pid.setAntiWindup(settings.anti_windup);
			pid.setTunings(settings.kp, settings.ki, settings.kd);
			pid.setSetpoint(settings.reference);
		}
	}
}
