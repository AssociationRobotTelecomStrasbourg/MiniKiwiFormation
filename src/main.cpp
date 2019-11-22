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
	float setpoint;
	bool mode;
	bool anti_windup;
} settings_t;

// Variables du PID
typedef struct {
	float input;
	float setpoint;
	float output;
	float integral;
} variables_t;

settings_t settings = {10, 0, 0, 0, 0, false, true}; // Réglages du PID
size_t settings_size = 22;

variables_t variables = {0, 0, 0, 0};
size_t variables_size = 16;

uint32_t time; // Temps de la dernière période d'échantillonnage

Motor motor(IN1_1, IN2_1); // Initialise motor
Encoder encoder(A_1, B_1); // Initialise encoder

PID pid(2, 0.3, 5); // Initialise pid

void setup() {
	Serial.begin(9600); // Initialise Serial communication
	while (!Serial); // Attend que la liaison soit établie

	// Initialise LED de debug
	pinMode(LED_BUILTIN, OUTPUT);
	digitalWrite(LED_BUILTIN, HIGH);

	// while(!Serial.available()); // Attend une consigne de pid_interface.py

	time = millis() - settings.sample_time; // Initialise le temps
}

void loop() {
	// Éxécute les instruction toutes les périodes d'échantillonnages
	if (millis() - time > settings.sample_time) {
		time = millis();

		// Calcul et applique le PID
		variables.input = encoder.read(); // Lit l'entrée

		pid.setInput(variables.input); // Met à jour l'entrée du PID
		pid.compute(); // Calcul le PID

		variables.integral = pid.getIntegral(); // Lit le terme intégral
		variables.output = pid.getOutput(); // Lit la sortie
		motor.setPwm(variables.output); // Applique la sortie

		// Envoie les variables du PID à pid_interface.py
		writeData(&variables, variables_size);

		// Met à jour les réglages du PID si réception de nouveaux réglages
		if (Serial.available()) {
			digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); // Change l'état de la led
			readData(&settings, settings_size); // Reçoit les réglages

			// Applique les réglages
			pid.setMode(settings.mode);
			pid.setAntiWindup(settings.anti_windup);
			pid.setTunings(settings.kp, settings.ki, settings.kd);
			pid.setSetpoint(settings.setpoint);
			variables.setpoint = settings.setpoint;
		}
	}
}
