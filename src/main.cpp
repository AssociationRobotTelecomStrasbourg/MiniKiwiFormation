#include <Arduino.h>
#include <Encoder.h>
#include "board.h"
#include "motor.h"
#include "pid.h"
#include "binserial.h"

typedef struct {
	float kp;
	float ki;
	float kd;
	float reference;
} parameters_t;

parameters_t parameters;
float input;
uint32_t time;

Motor motor(IN1_1, IN2_1);
Encoder encoder(A_1, B_1);

const uint32_t sample_time = 10;
PID pid(0, 0, 0, sample_time);

void setup() {
	// Initialise Serial communication
	Serial.begin(9600);
	while (!Serial);

	// Initialise LED de debug
	pinMode(LED_BUILTIN, OUTPUT);
	digitalWrite(LED_BUILTIN, HIGH);

	// Initialise le PID
	pid.setOutputLimits(-255, 255);
	pid.setInput(encoder.read());
	pid.setReference(0);

	// Attend de recevoir
	while(!Serial.available());

	// Active le pid
	pid.setMode(true);

	time = millis() - sample_time;
}

void loop() {
	if (millis() - time > sample_time) {
		time = millis();

		input = encoder.read();
		pid.setInput(input);
		pid.compute();
		motor.setPwm(pid.getOutput());

		writeData(&input, sizeof(input));
		if (Serial.available()) {
			digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
			readData(&parameters, sizeof(parameters));
			pid.setTunings(parameters.kp, parameters.ki, parameters.kd);
			pid.setReference(parameters.reference);
		}
	}
}
