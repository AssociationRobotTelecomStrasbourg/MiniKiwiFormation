#include <Arduino.h>
#include "binserial.h"

typedef struct {
	uint32_t sample_time;
	float kp;
	float ki;
	float kd;
	float reference;
} parameters_t;

parameters_t parameters;
float time;

void setup() {
	Serial.begin(9600);
	while (!Serial);

	pinMode(LED_BUILTIN, OUTPUT);
	digitalWrite(LED_BUILTIN, HIGH);
	readData(&parameters, sizeof(parameters));
}

void loop() {
	time = millis();
	writeData(&time, sizeof(time));
	if (Serial.available()) {
		digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
		readData(&parameters, sizeof(parameters));
	}
	delay(parameters.sample_time);
}
