#include <Arduino.h>
#include <Encoder.h>
#include "board.h"
#include "motor.h"
#include "PIDv2.h"

Motor motor(IN1_1, IN2_1);
Encoder encoder(A_1, B_1);

const uint32_t sample_time = 10;
PID pid(2, 0.2, 5, sample_time);

uint32_t time;
const uint32_t display_time = 30;

void setup() {
	Serial.begin(9600);
	delay(3000);

	pid.setOutputLimits(-255, 255);

	pid.setInput(encoder.read());
	pid.setReference(1200.0);

	pid.setMode(AUTOMATIC);

	time = millis();
	while(!Serial.available());
}

void loop() {
	pid.setInput(encoder.read());
	pid.compute();
	motor.set_pwm(int16_t(pid.getOutput()));

	if (millis() - time > display_time) {
		time += display_time;
		Serial.print(encoder.read());
		Serial.print(" ");
		Serial.println(pid.getOutput());
  }
}
