#include <Encoder.h>
#include "board.h"
#include "motor.h"
#include "PIDv2.h"

Encoder encoder(A_1, B_1); //Déclaration des pins de l'encodeur
int32_t position = 0;

Motor motor(IN1_1, IN2_1);

void setup() {
	Serial.begin(9600);
	delay(3000);
	motor.set_pwm(28);
}

void loop() {
	position = encoder.read();
	Serial.println(position);
	delay(100);
}
