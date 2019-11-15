#include <Arduino.h>
#include <Encoder.h> // Inclue la librairie encodeur
#include "board.h" // Contient les noms de pins de la Teensy

Encoder enc(A1, B1); //Déclaration des pins de l'encodeur
int32_t pos = 0; // variable qui prendra la valeur de la position du moteur

void setup() {
	Serial.begin(9600);
	Serial.println("Starting Test");
	delay(3000);
}

void loop() {
	pos = enc.read();
	Serial.println(pos);
	delay(100);
}
