#include "binserial.h"

void readData(void* data, size_t nb_bytes) {
	// size_t nb_bytes_read = 0;
	// char* buffer = (char*) data;
	// while (nb_bytes_read < nb_bytes) {
	// 	if (Serial.available()) {
	// 		buffer[nb_bytes_read] = Serial.read();
	// 		nb_bytes_read++;
	// 	}
	// }
	while (Serial.available() < nb_bytes);
	Serial.readBytes((char*) data, nb_bytes);
}

void writeData(void* data, size_t nb_bytes) {
	char* byteData = (char*) data;
	Serial.write(byteData, nb_bytes);
}
