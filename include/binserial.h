#ifndef BinSerial_H
#define BinSerial_H

#include <Arduino.h>

void readData(void* data, size_t nb_bytes);
void writeData(void* data, size_t nb_bytes);

#endif
