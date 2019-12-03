#include <Arduino.h>
#include <Encoder.h>
#include <pid.h>
#include "board.h"
#include "motor.h"
#include "binserial.h"

typedef struct {
    float input;
    float setpoint;
} speed_t;

typedef struct {
    float position;
    float last_positon;
} position_t;

const uint32_t sample_time = 10;
const float step_per_turn = 1200;

speed_t speed1 = {0., 0.}, speed2 = {0., 0.};
position_t p1 = {0., 0.}, p2 = {0., 0.};

uint32_t time; // Temps de la dernière période d'échantillonnage

Motor motor1(IN1_1, IN2_1); // Initialise motor
Motor motor2(IN1_2, IN2_2); // Initialise motor

Encoder encoder1(A_1, B_1); // Initialise encoder
Encoder encoder2(A_2, B_2); // Initialise encoder

PID speed_pid1(50, 0, 0); // Initialise pid
PID speed_pid2(50, 0, 0); // Initialise pid

void setup() {
    Serial.begin(9600); // Initialise Serial communication
    while (!Serial); // Attend que la liaison soit établie

    // Initialise LED de debug
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);

    // speed_pid1.setMode(true);
    // speed_pid2.setMode(true);

    time = millis() - sample_time; // Initialise le temps
}

void loop() {
    // Éxécute les instruction toutes les périodes d'échantillonnages
    if (millis() - time >= sample_time) {
        time = millis();

        p1.last_positon = p1.position;
        p2.last_positon = p2.position;
        p1.position = encoder1.read();
        p2.position = encoder2.read();

        speed1.input = (p1.position-p1.last_positon)*1000/step_per_turn/sample_time;
        speed2.input = (p2.position-p2.last_positon)*1000/step_per_turn/sample_time;

        speed_pid1.setInput(speed1.input);
        speed_pid2.setInput(speed2.input);

        speed_pid1.compute();
        speed_pid2.compute();

        motor1.setPwm(speed_pid1.getOutput());
        motor2.setPwm(speed_pid2.getOutput());

        // Envoie les variables du PID à pid_interface.py
        writeData(&speed1, sizeof(speed_t));
        writeData(&speed2, sizeof(speed_t));

        // Met à jour les réglages du PID si réception de nouveaux réglages
        if (Serial.available()) {
            digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
            readData(&speed1.setpoint, sizeof(speed1.setpoint));
            speed2.setpoint = speed1.setpoint;

            // Applique les réglages
            speed_pid1.setSetpoint(speed1.setpoint);
            speed_pid2.setSetpoint(speed2.setpoint);
        }
    }
}
