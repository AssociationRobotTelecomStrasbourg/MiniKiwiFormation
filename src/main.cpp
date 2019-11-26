#include <Arduino.h>
#include <Encoder.h>
#include "board.h"
#include "motor.h"
#include "pid.h"
#include "binserial.h"

float last_position;
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

settings_t settings = {1, 0, 0, 0, 0, false, true}; // Réglages du PID
size_t settings_size = 22;

variables_t position_variables = {0, 0, 0, 0}, speed_variables = {0, 0, 0, 0};
size_t variables_size = 16;

uint32_t time; // Temps de la dernière période d'échantillonnage

Motor motor(IN1_1, IN2_1); // Initialise motor
Encoder encoder(A_1, B_1); // Initialise encoder

PID speed_pid(50, 0.5, 0); // Initialise pid
PID position_pid(1, 0, 12); // Initialise pid

void setup() {
    Serial.begin(9600); // Initialise Serial communication
    while (!Serial); // Attend que la liaison soit établie

    // Initialise LED de debug
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);

    // while(!Serial.available()); // Attend une consigne de pid_interface.py

    time = micros()/1000. - settings.sample_time; // Initialise le temps
}

void loop() {
    // Éxécute les instruction toutes les périodes d'échantillonnages
    if (micros()/1000. - time > settings.sample_time) {
    time = micros()/1000;
    last_position = position_variables.input;
    position_variables.input = encoder.read();
        // Calcul et applique le PID
    speed_variables.input = (position_variables.input-last_position)/1200.*1000./settings.sample_time;

      // Lit l'entrée
    position_pid.setInput(position_variables.input);
    position_pid.compute(); // Calcul le PID
    speed_variables.setpoint = position_variables.output = position_pid.getOutput();
    position_variables.integral = position_pid.getIntegral();

    speed_pid.setSetpoint(speed_variables.setpoint);
        speed_pid.setInput(speed_variables.input); // Met à jour l'entrée du PID
        speed_pid.compute(); // Calcul le PID

        speed_variables.integral = speed_pid.getIntegral(); // Lit le terme intégral
        speed_variables.output = speed_pid.getOutput(); // Lit la sortie

        motor.setPwm(speed_variables.output); // Applique la sortie

        // Envoie les variables du PID à pid_interface.py
        writeData(&position_variables, variables_size);

        // Met à jour les réglages du PID si reData(&variables, variables_size);éception de nouveaux réglages
        if (Serial.available()) {
            digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); // Change l'état de la led
            readData(&settings, settings_size); // Reçoit les réglages

            // Applique les réglages
            speed_pid.setMode(settings.mode);
            position_pid.setMode(settings.mode);
            position_pid.setAntiWindup(settings.anti_windup);
            position_pid.setTunings(settings.kp, settings.ki, settings.kd);
            position_pid.setSetpoint(settings.setpoint);
            position_variables.setpoint = settings.setpoint;
        }
    }
}
