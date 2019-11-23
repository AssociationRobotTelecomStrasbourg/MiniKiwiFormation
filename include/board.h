#ifndef BOARD_H
#define BOARD_H

// Hardware Serial port 1, connected to ESP01
#define RX1 0
#define TX1 1

// Red debug led
#define LED_DEBUG 2

// Buzzer and pump control outputs (Pump mosfet can be used to drive up to 1A of load at 12V)
#define BZR 3
#define PMP 4

// Motor drivers inputs
// If 12V drive is used on 6V motors, limit PWM value to 128-140, current is internally limited for protection in case of mistake.
	// IN1  IN2  OUT1  OUT2  FUNC
	//  0    1    L     H     Reverse
	//  1    0    H     L     Forward
	//  0    1    H/L   L     Chop reverse, mixed decay
	//  1    0    L     H/L   Chop forward, mixed decay
	//  1    1    L     L     Brake, slow decay
	//  0    0    Z     Z     Coast, enters standby mode after 1ms

#define IN1_2 5
#define IN2_2 6

#define IN1_1 9
#define IN2_1 10

// Motor Encoder input, typical value is 100.37*12 = 1244.4 counts per turn
#define A_1 7
#define B_1 8
#define A_2 11
#define B_2 12

// Blinks the LED_BUILTIN
#define LED_TEENSY 13

// Inputs for pololu analog reflectance sensor, sensor input is connected to a voltage divider (ratio is 0.65)
// Other sensors needing only one pin and 5V power can be used on this input provided they can work with the integrated voltage divider
#define Q1 15
#define Q2 14

// Servo outputs
// Limit is 3 servos with a max combined current of 300mA max, servos should be used to drive small loads only.
#define SERVO_SIG1 23
#define SERVO_SIG2 22
#define SERVO_SIG3 21

// Servo Inputs
// Should be connected to the servomotor's potentiometer to get an accurate value of its position, should work with most 5V servos as their potentiometer value is around 2.3V
#define SERVO_FB1 20
#define SERVO_FB2 17
#define SERVO FB3 16

// I2C output
// 2 outputs are available on the board with 3.3V, Gnd and I2C.
#define SCL0 19
#define SDA0 18


#endif
