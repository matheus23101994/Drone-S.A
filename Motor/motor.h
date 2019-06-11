#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

#define MOTOR_1_PIN 32  //32
#define MOTOR_2_PIN 25  //25
#define MOTOR_3_PIN 26  //26
#define MOTOR_4_PIN 33  //33

enum {MOTOR_1 = 0, MOTOR_2, MOTOR_3, MOTOR_4};

void motorInit();
void motorWriteAll(uint16_t pwm1, uint16_t pwm2, uint16_t pwm3, uint16_t pwm4);
// void motorWrite(uint16_t numMotor, uint16_t pwm);

#endif