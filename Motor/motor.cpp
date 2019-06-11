
#include "motor.h"
#include <Arduino.h>

void motorInit()
{
	pinMode(MOTOR_1_PIN, OUTPUT);
    pinMode(MOTOR_2_PIN, OUTPUT);
    pinMode(MOTOR_3_PIN, OUTPUT);
    pinMode(MOTOR_4_PIN, OUTPUT);

    ledcSetup(0, 6000, 10);
    ledcAttachPin(MOTOR_1_PIN, 0);

    ledcSetup(1, 6000, 10);
    ledcAttachPin(MOTOR_2_PIN, 1);

    ledcSetup(2, 6000, 10);
    ledcAttachPin(MOTOR_3_PIN, 2);

    ledcSetup(3, 6000, 10);
    ledcAttachPin(MOTOR_4_PIN, 3);
    motorWriteAll(0, 0 ,0 , 0);
}

void motorWriteAll(uint16_t pwm1, uint16_t pwm2, uint16_t pwm3, uint16_t pwm4)
{
    ledcWrite(MOTOR_4, pwm1);
    ledcWrite(MOTOR_3, pwm2);
    ledcWrite(MOTOR_2, pwm3);
    ledcWrite(MOTOR_1, pwm4);
}

// void motorWrite(uint16_t numMotor, uint16_t pwm)
// {
//     ledcWrite(numMotor, pwm);
// }

