#include "pid.h"
#include "motor.h"
#include "MPU6050.h"
#include "Communication.h"
#include <Arduino.h>

double kp;
double ki;
double kd;

double _kp;
double _ki;
double _kd;

float angX1,angY1,angZ1; 
int16_t pwmMotor1, pwmMotor2, pwmMotor3, pwmMotor4;
int16_t pidRoll, pidPitch, pidYaw;
uint32_t now;

MPU Mpu;

Calling Wifi;

Pid pidX, pidY, pidZ; // 1.4,0.001,0.001; // 1.3,0.04,18.0 // 0.0,0.0,0.0 // 1.0,0.0,0.0 // 2.3, 0.04, 10

void setup()
{
    pidX.setGains(0.0,0.0,0.0);
    pidY.setGains(0.0,0.0,0.0);
    pidZ.setGains(4.0,0.02,0.0);
    Wifi.Setting();
    motorInit();
    Serial.begin(115200);
    pidX.setOutputLimits(-500.0,500.0);
    pidY.setOutputLimits(-500.0,500.0);
    pidZ.setOutputLimits(-500.0,500.0);
    pidX.setSetPoint(0.0);
    pidY.setSetPoint(0.0);
    pidZ.setSetPoint(0.0);
    Mpu.settingMPU();
} 
void loop()
{
    Wifi.Looping();
    Wifi.ReadWifi(&now,&kp,&ki,&kd);
    pidX.setGains(kp,ki,kd);
    pidY.setGains(kp,ki,kd);
    //pidZ.setGains(_kp,_ki,_kd);
    Mpu.looppingMPU();
    // Lê os ângulos
    Mpu.readAng(&angX1, &angY1, &angZ1);
    
    //Calcula PID
    pidPitch  = (int16_t)pidY.process(angY1);
    pidRoll   = (int16_t)pidX.process(angX1);
    pidYaw   = (int16_t)pidZ.process(angZ1);
    

    pwmMotor1 = now + pidRoll - pidPitch;// - pidYaw;
    pwmMotor2 = now + pidRoll + pidPitch;// + pidYaw;
    pwmMotor3 = now - pidRoll + pidPitch;// - pidYaw;
    pwmMotor4 = now - pidRoll - pidPitch;// + pidYaw;

    // Wifi.WriteWifi(angX1,angY1,angZ1,kp,ki,kd,pwmMotor1,pwmMotor2,pwmMotor3,pwmMotor4);

    if(pwmMotor1 < 0 || now == 0)pwmMotor1 = 0;
    if(pwmMotor2 < 0 || now == 0)pwmMotor2 = 0;
    if(pwmMotor3 < 0 || now == 0)pwmMotor3 = 0;
    if(pwmMotor4 < 0 || now == 0)pwmMotor4 = 0;
    
    // // Aplica PID aos motores
    motorWriteAll(pwmMotor1, pwmMotor2, pwmMotor3, pwmMotor4);

   
    
}

