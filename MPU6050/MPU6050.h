#ifndef MPU6050_H
#define MPU6050_H

#include <Arduino.h>
#include <Wire.h>
#include <Kalman.h>

#define G_GAIN 0.00875
#define AA 0.98

class MPU
{
    private:
        // const int mpu=0x68;  //pino aberto 0X68 , pino ligado em 3,3V 0x69
        // int16_t ax, ay, az;
        // int16_t gx, gy, gz, gyro_x_cal, gyro_y_cal, gyro_z_cal;
        // float acelx, acely, acelz, rate_gyr_x, rate_gyr_y, rate_gyr_z, gyroXangle, gyroYangle, gyroZangle;
        // float AccXangle, AccYangle, AccZangle;
        // float const_calib = 16071.82;
        // float const_gravid = 9.81;
        // unsigned long pT;
        // unsigned long RunTime = 0;
        // int temperature;
        // int cycle;
        // float CFangleX, CFangleY, CFangleZ;

        /*------------Declarando variáveis------------*/
        uint8_t i2c_data[14];
        double accX, accY, accZ, gyroX, gyroY, gyroZ;
        double KalAngleX, KalAngleY, KalAngleZ, variacaoZ;
        double gyroXangle, gyroYangle, gyroZangle;
        double tempoAnterior;
        bool Status_LED = false;

        u_long lasTime = 0;
        /*-----------------Instâncias-----------------*/
        Kalman KalmanX, KalmanY, KalmanZ;
        /*-----------------Definições-----------------*/
        const uint8_t IMUAddress = 0x68; // AD0 is logic low on the PCB
        const uint16_t I2C_TIMEOUT = 1000; // Used to check for errors in I2C communication
        /*--------------------------------------------*/

    public:
        uint8_t i2c_Write(uint8_t registerAddress, uint8_t *data, uint8_t length, bool sendStop);
        uint8_t i2c_Write(uint8_t registerAddress, uint8_t data, bool _sendStop);
        uint8_t i2c_Read(uint8_t registerAddress, uint8_t *data, uint8_t nbytes);
        void readAng(float *angX, float *angY, float *angZ);
        void configMPU();
        void initKalman();
        void lerAcelGyro();
        void settingMPU();
        void looppingMPU();






        // void initialize();
        // void Atualiza();
        // void calibration();
        // void update();
        // float readTemp();
};

#endif