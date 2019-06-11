#ifndef Communication_H
#define Communication_H

#include <Arduino.h>
#include <WiFi.h>


class Calling
{
    public:
        const char* ssid     = "Drone S.A";
        const char* password = "86946885";

        int value = 0;
        double KP = 0.0;
        double KI = 0.0;
        double KD = 0.0;

        void Looping();
        void Setting();
        void tcp();
        void ReadWifi(uint32_t *going, double *Kp, double *Ki, double *Kd);
        // void WriteWifi( float angX_, float angY_,int16_t pitch, int16_t roll);//,float angZ_ );    
        void WriteWifi(float angX_, float angY_,float angZ_,double _kp, double _ki,double _kd,int16_t pwm1, int16_t pwm2,int16_t pwm3, int16_t pwm4);       

        u_long time = 0;

        String aux = "";
};

#endif
