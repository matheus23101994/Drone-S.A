#ifndef Pid_H
#define Pid_H

class Pid
{
    private:
    
        float Kp, Ki, Kd;
        float Iterm = 0;
        float lastValue = 0;
        float outMin, outMax;
        float setPoint = 0;

    public:

        // Pid(float _Kp, float _Ki, float _Kd);
        void setGains(float _Kp, float _Ki, float _Kd);
        void setSetPoint(float newSetPoint);
        void setOutputLimits(float min, float max);
        float process(float newValue);   
};

#endif