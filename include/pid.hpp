#pragma once
#include <Arduino.h>

class PID {
    public:
        PID(float kp, float ki, float kd, float dt, float outMin, float outMax, float alpha);

        float update(float error);
        
    private:
        float kp;
        float ki;
        float kd;

        float dt;

        float outMin;
        float outMax;

        float alpha;

        float integral = 0.0;
        float lastError = 0.0;

        static float clamp(float val, float min, float max) {
            if (val > max)
    	        return max;
            if (val < min)
    	        return min;
            return val;
        }
};    