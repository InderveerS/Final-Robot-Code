#pragma once
#include <Arduino.h>

class PID {
    public:
        PID(float kp, float ki, float kd, float dt, float outMin, float outMax, float alpha, float integralActivationThreshold);
        float update(float error);

        void reset() {
            integral = 0.0f;
            lastError = 0.0f;
            lastFilteredD = 0.0f;
        }
        
    private:
        float kp;
        float ki;
        float kd;

        float dt;

        float outMin;
        float outMax;

        float alpha;

        float integralActivationThreshold;

        float integral = 0.0f;
        float lastError = 0.0f;

        float lastFilteredD = 0.0f;

        static float clamp(float val, float min, float max) {
            if (val > max)
    	        return max;
            if (val < min)
    	        return min;
            return val;
        }
};    