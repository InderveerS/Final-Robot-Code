#pragma once
#include <Arduino.h>
#include "driver/adc.h"

#define FAR_LEFT_CH ADC1_CHANNEL_0 
#define LEFT_CH ADC1_CHANNEL_1
#define CENTER_CH ADC1_CHANNEL_2
#define RIGHT_CH ADC1_CHANNEL_3
#define FAR_RIGHT_CH ADC1_CHANNEL_4

#define LINE_PRESENT_THRESHOLD 1000 // TODO: tune this for line detection
#define MAX_ERROR 100 // TODO: tune this for line detection

class IRArray {
    public: 
        IRArray();

        float readLine();
        uint16_t readFarLeft();
        uint16_t readFarRight();

        void calibrateMiddle();
        void calibrateFarLeft();
        void calibrateFarRight();
    private:
        const float width = 5.0f; // width between sensors in mm
        uint16_t normalize(uint16_t value, uint16_t min, uint16_t max);

        uint16_t minL = 0, maxL = 4095;
        uint16_t minC = 0, maxC = 4095;
        uint16_t minR = 0, maxR = 4095;
        
        uint16_t minFL = 0, maxFL = 4095;
        uint16_t minFR = 0, maxFR = 4095;

        int8_t lastDirection = 0; // -1 for left, 1 for right, 0 for center
};    