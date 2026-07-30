#pragma once
#include <Arduino.h>
#include "driver/adc.h"
#include "config.hpp"

class IRArray {
    public: 
        IRArray();

        float readLine();
        uint16_t readFarLeft();
        uint16_t readFarRight();

        uint16_t getMinL() const { return minL; }
        uint16_t getMaxL() const { return maxL; }
        uint16_t getMinC() const { return minC; }
        uint16_t getMaxC() const { return maxC; }  
        uint16_t getMinR() const { return minR; }
        uint16_t getMaxR() const { return maxR; }
        uint16_t getMinFL() const { return minFL; }
        uint16_t getMaxFL() const { return maxFL; }
        uint16_t getMinFR() const { return minFR; }
        uint16_t getMaxFR() const { return maxFR; }

        // Reads the three middle sensors and hands back their NORMALIZED
        // (0-1000) values; returns their sum, i.e. exactly what getTotal()
        // reports and what LINE_PRESENT_THRESHOLD gates on. Use this instead of
        // getTotal() when you also want the individual sensors - it is the same
        // three ADC conversions either way.
        uint16_t readMiddle(uint16_t& left, uint16_t& center, uint16_t& right);

        uint16_t getTotal();

        void calibrateMiddle();
        void calibrateFarLeft();
        void calibrateFarRight();
    private:
        const float width = cfg::IR_SENSOR_WIDTH_MM;
        uint16_t normalize(uint16_t value, uint16_t min, uint16_t max);

        // MinL: 179, MaxL: 3475, MinC: 181, MaxC: 3771, MinR: 204, MaxR: 3983
        // MinL: 157, MaxL: 3475, MinC: 165, MaxC: 3771, MinR: 179, MaxR: 3983
        // MinL: 153, MaxL: 3475, MinC: 157, MaxC: 3771, MinR: 159, MaxR: 3983
        // MinL: 163, MaxL: 1193, MinC: 160, MaxC: 1881, MinR: 165, MaxR: 1514
        // MinL: 185, MaxL: 3209, MinC: 158, MaxC: 1979, MinR: 191, MaxR: 3037
        // MinL: 200, MaxL: 2855, MinC: 191, MaxC: 2923, MinR: 203, MaxR: 2839
        // MinL: 217, MaxL: 3391, MinC: 201, MaxC: 2724, MinR: 201, MaxR: 3303
        // MinL: 216, MaxL: 3437, MinC: 183, MaxC: 2870, MinR: 201, MaxR: 3465
        uint16_t minL = 216, maxL = 3437;
        uint16_t minC = 183, maxC = 2870;
        uint16_t minR = 201, maxR = 3465;

        // MinFR: 1235, MaxFR: 3877, MinFL: 267, MaxFL: 3271
        // MinFR: 263, MaxFR: 3365, MinFL: 279, MaxFL: 3801
        // MinFR: 199, MaxFR: 2870, MinFL: 185, MaxFL: 2333
        uint16_t minFL = 185, maxFL = 2333;
        uint16_t minFR = 199, maxFR = 2870;

        int8_t lastDirection = 0; // -1 for left, 1 for right, 0 for center
};    