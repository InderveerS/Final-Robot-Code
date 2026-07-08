#pragma once
#include <stdint.h>
#include <Arduino.h>
#include "driver/mcpwm.h" 

class Motor {
    public:
        Motor(uint8_t mPin1, uint8_t mPin2, bool mInverted, mcpwm_unit_t mcpwmUnit, mcpwm_timer_t mcpwmTimer);

        // sets duty cycle as positive or negative percentage of full speed (for direction)
        void setPWMPercent(int8_t percent);

        void stop();
        
    private:
        uint8_t mPin1;
        uint8_t mPin2;
        bool mInverted;
        mcpwm_unit_t mcpwmUnit;
        mcpwm_timer_t mcpwmTimer;
        mcpwm_io_signals_t signalA, signalB;
        int8_t last_dir = 0;

        const uint16_t freq = 1000; //PWM channel frequency Hz
        // const uint8_t resolution = 8; //PWM channel resolution bits

};
