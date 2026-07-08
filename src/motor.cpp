#include "motor.hpp"
#include <math.h>

Motor::Motor(uint8_t mPin1, uint8_t mPin2, bool mInverted, mcpwm_unit_t mcpwmUnit, mcpwm_timer_t mcpwmTimer) {
    this->mPin1 = mPin1;  
    this->mPin2 = mPin2;  
    this->mInverted = mInverted;
    this->mcpwmUnit = mcpwmUnit;
    this->mcpwmTimer = mcpwmTimer;

    // Dynamically assign the correct operator signals based on the timer
    switch (mcpwmTimer) {
        case MCPWM_TIMER_0:
            signalA = MCPWM0A;
            signalB = MCPWM0B;
            break;
        case MCPWM_TIMER_1:
            signalA = MCPWM1A;
            signalB = MCPWM1B;
            break;
        case MCPWM_TIMER_2:
            signalA = MCPWM2A;
            signalB = MCPWM2B;
            break;
        default:
            signalA = MCPWM0A;
            signalB = MCPWM0B;
            break;
    }

    mcpwm_gpio_init(mcpwmUnit, signalA, mPin1); 
    mcpwm_gpio_init(mcpwmUnit, signalB, mPin2); 

    mcpwm_config_t pwm_config;
    pwm_config.frequency = freq; 
    pwm_config.cmpr_a = 0.0;     // Start at 0% (All MOSFETs OFF)
    pwm_config.cmpr_b = 0.0;     // Start at 0% (All MOSFETs OFF)
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    
    mcpwm_init(mcpwmUnit, mcpwmTimer, &pwm_config);
}

void Motor :: stop() {
    mcpwm_set_duty(mcpwmUnit, mcpwmTimer, MCPWM_OPR_A, 0.0);
    mcpwm_set_duty(mcpwmUnit, mcpwmTimer, MCPWM_OPR_B, 0.0);
}

void Motor::setPWMPercent(int8_t percent) {
    // Clamp the value
    if(percent > 100) percent = 100;
    if(percent < -100) percent = -100;

    // Handle robot chassis inversion
    if (mInverted) {
        percent = -percent;
    }

    float active_duty = abs(percent);

    // 1 (Forward), -1 (Reverse), 0 (Stop)
    int8_t current_dir = (percent > 0) ? 1 : ((percent < 0) ? -1 : 0);

    // SOFTWARE DEAD-TIME INJECTION
    // Triggered only when flipping between Forward and Reverse
    if (current_dir != last_dir && last_dir != 0 && current_dir != 0) {
        
        // Force both pins LOW (Turns off all MOSFETs to prevent shoot-through)
        mcpwm_set_duty(mcpwmUnit, mcpwmTimer, MCPWM_OPR_A, 0.0);
        mcpwm_set_duty(mcpwmUnit, mcpwmTimer, MCPWM_OPR_B, 0.0);
        
        // Hold for 500 microseconds to let the MOSFET gates fully discharge
        delayMicroseconds(500); // TODO: DOUBLE CHECK THIS IN CLASS BEFORE USING
    }
    last_dir = current_dir;

    // Apply the driving logic
    if (percent > 0) {
        // Forward: Hold PWM1 LOW, pulse PWM0
        mcpwm_set_duty(mcpwmUnit, mcpwmTimer, MCPWM_OPR_B, 0.0);
        mcpwm_set_duty(mcpwmUnit, mcpwmTimer, MCPWM_OPR_A, active_duty);
        
    } else if (percent < 0) {
        // Reverse: Hold PWM0 LOW, pulse PWM1
        mcpwm_set_duty(mcpwmUnit, mcpwmTimer, MCPWM_OPR_A, 0.0);
        mcpwm_set_duty(mcpwmUnit, mcpwmTimer, MCPWM_OPR_B, active_duty);
        
    } else {
        // Coast: Hold both pins LOW
        mcpwm_set_duty(mcpwmUnit, mcpwmTimer, MCPWM_OPR_A, 0.0);
        mcpwm_set_duty(mcpwmUnit, mcpwmTimer, MCPWM_OPR_B, 0.0);
    }
}

