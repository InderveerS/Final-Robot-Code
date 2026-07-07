#pragma once
#include <Arduino.h>
#include <stdint.h>
#include "driver/pcnt.h"

// Wraps one PCNT unit configured for x4 quadrature decoding of a single
// encoder. Provides a 32-bit absolute tick count (extended beyond the
// 16-bit PCNT hardware register via an overflow ISR), plus velocity and
// distance helpers built on top of that tick count.
class Encoder {
    public:
        // pinA/pinB    : quadrature channel GPIOs
        // unit         : PCNT_UNIT_0 .. PCNT_UNIT_3 on ESP32-S3 (only 4 units
        //                total - each Encoder instance needs its own)
        // countsPerRev : encoder ticks per WHEEL revolution - i.e. already
        //                folding in x4 quadrature and any gear ratio
        // wheelCircumferenceM : wheel circumference in meters
        Encoder(uint8_t pinA, uint8_t pinB, pcnt_unit_t unit,
                float countsPerRev, float wheelCircumferenceM);

        void begin();

        // Absolute tick count since the last resetCount() (32-bit - no
        // realistic rollover concern, see write-up).
        int32_t getCount();
        void resetCount();

        // Instantaneous velocity in m/s: (ticks since last call to this
        // function) / dt. dt is supplied by the caller - this function
        // never measures or waits on time itself, so it never blocks.
        float getVelocity(float dt);

        // Distance tracking, in meters, relative to a reference point.
        void startDistance();          // mark the reference point
        float getDistance();           // distance since reference (non-destructive)
        float getDistanceAndReset();   // distance since reference, then re-marks reference here

    private:
        int32_t readAbsoluteTicks();
        static void IRAM_ATTR onOverflow(void* arg);

        uint8_t mPinA;
        uint8_t mPinB;
        pcnt_unit_t mUnit;
        float mMetersPerCount;

        volatile int32_t mAccum = 0;   // updated only in onOverflow()
        portMUX_TYPE mMux = portMUX_INITIALIZER_UNLOCKED;

        int32_t mLastVelocityTicks = 0;
        int32_t mDistanceRefTicks = 0;
};