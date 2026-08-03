#pragma once
#include <Arduino.h>
#include "config.hpp"

// Teletubby detection over the ESP-CAM UART.
//
// Wire format from TestBlobs2.0 (see its vision_link.h), one ASCII line:
//   V,<seq>,<mask>,<best>,<conf>,<cx>,<cy>*<XOR>
// It streams continuously at ~15 Hz and reports negatives (mask=0, best=255),
// which is what lets us tell "nothing at this spot" from "no camera".
//
// Split by owner: Vision owns the UART and is driven ONLY by visionTask.
// TubbyHunt is mission policy and lives in missionTask. Nothing else touches
// either.

// MUST match ColorId in TestBlobs2.0/include/color_types.h.
enum TubbyColor : uint8_t {
    TUBBY_RED = 0, TUBBY_GREEN, TUBBY_PURPLE, TUBBY_YELLOW, TUBBY_COLOR_COUNT
};
constexpr uint8_t TUBBY_NO_COLOR = 0xFF;

struct VisionSample {
    uint16_t seq;  // CAM frameId & 0xFFFF; a stuck value means a stalled pipeline
    uint8_t mask;  // bit per confirmed colour, 0 = nothing seen
    uint8_t best;  // highest-confidence colour, or TUBBY_NO_COLOR
    uint8_t conf;  // 0-255, raw so the threshold lives on this board
    uint32_t rxMs; // when WE received it
};

class Vision {
    public:
        // visionTask ONLY - HardwareSerial is not safe from two tasks.
        void pump();

        // Safe from any task. False if no valid line has ever arrived.
        bool snapshot(VisionSample& out);

        // Checksum/structure failures vs accepted lines. A climbing bad count
        // is the signal that ESP_BAUD is too high for the wire - it is the only
        // way to tell a noisy link from a quiet one.
        uint32_t goodCount() const { return mGood; }
        uint32_t badCount() const { return mBad; }

    private:
        bool parse(const char* line, VisionSample& out) const;

        // The sample is 5 fields written on core 0 and read on core 1. A plain
        // volatile struct can tear - a colour from one frame landing on the
        // timestamp of the next - so the copy is done under a spinlock. It runs
        // at ~50 Hz against ~100 Hz, so the cost is nothing.
        portMUX_TYPE mMux = portMUX_INITIALIZER_UNLOCKED;
        VisionSample mSample = {};
        bool mHaveSample = false;

        // Bumped on core 0, read from loop() on core 1. Aligned 32-bit access
        // is atomic on Xtensa, so no lock - just keep the compiler honest.
        volatile uint32_t mGood = 0;
        volatile uint32_t mBad = 0;
};

extern Vision vision;

// Drains and parses the link. Started from setup() in vision mode INSTEAD of
// espLoggingTask - the two share Serial1 and cannot both run.
void visionTask(void* pvParameters);

enum class LookResult {
    Found,  // a new colour was confirmed here
    Empty,  // the camera answered and saw nothing worth believing
    NoData  // nothing valid arrived: link or CAM is down, NOT an empty spot
};

class TubbyHunt {
    public:
        void begin(); // LED pin

        // False once both teletubbies are accounted for. Every scan block in
        // mission.cpp is guarded on this, and it also drives stopAtEnd on the
        // follow leading into each spot so a skipped spot costs no deceleration.
        bool hunting() const { return mFound < cfg::TUBBY_TARGET_COUNT; }
        uint8_t found() const { return mFound; }

        // Blocks at the current pose until the camera answers or it times out.
        // Records the colour and flashes the LED on a confirmed new find.
        LookResult look(int spotId);

    private:
        bool recordColor(uint8_t c);
        void flashLed();

        uint8_t mFound = 0;
        uint8_t mFoundMask = 0; // bit per colour already recorded
};
