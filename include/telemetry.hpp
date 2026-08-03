#pragma once
#include <stdint.h>
#include "linkMode.hpp"

// Generic, global activity telemetry: the blocking primitives and the mission
// write "what am I doing right now" here, and espLoggingTask reads it.
// Every hook below strips to a no-op when disabled (the empty inline calls
// compile away, so zero runtime cost when logging is off).
//
// Tied to the link mode rather than set by hand: the sd_logging board is the
// only thing that consumes these rows, so in vision mode nothing is listening
// and the rows would just burn CPU on a wire carrying detections the other way.
#define TELEMETRY_ENABLED (ESP_LINK_MODE == ESP_LINK_SD)

namespace telemetry {

typedef bool (*EventFn)();

extern volatile const char* activity; // "MOVE","TURN","FOLLOW","TURN_UNTIL","MOVE_UNTIL","PAUSE",...
extern volatile float goal;           // target distance / heading / max angle
extern volatile bool hasEvent;        // waiting on an event predicate?
extern volatile EventFn eventPtr;     // the event fn (for name lookup), or nullptr
extern volatile int step;             // mission section index, bumped by the mission

#if TELEMETRY_ENABLED
inline void setActivity(const char* a, float g, bool ev, EventFn evp) {
    activity = a; goal = g; hasEvent = ev; eventPtr = evp;
}
inline void pause()       { activity = "PAUSE"; hasEvent = false; eventPtr = nullptr; }
inline void nextStep()    { step = step + 1; }
inline void setStep(int s){ step = s; }
#else
inline void setActivity(const char*, float, bool, EventFn) {}
inline void pause() {}
inline void nextStep() {}
inline void setStep(int) {}
#endif

} // namespace telemetry
