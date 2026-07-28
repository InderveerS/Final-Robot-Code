#pragma once
#include "telemetry.hpp"

// The competition mission: a blocking move/turn/follow sequence. Runs once,
// then parks with the motors stopped. Start it as a task from setup().
void missionTask(void* pvParameters);

// Maps an event predicate function pointer to a short human name (for logging).
const char* eventName(telemetry::EventFn p);
