#include "telemetry.hpp"

namespace telemetry {
volatile const char* activity = "INIT";
volatile float goal = 0.0f;
volatile bool hasEvent = false;
volatile EventFn eventPtr = nullptr;
volatile int step = 0;
}
