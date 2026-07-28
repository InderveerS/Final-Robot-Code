#pragma once
#include <stdint.h>

// Requires an event predicate to read true on N CONSECUTIVE control cycles
// before the ...Until primitives act on it.
//
// Why: the sensor-backed predicates (IR array, back limit switches) glitch. A
// single noisy sample was enough to end a primitive, which shows up in a log as
// a move or turn that stopped early for no visible reason - the offending
// sample is gone by the next logged row. Requiring a run of trues means one
// stray reading costs nothing, because the count starts over.
//
// The cost is latency: the event is acted on N-1 cycles late, so at the usual
// 10 ms control period N=3 delays a stop by 20 ms. Keep N small.
//
// N = 1 reproduces the original behaviour exactly (fire on the first true
// sample), which is why it is the default at every call site.
class EventDebounce {
    public:
        // needed = consecutive true samples required; 0 is treated as 1.
        explicit EventDebounce(uint16_t needed) : mNeeded(needed ? needed : 1) {}

        // Call at most ONCE per control cycle - the run length is counted in
        // calls, not in time. Returns true on the cycle that completes an
        // unbroken run of `needed` trues. A null predicate never fires, which
        // is how the primitives express "no event, run to the limit".
        bool poll(bool (*event)()) {
            if (event == nullptr) return false;
            if (!event()) { mRun = 0; return false; }
            if (mRun < mNeeded) mRun++; // saturate: a long true run can't wrap
            return mRun >= mNeeded;
        }

    private:
        uint16_t mNeeded;
        uint16_t mRun = 0;
};
