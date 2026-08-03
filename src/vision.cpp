#include "vision.hpp"
#include "robot.hpp"
#include "telemetry.hpp"

Vision vision;

// ---------------------------------------------------------------------------
// Vision - UART side. Driven only by visionTask.
// ---------------------------------------------------------------------------

bool Vision::parse(const char* line, VisionSample& out) const {
    const char* star = strchr(line, '*');
    if (!star || star == line || !star[1] || !star[2]) return false;

    // NMEA style: XOR of every character before '*', two hex digits after it.
    uint8_t sum = 0;
    for (const char* p = line; p < star; p++) sum ^= (uint8_t)*p;
    if (!isxdigit((unsigned char)star[1]) || !isxdigit((unsigned char)star[2])) return false;
    const char hex[3] = { star[1], star[2], '\0' };
    if ((uint8_t)strtoul(hex, nullptr, 16) != sum) return false;

    unsigned seq, mask, best, conf;
    int cx, cy;
    if (sscanf(line, "V,%u,%u,%u,%u,%d,%d", &seq, &mask, &best, &conf, &cx, &cy) != 6) {
        return false;
    }

    // Range checks, then the cross-check that actually earns its keep: `best`
    // must be one of the bits set in `mask`. That pairing is free from the
    // format and rejects most corruption a 2-bit error slips past the XOR.
    if (seq > 0xFFFFu || mask > 0x0Fu || conf > 255u) return false;
    if (cx < 0 || cx > 2000 || cy < 0 || cy > 2000) return false;
    if (best != TUBBY_NO_COLOR) {
        if (best >= TUBBY_COLOR_COUNT) return false;
        if (!(mask & (1u << best))) return false;
    }

    out.seq = (uint16_t)seq;
    out.mask = (uint8_t)mask;
    out.best = (uint8_t)best;
    out.conf = (uint8_t)conf;
    out.rxMs = millis();
    return true;
}

void Vision::pump() {
    // ONE line per poll() call, so this must loop or we fall behind the sender
    // by exactly the lines we skip.
    while (robotCommunicator.poll()) {
        VisionSample s;
        if (!parse(robotCommunicator.lastMessage(), s)) {
            mBad = mBad + 1;
            continue;
        }
        mGood = mGood + 1;

        portENTER_CRITICAL(&mMux);
        mSample = s;
        mHaveSample = true;
        portEXIT_CRITICAL(&mMux);
    }
}

bool Vision::snapshot(VisionSample& out) {
    bool have;
    portENTER_CRITICAL(&mMux);
    have = mHaveSample;
    if (have) out = mSample;
    portEXIT_CRITICAL(&mMux);
    return have;
}

void visionTask(void* pvParameters) {
    for (;;) {
        vision.pump();
        vTaskDelay(pdMS_TO_TICKS(20)); // ~50 Hz against the CAM's ~15 Hz
    }
}

// ---------------------------------------------------------------------------
// TubbyHunt - mission policy. Lives in missionTask.
// ---------------------------------------------------------------------------

void TubbyHunt::begin() {
    pinMode(cfg::TELETUBBY_LED, OUTPUT);
    digitalWrite(cfg::TELETUBBY_LED, LOW);
}

// Each colour appears exactly once on the course, so a colour we already hold
// is either a false positive or the same teletubby seen twice. Either way it
// must not count.
bool TubbyHunt::recordColor(uint8_t c) {
    if (c >= TUBBY_COLOR_COUNT) return false;
    const uint8_t bit = (uint8_t)(1u << c);
    if (mFoundMask & bit) return false;
    mFoundMask |= bit;
    mFound++;
    return true;
}

void TubbyHunt::flashLed() {
    for (uint8_t i = 0; i < cfg::TUBBY_LED_FLASHES; i++) {
        digitalWrite(cfg::TELETUBBY_LED, HIGH);
        vTaskDelay(pdMS_TO_TICKS(cfg::TUBBY_LED_ON_MS));
        digitalWrite(cfg::TELETUBBY_LED, LOW);
        vTaskDelay(pdMS_TO_TICKS(cfg::TUBBY_LED_ON_MS));
    }
}

LookResult TubbyHunt::look(int spotId) {
    (void)spotId;

#if ESP_LINK_MODE == ESP_LINK_SD
    // No camera on this wire - Serial1 is the SD logger. Hold the original
    // blind dwell so logged diagnostic runs behave exactly as they always have.
    telemetry::pause();
    vTaskDelay(pdMS_TO_TICKS(cfg::TUBBY_DWELL_MS));
    return LookResult::NoData;
#else
    telemetry::pause();

    VisionSample s;
    const bool haveArm = vision.snapshot(s);
    const uint16_t armSeq = haveArm ? s.seq : 0;

    uint8_t candidate = TUBBY_NO_COLOR;
    uint8_t agree = 0;
    uint16_t lastSeq = armSeq;
    bool sawAny = false;

    const uint32_t start = millis();
    uint32_t lastValidMs = start;

    while ((millis() - start) < cfg::TUBBY_LOOK_TIMEOUT_MS) {
        if (vision.snapshot(s) && s.seq != lastSeq) {
            // Frames exposed before the aim turn stopped are blurred across the
            // rotation and belong to no spot. Counting in frames rather than ms
            // self-adapts to the CAM's real rate. The upper bound rejects a seq
            // that is OLDER than the arm point once the 16-bit counter wraps.
            const uint16_t age = (uint16_t)(s.seq - armSeq);
            const bool fresh = !haveArm || (age >= cfg::TUBBY_BLANK_FRAMES && age < 30000);

            if (fresh) {
                lastSeq = s.seq;
                sawAny = true;
                lastValidMs = millis();

                if (s.best != TUBBY_NO_COLOR && s.conf >= cfg::TUBBY_MIN_CONF) {
                    if (s.best == candidate) agree++;
                    else { candidate = s.best; agree = 1; }

                    // Two frames from DIFFERENT seq values must agree. A single
                    // corrupt line that survives the checksum cannot get here.
                    if (agree >= cfg::TUBBY_CONFIRM_FRAMES && recordColor(candidate)) {
                        flashLed();
                        return LookResult::Found;
                    }
                } else {
                    candidate = TUBBY_NO_COLOR; // a clean empty frame breaks the run
                    agree = 0;
                }
            }
        }

        // Nothing valid for this long means the CAM or the wire is gone, not
        // that the spot is empty. Bail early instead of burning the whole
        // timeout at every remaining spot.
        if ((millis() - lastValidMs) > cfg::TUBBY_STALE_MS) return LookResult::NoData;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    return sawAny ? LookResult::Empty : LookResult::NoData;
#endif
}
