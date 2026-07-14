#include "encoder.hpp"

Encoder::Encoder(uint8_t pinA, uint8_t pinB, pcnt_unit_t unit,
                  float countsPerRev, float wheelCircumferenceM)
    : mPinA(pinA), mPinB(pinB), mUnit(unit),
      mMetersPerCount(wheelCircumferenceM / countsPerRev) {}

void Encoder::begin() {
    // Channel 0: A is the edge signal, B is the level signal
    pcnt_config_t chanA = {};
    chanA.pulse_gpio_num = mPinA;
    chanA.ctrl_gpio_num  = mPinB;
    chanA.unit    = mUnit;
    chanA.channel = PCNT_CHANNEL_0;
    chanA.pos_mode = PCNT_COUNT_INC;
    chanA.neg_mode = PCNT_COUNT_DEC;
    chanA.lctrl_mode = PCNT_MODE_REVERSE;
    chanA.hctrl_mode = PCNT_MODE_KEEP;
    chanA.counter_h_lim = INT16_MAX;
    chanA.counter_l_lim = INT16_MIN;
    pcnt_unit_config(&chanA);

    // Channel 1: same unit, roles of A/B swapped, gets x4 decoding instead of x1
    pcnt_config_t chanB = chanA;
    chanB.pulse_gpio_num = mPinB;
    chanB.ctrl_gpio_num  = mPinA;
    chanB.channel = PCNT_CHANNEL_1;
    chanB.pos_mode = PCNT_COUNT_DEC;
    chanB.neg_mode = PCNT_COUNT_INC;
    // chanB.lctrl_mode = PCNT_MODE_KEEP;
    // chanB.hctrl_mode = PCNT_MODE_REVERSE;
    pcnt_unit_config(&chanB);

    pcnt_set_filter_value(mUnit, 250); // TODO: tune this for encoder
    pcnt_filter_enable(mUnit);

    // Interrupt only when the 16-bit hardware register itself wraps around -
    // NOT once per encoder edge. 
    pcnt_event_enable(mUnit, PCNT_EVT_H_LIM);
    pcnt_event_enable(mUnit, PCNT_EVT_L_LIM);

    // pcnt_isr_service_install() is global to the whole PCNT peripheral,
    // not per-unit - only the first Encoder instance should actually install it.
    static bool isrServiceInstalled = false;
    if (!isrServiceInstalled) {
        pcnt_isr_service_install(0);
        isrServiceInstalled = true;
    }
    pcnt_isr_handler_add(mUnit, &Encoder::onOverflow, this);

    pcnt_counter_pause(mUnit);
    pcnt_counter_clear(mUnit);
    pcnt_counter_resume(mUnit);

    gpio_pullup_en((gpio_num_t)mPinA);
    gpio_pullup_en((gpio_num_t)mPinB);

    portMUX_TYPE mMux = portMUX_INITIALIZER_UNLOCKED;
}

void IRAM_ATTR Encoder::onOverflow(void* arg) {
    Encoder* self = static_cast<Encoder*>(arg);
    uint32_t status;
    pcnt_get_event_status(self->mUnit, &status);
    portENTER_CRITICAL_ISR(&self->mMux);
    if (status & PCNT_EVT_H_LIM) self->mAccum += INT16_MAX;
    if (status & PCNT_EVT_L_LIM) self->mAccum += INT16_MIN;
    portEXIT_CRITICAL_ISR(&self->mMux);
}

// Combines the ISR-maintained overflow accumulator with the live hardware
// register into one 32-bit absolute tick count. Never clears the hardware
// register, so getVelocity() and getDistance() can each call this
// independently without fighting over who "owns" resetting it.
int32_t Encoder::readAbsoluteTicks() {
    int16_t raw;
    int32_t accumSnapshot;
    portENTER_CRITICAL(&mMux);
    pcnt_get_counter_value(mUnit, &raw);
    accumSnapshot = mAccum;
    portEXIT_CRITICAL(&mMux);
    return accumSnapshot + raw;
}

int32_t Encoder::getCount() {
    return readAbsoluteTicks();
}

void Encoder::resetCount() {
    pcnt_counter_pause(mUnit);
    portENTER_CRITICAL(&mMux);
    mAccum = 0;
    portEXIT_CRITICAL(&mMux);
    pcnt_counter_clear(mUnit);
    pcnt_counter_resume(mUnit);
    mLastVelocityTicks = 0;
    mDistanceRefTicks = 0;
}

float Encoder::getVelocity(float dt) {
    int32_t now = readAbsoluteTicks();
    int32_t deltaTicks = now - mLastVelocityTicks;
    mLastVelocityTicks = now;
    if (dt <= 0.0f) return 0.0f;
    return (deltaTicks * mMetersPerCount) / dt;
}

void Encoder::startDistance() {
    mDistanceRefTicks = readAbsoluteTicks();
}

float Encoder::getDistance() {
    return (readAbsoluteTicks() - mDistanceRefTicks) * mMetersPerCount;
}

float Encoder::getDistanceAndReset() {
    int32_t now = readAbsoluteTicks();
    float d = (now - mDistanceRefTicks) * mMetersPerCount;
    mDistanceRefTicks = now;
    return d;
}