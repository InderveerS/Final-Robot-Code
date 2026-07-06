#include "encoder.hpp"


Encoder::Encoder(uint8_t pinA, uint8_t pinB) : mPinA(pinA), mPinB(pinB) {
}

void Encoder::begin() {

}

int32_t Encoder::getCount() {
    return mCount;
}

void Encoder::resetCount() {
    mCount = 0;
}
