#include "irArray.hpp"

IRArray :: IRArray() {
    adc1_config_width(ADC_WIDTH_BIT_12);

    adc1_config_channel_atten(FAR_LEFT_CH, ADC_ATTEN_DB_11);
    adc1_config_channel_atten(LEFT_CH, ADC_ATTEN_DB_11);
    adc1_config_channel_atten(CENTER_CH, ADC_ATTEN_DB_11);
    adc1_config_channel_atten(RIGHT_CH, ADC_ATTEN_DB_11);
    adc1_config_channel_atten(FAR_RIGHT_CH, ADC_ATTEN_DB_11);
}


uint16_t IRArray :: normalize(uint16_t value, uint16_t min, uint16_t max) {
    if (max <= min) {
        return 0;
    }
    else if (value < min) {
        return 0;
    } else if (value > max) {
        return 1000;
    } else {
        return (value - min) * 1000 / (max - min);
    }
}

void IRArray :: calibrateMiddle() {
    uint16_t l = adc1_get_raw(LEFT_CH), c = adc1_get_raw(CENTER_CH), r = adc1_get_raw(RIGHT_CH);
    if (l < minL) minL = l;  if (l > maxL) maxL = l;
    if (c < minC) minC = c;  if (c > maxC) maxC = c;
    if (r < minR) minR = r;  if (r > maxR) maxR = r;
}    

float IRArray :: readLine() {
    uint16_t left = adc1_get_raw(LEFT_CH);
    uint16_t center = adc1_get_raw(CENTER_CH);
    uint16_t right = adc1_get_raw(RIGHT_CH);

    // Normalize the readings to a scale of 0 to 1000
    left = normalize(left, minL, maxL);
    center = normalize(center, minC, maxC);
    right = normalize(right, minR, maxR);

    uint16_t total = left + center + right;

    if (total >= LINE_PRESENT_THRESHOLD) {
        float error = (left * -width) + (right * width);
        lastDirection = (error < 0) ? -1 : ((error > 0) ? 1 : 0);
        return error / total; // position is error/total
    } else {
        return lastDirection * MAX_ERROR; // No line detected, go in last known direction extremely
    }   
}

void IRArray :: calibrateFarLeft() {
    uint16_t fl = adc1_get_raw(FAR_LEFT_CH);
    if (fl < minFL) minFL = fl;  if (fl > maxFL) maxFL = fl;
}

void IRArray :: calibrateFarRight() {
    uint16_t fr = adc1_get_raw(FAR_RIGHT_CH);
    if (fr < minFR) minFR = fr;  if (fr > maxFR) maxFR = fr;
}

uint16_t IRArray :: readFarLeft() {
    return normalize(adc1_get_raw(FAR_LEFT_CH), minFL, maxFL);
}

uint16_t IRArray :: readFarRight() {
    return normalize(adc1_get_raw(FAR_RIGHT_CH), minFR, maxFR);
}