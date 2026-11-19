#ifndef __ENCODER_H
#define __ENCODER_H
#define TIME_SIMPLE 20.0f
#define ALPHA 0.85f

#include <stdint.h>

float GetCurrent_Speed(uint8_t X, uint16_t PPR, uint16_t GR, float Last_Speed);

#endif
