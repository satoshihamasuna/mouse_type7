/*
 * half_float.cpp
 *
 *  Created on: 2024/03/06
 *      Author: sato1
 */
#include "../Inc/half_float.h"
#include <stdint.h>
#include <math.h>

float half_to_float(half_float hf) {
    int sign = (hf.data >> 15) & 0x0001;
    int exponent = (hf.data >> 10) & 0x001F;
    int fraction = hf.data & 0x03FF;

    if (exponent == 0) {
        if (fraction == 0) {
            return sign ? -0.0f : 0.0f;
        } else {
            return (sign ? -1.0f : 1.0f) * fraction * (1.0f / 1024.0f) * powf(2, -14);
        }
    } else if (exponent == 31) {
        if (fraction == 0) {
            return sign ? -INFINITY : INFINITY;
        } else {
            return NAN;
        }
    } else {

        return (sign ? -1.0f : 1.0f) * (1.0f + fraction * (1.0f / 1024.0f)) * powf(2, exponent - 15);
    }
}

// FloatからHalf Floatへの変換
half_float float_to_half(float f) {
    half_float hf;
    uint32_t *f_ptr = (uint32_t *)&f;
    uint32_t f_value = *f_ptr;

    uint32_t sign = (f_value >> 31) & 0x00000001;
    uint32_t exponent = (f_value >> 23) & 0x000000FF;
    uint32_t fraction = f_value & 0x007FFFFF;

    if (exponent == 0) {
        hf.data = (sign << 15) | (0 << 10) | (fraction >> 13);
    } else if (exponent == 255) {
        hf.data = (sign << 15) | (31 << 10) | 0; // Infinite or NaN, exponent is set to 31
    } else {
        hf.data = (sign << 15) | ((exponent - 127 + 15) << 10) | (fraction >> 13);
    }

    return hf;
}

/*

// Function to convert a 16-bit half float to a 32-bit float
half_float float_to_half(float f) {
    half_float hf;
    uint32_t bits = *((uint32_t *)&f);
    uint16_t sign = (bits >> 31) & 0x1;
    uint16_t exponent = ((bits >> 23) & 0xFF) - 127;
    uint32_t mantissa = bits & 0x7FFFFF;

    // Handle special cases: zero, infinity, NaN
    if (exponent == -127) {
        hf.parts.exponent = 0;
        hf.parts.mantissa = 0;
    } else if (exponent == 128) {
        hf.parts.exponent = 31;
        hf.parts.mantissa = 0;
    } else {
        hf.parts.exponent = exponent + 15;
        hf.parts.mantissa = mantissa >> 13;
    }

    hf.parts.sign = sign;
    return hf;
}

// Function to convert a 16-bit half float to a 32-bit float
float half_to_float(half_float hf) {
    uint16_t sign = hf.parts.sign;
    int16_t exponent = hf.parts.exponent - 15;
    uint32_t mantissa = hf.parts.mantissa << 13;
    uint32_t bits = (sign << 31) | ((exponent + 127) << 23) | mantissa;
    return *((float *)&bits);
}

*/


