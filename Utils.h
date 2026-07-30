#ifndef SPIDER_UTILS_H
#define SPIDER_UTILS_H

#include <Arduino.h>
#include <math.h>

namespace Utils
{
    // Convert radians to degrees
    inline float radToDeg(float radians)
    {
        return radians * 180.0f / PI;
    }

    // Convert degrees to radians
    inline float degToRad(float degrees)
    {
        return degrees * PI / 180.0f;
    }

    // Clamp a float value
    inline float clamp(float value, float minimum, float maximum)
    {
        if (value < minimum)
            return minimum;

        if (value > maximum)
            return maximum;

        return value;
    }

    // Linear mapping for float values
    inline float mapFloat(
        float x,
        float in_min,
        float in_max,
        float out_min,
        float out_max)
    {
        return (x - in_min) *
               (out_max - out_min) /
               (in_max - in_min) +
               out_min;
    }

    // Normalize an angle to the range [0, 2π)
    inline float normalizeAngle(float angle)
    {
        while (angle >= 2.0f * PI)
            angle -= 2.0f * PI;

        while (angle < 0.0f)
            angle += 2.0f * PI;

        return angle;
    }
}

#endif