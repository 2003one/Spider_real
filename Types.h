#ifndef SPIDER_TYPES_H
#define SPIDER_TYPES_H
#include <Arduino.h>
struct IKAngles
{
    float theta1;
    float theta2;
    float theta3;
};

struct JointAngles
{
    float coxa;
    float femur;
    float tibia;
};

struct FootPosition
{
    float px;
    float py;
    float pz;
};

struct FootTrajectory
{
    float px;
    float lift;
};

struct Leg
{
    const char* name;

    int outwardSign;

    float phaseOffset;

    uint8_t firstChannel;
};

#endif
