#ifndef SPIDER_IK_H
#define SPIDER_IK_H

#include "Types.h"

class IKSolver
{
public:

    IKAngles solve(
        float px,
        float py,
        float pz);

    JointAngles toServoAngles(
        float theta1,
        float theta2,
        float theta3,
        int outwardSign);
};

#endif
