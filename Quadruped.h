#ifndef SPIDER_QUADRUPED_H
#define SPIDER_QUADRUPED_H

#include "IK.h"
#include "Gait.h"
#include "ServoController.h"
#include "Types.h"

class Quadruped
{
public:

    Quadruped();

    void begin();

    void update(float dt);

private:

    void writeAllZero();

    void homeJoints(JointAngles target[4]);

    void standUp(float seconds);

    IKSolver ik;

    GaitGenerator gait;

    ServoController servo;

    Leg legs[4];
};

#endif
