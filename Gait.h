#ifndef SPIDER_GAIT_H
#define SPIDER_GAIT_H

#include "Types.h"

class GaitGenerator
{
public:

    GaitGenerator();

    // Advance the gait phase
    void update(float dt);

    // Current global gait phase
    float getPhase() const;

    // Foot trajectory for one leg
    FootTrajectory footTrajectory(float phase);

private:

    float phase;
};

#endif