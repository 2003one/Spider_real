#include "Gait.h"
#include "Config.h"

#include <Arduino.h>
#include <math.h>

GaitGenerator::GaitGenerator()
{
    phase = 0.0f;
}

void GaitGenerator::update(float dt)
{
    phase += 2.0f * PI * GAIT_HZ * dt;

    while (phase >= 2.0f * PI)
    {
        phase -= 2.0f * PI;
    }
}

float GaitGenerator::getPhase() const
{
    return phase;
}

FootTrajectory GaitGenerator::footTrajectory(float t)
{
    FootTrajectory foot;

    //--------------------------------------------------
    // Swing Phase
    //--------------------------------------------------

    if (t < PI)
    {
        float tNorm = t / PI;

        foot.px =
            STEP_LENGTH *
            (tNorm - 0.5f);

        foot.lift =
            STEP_HEIGHT *
            sin(PI * tNorm);
    }

    //--------------------------------------------------
    // Stance Phase
    //--------------------------------------------------

    else
    {
        float tNorm =
            (t - PI) / PI;

        foot.px =
            STEP_LENGTH *
            (0.5f - tNorm);

        foot.lift = 0.0f;
    }

    return foot;
}