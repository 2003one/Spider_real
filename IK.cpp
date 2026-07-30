#include "IK.h"
#include "Config.h"

#include <Arduino.h>
#include <math.h>

IKAngles IKSolver::solve(float px, float py, float pz)
{
    IKAngles result;

    //--------------------------------------------------
    // Coxa Angle
    //--------------------------------------------------

    result.theta1 = atan2(py, px);

    //--------------------------------------------------
    // Distance from coxa joint
    //--------------------------------------------------

    float r = sqrt(px * px + py * py);

    float rReach = r - L1;

    float h = pz;

    float d = sqrt(rReach * rReach + h * h);

    //--------------------------------------------------
    // Reachability Check
    //--------------------------------------------------

    if (d > (L2 + L3))
    {
        d = L2 + L3;
    }

    if (d < fabs(L2 - L3))
    {
        d = fabs(L2 - L3);
    }

    //--------------------------------------------------
    // Tibia Angle
    //--------------------------------------------------

    float cosTheta3 =
        (d * d - L2 * L2 - L3 * L3) /
        (2.0f * L2 * L3);

    cosTheta3 = constrain(cosTheta3, -1.0f, 1.0f);

    result.theta3 = acos(cosTheta3);

    //--------------------------------------------------
    // Femur Angle
    //--------------------------------------------------

    float alpha =
        atan2(-h, rReach);

    float beta =
        atan2(
            L3 * sin(result.theta3),
            L2 + L3 * cos(result.theta3));

    result.theta2 =
        alpha - beta;

    return result;
}

JointAngles IKSolver::toServoAngles(
    float theta1,
    float theta2,
    float theta3,
    int outwardSign)
{
    JointAngles joints;

    //--------------------------------------------------
    // Same mapping used in ROS2
    //--------------------------------------------------

    joints.coxa =
        outwardSign *
        (theta1 - PI / 2.0f);

    joints.femur =
        outwardSign *
        (PI / 2.0f - theta2);

    joints.tibia =
        -outwardSign *
        theta3;

    return joints;
}