#include "Quadruped.h"

#include "Config.h"

#include <Arduino.h>
#include <math.h>

Quadruped::Quadruped()
{
    //--------------------------------------------------
    // Same leg table as the ROS2 walk_forward node:
    //
    //   outward_sign : +1 left legs, -1 right legs
    //   phase_offset : trot — FL+RR together,
    //                         FR+RL together
    //--------------------------------------------------

    legs[0] = { "FL",  1, 0.0f, 0 };   // Front Left
    legs[1] = { "FR", -1, PI,   3 };   // Front Right
    legs[2] = { "RL",  1, PI,   6 };   // Rear Left
    legs[3] = { "RR", -1, 0.0f, 9 };   // Rear Right
}

void Quadruped::begin()
{
    servo.begin();

#if CALIBRATION_MODE

    Serial.println("CALIBRATION MODE - holding neutral");

#else

    //--------------------------------------------------
    // 1. Zero pose — legs straight down
    //    (the URDF zero configuration, same pose
    //     the robot spawns in inside Gazebo)
    //--------------------------------------------------

    Serial.println("Zero pose - legs straight");

    writeAllZero();

    delay(2000);

    //--------------------------------------------------
    // 2. Slowly fold into the home stance
    //    (like stand.py in the simulation)
    //--------------------------------------------------

    Serial.println("Standing up...");

    standUp(STAND_UP_TIME);

    delay(1000);

    Serial.println("Starting trot gait");

#endif
}

void Quadruped::writeAllZero()
{
    for(int ch = 0; ch < 12; ch++)
    {
        servo.writeServo(ch, 0.0f);

        //--------------------------------------------------
        // One at a time — gentle on the power supply
        //--------------------------------------------------

        delay(100);
    }
}

void Quadruped::homeJoints(JointAngles target[4])
{
    //--------------------------------------------------
    // Home = foot at (0, HOME_PY, HOME_Z)
    //--------------------------------------------------

    IKAngles ikAngles =
        ik.solve(
            0.0f,
            HOME_PY,
            HOME_Z);

    for(int i = 0; i < 4; i++)
    {
        target[i] =
            ik.toServoAngles(
                ikAngles.theta1,
                ikAngles.theta2,
                ikAngles.theta3,
                legs[i].outwardSign);
    }
}

void Quadruped::standUp(float seconds)
{
    //--------------------------------------------------
    // Linear interpolation in joint space:
    // all joints go 0 -> home together, at 50 Hz
    //--------------------------------------------------

    JointAngles target[4];

    homeJoints(target);

    unsigned long start = millis();

    unsigned long duration =
        (unsigned long)(seconds * 1000.0f);

    while(true)
    {
        unsigned long elapsed = millis() - start;

        float s = (float)elapsed / (float)duration;

        if(s > 1.0f)
        {
            s = 1.0f;
        }

        //--------------------------------------------------
        // Ease in/out (smoothstep) — no jerk at the
        // start or end of the motion
        //--------------------------------------------------

        float k = s * s * (3.0f - 2.0f * s);

        for(int i = 0; i < 4; i++)
        {
            JointAngles j;

            j.coxa  = k * target[i].coxa;
            j.femur = k * target[i].femur;
            j.tibia = k * target[i].tibia;

            servo.writeLeg(
                legs[i].firstChannel,
                j);
        }

        if(s >= 1.0f)
        {
            break;
        }

        delay(20);   // 50 Hz
    }
}

void Quadruped::update(float dt)
{
#if CALIBRATION_MODE

    //--------------------------------------------------
    // Hold every servo at neutral — with SERVO_OFFSET
    // and SERVO_DIRECTION applied — so horns can be
    // mounted and offsets tuned safely.
    //--------------------------------------------------

    for(int ch = 0; ch < 12; ch++)
    {
        servo.writeServo(ch, 0.0f);
    }

    delay(100);

    return;

#endif

    //--------------------------------------------------
    // Update gait phase
    //--------------------------------------------------

    gait.update(dt);

    float phase = gait.getPhase();

    //--------------------------------------------------
    // Process every leg
    //--------------------------------------------------

    for(int i = 0; i < 4; i++)
    {
        float legPhase =
            phase +
            legs[i].phaseOffset;

        while(legPhase >= 2.0f * PI)
        {
            legPhase -=
                2.0f * PI;
        }

        //--------------------------------------------------
        // Foot trajectory
        //--------------------------------------------------

        FootTrajectory foot =
            gait.footTrajectory(
                legPhase);

        //--------------------------------------------------
        // Inverse Kinematics
        // (identical call to the ROS2 node:
        //  ik_solve(px_off, HOME_PY, HOME_Z + lift))
        //--------------------------------------------------

        IKAngles ikAngles =
            ik.solve(
                foot.px,
                HOME_PY,
                HOME_Z + foot.lift);

        //--------------------------------------------------
        // Convert to servo angles
        // (outward_sign applied to ALL THREE joints)
        //--------------------------------------------------

        JointAngles joints =
            ik.toServoAngles(
                ikAngles.theta1,
                ikAngles.theta2,
                ikAngles.theta3,
                legs[i].outwardSign);

        //--------------------------------------------------
        // Send to PCA9685
        //--------------------------------------------------

        servo.writeLeg(
            legs[i].firstChannel,
            joints);
    }
}
