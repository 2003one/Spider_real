#ifndef SPIDER_CONFIG_H
#define SPIDER_CONFIG_H

//======================================================
// Robot Dimensions (meters)
//======================================================

constexpr float L1 = 0.035f;
constexpr float L2 = 0.070f;
constexpr float L3 = 0.075f;

//======================================================
// Home Position
//======================================================

constexpr float HOME_PY = 0.08f;
constexpr float HOME_Z  = -0.10f;

//======================================================
// Walking Parameters
//======================================================

constexpr float STEP_LENGTH = 0.03f;
constexpr float STEP_HEIGHT = 0.015f;

// Gait cycles per second. 0.0 = frozen (phase never
// advances, robot stands still). Start slow and tune up.
constexpr float GAIT_HZ = 0.5f;

//======================================================
// Startup
//======================================================

// Seconds to fold from zero pose (legs straight)
// into the home stance
constexpr float STAND_UP_TIME = 2.0f;

//======================================================
// Servo PWM
//======================================================

constexpr int SERVO_MIN = 110;
constexpr int SERVO_MAX = 490;

constexpr int SERVO_FREQ = 50;    // ← THIS LINE MUST EXIST

//======================================================
// Bring-up Mode
//
// 1 = hold ALL servos at neutral (0 rad).
//     Use this to mount horns and tune
//     SERVO_OFFSET in Calibration.h.
// 0 = normal walking.
//======================================================

#define CALIBRATION_MODE 0

#endif
