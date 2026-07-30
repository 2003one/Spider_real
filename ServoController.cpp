#include "ServoController.h"
#include "Config.h"
#include "Calibration.h"

ServoController::ServoController()
: pwm(0x40)
{
}

void ServoController::begin()
{
    // ESP8266
    Wire.begin(4, 5);

    pwm.begin();

    pwm.setPWMFreq(SERVO_FREQ);

    delay(100);
}

uint16_t ServoController::angleToPulse(
    uint8_t channel,
    float angle)
{
    //--------------------------------------------------
    // Per-servo direction (from Calibration.h)
    //
    // In Gazebo the URDF mirrored the right-leg joint
    // axes for us. On hardware, whether a servo needs
    // flipping depends on how it is physically mounted,
    // so it is corrected here, per channel.
    //--------------------------------------------------

    angle *= SERVO_DIRECTION[channel];

    //--------------------------------------------------
    // Convert radians to degrees
    //--------------------------------------------------

    float degrees = angle * 180.0f / PI;

    //--------------------------------------------------
    // Per-servo mounting offset (degrees)
    //--------------------------------------------------

    degrees += SERVO_OFFSET[channel];

    //--------------------------------------------------
    // Shift from [-90,+90] to [0,180]
    // (0 rad = mechanical neutral = pulse midpoint)
    //--------------------------------------------------

    degrees += 90.0f;

    degrees = constrain(degrees, 0.0f, 180.0f);

    //--------------------------------------------------
    // Float mapping — the integer map() was throwing
    // away sub-degree resolution
    //--------------------------------------------------

    float pulse =
        SERVO_MIN +
        (degrees / 180.0f) *
        (SERVO_MAX - SERVO_MIN);

    return (uint16_t)(pulse + 0.5f);
}

void ServoController::writeServo(
    uint8_t channel,
    float angle)
{
    pwm.setPWM(
        channel,
        0,
        angleToPulse(channel, angle));
}

void ServoController::writeLeg(
    uint8_t firstChannel,
    JointAngles joints)
{
    writeServo(firstChannel, joints.coxa);

    writeServo(firstChannel + 1, joints.femur);

    writeServo(firstChannel + 2, joints.tibia);
}

void ServoController::testServo()
{
    //--------------------------------------------------
    // ONE sweep of channel 0, then return.
    // (Previously while(true) — begin() never
    //  returned and the gait never started.)
    //--------------------------------------------------

    pwm.setPWM(0, 0, SERVO_MIN);   // One end
    delay(1000);

    pwm.setPWM(0, 0, (SERVO_MIN + SERVO_MAX) / 2);   // Middle
    delay(1000);

    pwm.setPWM(0, 0, SERVO_MAX);   // Other end
    delay(1000);

    pwm.setPWM(0, 0, (SERVO_MIN + SERVO_MAX) / 2);   // Back to neutral
    delay(500);

}
