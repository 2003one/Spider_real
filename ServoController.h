#ifndef SPIDER_SERVO_CONTROLLER_H
#define SPIDER_SERVO_CONTROLLER_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#include "Types.h"

class ServoController
{
public:
    ServoController();

    void begin();

    void writeServo(uint8_t channel, float angle);

    void writeLeg(uint8_t firstChannel, JointAngles joints);

    void testServo();

private:
    Adafruit_PWMServoDriver pwm;

    uint16_t angleToPulse(uint8_t channel, float angle);
};

#endif
