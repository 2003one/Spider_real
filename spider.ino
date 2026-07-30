#include "Quadruped.h"

Quadruped robot;

unsigned long previousTime = 0;

void setup()
{
    Serial.begin(115200);

    robot.begin();

    previousTime = millis();
}

void loop()
{
    unsigned long currentTime = millis();

    float dt = (currentTime - previousTime) / 1000.0f;

    previousTime = currentTime;

    robot.update(dt);
}