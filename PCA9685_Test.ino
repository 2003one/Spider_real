/*
 * Set all servos on a PCA9685 to their NEUTRAL (center) position.
 * Board:  ESP8266 (NodeMCU / Wemos D1 mini)
 * Driver: PCA9685 over I2C  (SDA=D2/GPIO4, SCL=D1/GPIO5)
 *
 * Library: "Adafruit PWM Servo Driver Library"
 *   Install via Arduino IDE -> Library Manager -> search "PCA9685".
 */

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// ---- Configuration ----------------------------------------------------
#define I2C_ADDR      0x40    // default PCA9685 address
#define NUM_SERVOS    12      // how many channels you actually use (0..NUM_SERVOS-1)
#define SERVO_FREQ    50      // standard analog servos run at 50 Hz
#define NEUTRAL_US    1500    // pulse width for center (~90 deg); tweak per servo
// -----------------------------------------------------------------------

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(I2C_ADDR);

void setup() {
  Serial.begin(115200);
  delay(200);

  Wire.begin(D2, D1);              // SDA=D2, SCL=D1 on the ESP8266
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);  // improves timing accuracy
  pwm.setPWMFreq(SERVO_FREQ);
  delay(10);

  Serial.println("Centering all servos...");
  for (uint8_t ch = 0; ch < NUM_SERVOS; ch++) {
    pwm.writeMicroseconds(ch, NEUTRAL_US);
    delay(50);                     // small stagger to limit inrush current
  }
  Serial.println("Done. All servos at neutral.");
}

void loop() {
  // Nothing to do — servos stay put at their neutral position.
}