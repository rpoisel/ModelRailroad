/***************************************************
  This is an example for our Adafruit 16-channel PWM & Servo driver
  Servo test - this will drive 8 servos, one after the other on the
  first 8 pins of the PCA9685

  Pick one up today in the adafruit shop!
  ------> http://www.adafruit.com/products/815

  These drivers use I2C to communicate, 2 pins are required to
  interface.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ****************************************************/

#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
// you can also call it with a different address you want
// Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x41);
// you can also call it with a different address and I2C interface
// Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40, Wire);

// Depending on your servo make, the pulse width min and max may vary.
// Tune these as necessary to avoid hitting the servo's mechanical stops.
constexpr uint16_t SERVO_MIN_US = 600;
constexpr uint16_t SERVO_MAX_US = 2400;
constexpr uint8_t SERVO_MIN_DEG = 0;
constexpr uint8_t SERVO_MAX_DEG = 180;
constexpr uint8_t SERVO_FREQ = 50; // Analog servos run at ~50 Hz updates

// our servo # counter
uint8_t servonum = 0;

void setup() {
  Serial.begin(9600);
  Serial.println("8 channel Servo test!");

  pwm.begin();
  /*
   * In theory the internal oscillator (clock) is 25MHz but it really isn't
   * that precise. You can 'calibrate' this by tweaking this number until
   * you get the PWM update frequency you're expecting!
   * The int.osc. for the PCA9685 chip is a range between about 23-27MHz and
   * is used for calculating things like writeMicroseconds()
   * Analog servos run at ~50 Hz updates, It is importaint to use an
   * oscilloscope in setting the int.osc frequency for the I2C PCA9685 chip.
   * 1) Attach the oscilloscope to one of the PWM signal pins and ground on
   *    the I2C PCA9685 chip you are setting the value for.
   * 2) Adjust setOscillatorFrequency() until the PWM update frequency is the
   *    expected value (50Hz for most ESCs)
   * Setting the value here is specific to each individual I2C PCA9685 chip and
   * affects the calculations for the PWM update frequency.
   * Failure to correctly set the int.osc value will cause unexpected PWM
   * results
   */
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ); // Analog servos run at ~50 Hz updates

  delay(10);
}

uint16_t angleToMicroseconds(uint8_t angle_deg) {
  angle_deg = constrain(angle_deg, SERVO_MIN_DEG, SERVO_MAX_DEG);

  return map(angle_deg, SERVO_MIN_DEG, SERVO_MAX_DEG, SERVO_MIN_US,
             SERVO_MAX_US);
}

void writeServoAngle(uint8_t channel, uint8_t angle_deg) {
  uint16_t const pulse_us = angleToMicroseconds(angle_deg);

  Serial.print("channel=");
  Serial.print(channel);
  Serial.print(" angle=");
  Serial.print(angle_deg);
  Serial.print(" pulse_us=");
  Serial.println(pulse_us);

  pwm.writeMicroseconds(channel, pulse_us);
}

void loop() {
  // Drive each servo one at a time using angle-based writeMicroseconds().
  Serial.println(servonum);
  for (uint8_t angle = SERVO_MIN_DEG; angle <= SERVO_MAX_DEG; angle++) {
    writeServoAngle(servonum, angle);
    delay(10);
  }

  delay(500);
  for (uint8_t angle = SERVO_MAX_DEG; angle > SERVO_MIN_DEG; angle--) {
    writeServoAngle(servonum, angle);
    delay(10);
  }
  writeServoAngle(servonum, SERVO_MIN_DEG);

  delay(500);

  writeServoAngle(servonum, 57);

  delay(500);
  writeServoAngle(servonum, 90);

  delay(500);

  servonum++;
  if (servonum > 1)
    servonum = 0; // Testing the first 8 servo channels
}
