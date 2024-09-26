#include <Arduino.h>
#include "Wire.h"
#include "LedControl.h"
#include "PPMReader.h"
#include "Servo.h"

const int16_t COMPASS_ADDRESS = 0x01;
int16_t converted_value;
const uint32_t MID = 1500;
uint16_t raw;
//compass

byte INTERRUPT_PIN = 19;
byte CHANNEL_AMOUNT = 8;
const uint16_t MOTOR_MIDDLE_VALUE = 1500;
const uint8_t MOTOR_MIDDLE_FILTER = 20;

byte CUBE_STATE = 6;
byte WINCH_STATE = 7;
byte MODE_CONTROL = 8;
byte EMPTY = 5;

const uint8_t YAW = 4;
const uint8_t ROLL = 1;
const uint8_t PITCH = 2;
const uint8_t THROTTLE = 3;  // channels

struct
{
  Servo servo_back_right;   // 4
  Servo servo_back_left;    // 5
  Servo servo_front_right;  // 6
  Servo servo_front_left;   // 7   motors
} motors;

struct
{
  Servo servo_cube;
  Servo servo_winch;  ///
} a;

struct
{
  int Speed_back_right;   // 4
  int Speed_back_left;    // 5
  int Speed_front_right;  // 6
  int Speed_front_left;   // 7
} MotorsSpeeds;

struct
{
  uint16_t mode;
  uint16_t winch_state;
  uint16_t cube_state;
  uint16_t empty;

  int16_t yaw;
  int16_t roll;
  int16_t pitch;
  int16_t throttle;  // values
} controlValues;

bool flag_start = false;
uint8_t first_mode;

PPMReader ppm(INTERRUPT_PIN, CHANNEL_AMOUNT);

//control

uint16_t last_roll_val; 
auto last_roll_val_tmr = millis();

void convertedRawValue();
void readCompassSensor();
uint16_t adjustValue(uint16_t value);
uint16_t readPPMData(const uint8_t channel, int8_t min, uint16_t max, uint16_t defvalue);
void read();
void manual();
void move();
void move_stop();

void readCompassSensor() {
  Wire.beginTransmission(COMPASS_ADDRESS);
  Wire.write(0x44);
  Wire.endTransmission();
  Wire.requestFrom(COMPASS_ADDRESS, 2);
  while (Wire.available() < 2)
    ;

  byte lowbyte = Wire.read();
  byte highbyte = Wire.read();

  raw = word(highbyte, lowbyte);
}

void convertedRawValue() {
  if (raw > 180) converted_value = raw - 360;
  else converted_value = raw;
}

uint16_t adjustValue(uint16_t value) {
  if (abs(MOTOR_MIDDLE_VALUE - value) < MOTOR_MIDDLE_FILTER) {
    return MOTOR_MIDDLE_VALUE;
  } else {
    return value;
  }
}

uint16_t readPPMData(const uint8_t channel, int8_t min, uint16_t max, unsigned int defvalue) {
  unsigned value = ppm.latestValidChannelValue(channel, defvalue) + 100;

  if (value < 100)
    return defvalue;

  return (uint16_t)map(value - 100, 1000, 2000, min, max);
}

void read() {
  
    controlValues.cube_state = readPPMData(CUBE_STATE, 0, 1, 1);
    controlValues.winch_state = readPPMData(WINCH_STATE, 0, 2, 1);
    controlValues.mode = readPPMData(MODE_CONTROL, 0, 1, -2);
    controlValues.yaw = readPPMData(YAW, -100, 100, 0);            // поворот
    controlValues.roll = readPPMData(ROLL, -100, 100, 0);          // подворот
    controlValues.pitch = readPPMData(PITCH, -100, 100, 0);        // подворот
    controlValues.throttle = readPPMData(THROTTLE, -100, 100, 0);  // газ скорость

}

void manual() {
  MotorsSpeeds.Speed_back_left = map(constrain(controlValues.throttle - controlValues.yaw, -100, 100), -100, 100, 1000, 2000);
  MotorsSpeeds.Speed_back_right = map(constrain(controlValues.throttle + controlValues.yaw, -100, 100), -100, 100, 1000, 2000);
  MotorsSpeeds.Speed_front_left = map(constrain(controlValues.throttle - controlValues.yaw, -100, 100), -100, 100, 1000, 2000);
  MotorsSpeeds.Speed_front_right = map(constrain(controlValues.throttle + controlValues.yaw, -100, 100), -100, 100, 1000, 2000);
}

void move_stop() {
  MotorsSpeeds.Speed_back_left = MID;
  MotorsSpeeds.Speed_back_right = MID;
  MotorsSpeeds.Speed_front_left = MID;
  MotorsSpeeds.Speed_front_right = MID;
}

void move() {
  motors.servo_back_left.writeMicroseconds(MotorsSpeeds.Speed_back_left);
  motors.servo_back_right.writeMicroseconds(MotorsSpeeds.Speed_back_right);
  motors.servo_front_left.writeMicroseconds(MotorsSpeeds.Speed_front_left);
  motors.servo_front_right.writeMicroseconds(MotorsSpeeds.Speed_front_right);
}

void setup() {
  Wire.begin();
  Wire.beginTransmission(COMPASS_ADDRESS);
  Wire.write(0x00);
  Wire.endTransmission();
  while (Wire.available() > 0)
    Wire.read();
  Serial.begin(9600);

  motors.servo_back_right.attach(4, 1000, 2000);
  motors.servo_back_left.attach(6, 1000, 2000);
  motors.servo_front_right.attach(5, 1000, 2000);
  motors.servo_front_left.attach(7, 1000, 2000);
  delay(7500);

  read();
  first_mode = controlValues.mode;

}

void loop() {
  move_stop();
  read();
  readCompassSensor();
  convertedRawValue();
    if (controlValues.mode == 0) {
      manual();  // manual
    }
  Serial.println(converted_value);
  delay(50);
  move();
}

