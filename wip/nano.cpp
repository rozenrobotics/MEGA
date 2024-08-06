#include <Arduino.h>
#include <Servo.h>

#define mega Serial

Servo MotorZR, MotorZL, MotorPR, MotorPL;

int spmega[4];
bool boolgun = false, boolbomb = false;
unsigned long timer = millis();



int readbyte() {
    return 100 * (int)mega.read()
         + 10  * (int)mega.read()
         - 200 + (int)mega.read();
}

void uartmega() {
    for (auto &i : spmega) i = readbyte() - 200;

    boolgun  = mega.read(),
    boolbomb = mega.read();

    timer = millis();
}

void move() {
    MotorZR.writeMicroseconds(map(spmega[0], -100, 100, -500, 500) + 1490);
    MotorZL.writeMicroseconds(map(spmega[1], -100, 100, -500, 500) + 1490);
    MotorPR.writeMicroseconds(map(spmega[2], -100, 100, -500, 500) + 1490);
    MotorPL.writeMicroseconds(map(spmega[3], -100, 100, -500, 500) + 1490);
}

void setup() {
    mega.begin(9600);
    //speed
    MotorZR.attach(8,   1000, 2000); //right
    MotorZL.attach(10,  1000, 2000); //left

    //rul
    MotorPR.attach(9,  1000, 2000); //right
    MotorPL.attach(11, 1000, 2000); //left
    pinMode(13, 1);

    delay(10000);
    timer = millis();
}

void loop() {
    if (mega.available() > 0) uartmega();
    else if (timer + 5000 < millis()) for(auto &i : spmega) i = 0;

    move();
}