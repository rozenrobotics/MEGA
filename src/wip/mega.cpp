#include <Arduino.h>
#include <IBusBM.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Max72xxPanel.h>

#define rpi  Serial2
#define nano Serial3

const int compassAddr = 0x01;
unsigned long timer = millis();
const byte M[8] = {0b11000011, 0b11000011, 0b11000011, 0b11011011, 0b11111111, 0b11111111, 0b11100111, 0b11000011},
           A[8] = {0b01100110, 0b01100110, 0b01111110, 0b01111110, 0b01100110, 0b01100110, 0b01111110, 0b00111100},
           G[8] = {0b11111111, 0b10100101, 0b11100111, 0b00100100, 0b00100100, 0b00111100, 0b00100100, 0b00011000},
           N[8] = {0b11000011, 0b11000111, 0b11001111, 0b11011111, 0b11111011, 0b11110011, 0b11100011, 0b11000011},
           B[8] = {0b00011110, 0b00111111, 0b10111111, 0b10111111, 0b10111111, 0b10011110, 0b01001100, 0b00111000};

IBusBM ibus;
Max72xxPanel matrix = Max72xxPanel(53, 1, 1);

int st;
int spnano[4];
int error, speed, goal, azimuth;
char boolgun = '0', boolbomb = '0';

int readChannel(byte channel, int min, int max, int defvalue) {
    uint16_t ch = ibus.readChannel(channel);
    if (ch < 100) return defvalue;
    return map(ch, 1000, 2000, min, max);
}

int readbyte() {
    return 100 * (int)Serial2.read()
         + 10  * (int)Serial2.read()
         - 200 + (int)Serial2.read();
}

bool key() {
    return !digitalRead(A0);
}

void mat() {
    matrix.fillScreen(LOW);

    if (st == 0) for (int i = 0; i < 8; ++i) for (int j = 0; j < 8; ++j) matrix.drawPixel(j, i, N[i] & (1 << j));
    if (st == 1) for (int i = 0; i < 8; ++i) for (int j = 0; j < 8; ++j) matrix.drawPixel(j, i, A[i] & (1 << j));
    if (st == 2) {
        if (boolbomb)              for (int i = 0; i < 8; ++i) for (int j = 0; j < 8; ++j) matrix.drawPixel(j, i, B[i] & (1 << j));
        if (boolgun )              for (int i = 0; i < 8; ++i) for (int j = 0; j < 8; ++j) matrix.drawPixel(j, i, G[i] & (1 << j));
        if (!boolgun && !boolbomb) for (int i = 0; i < 8; ++i) for (int j = 0; j < 8; ++j) matrix.drawPixel(j, i, M[i] & (1 << j));
    }

    matrix.write();
}

void debug() {
    Serial2.print(st);            // boat mode
    Serial2.print(azimuth + 100); // azimuth
}

void state() {
    key() ?
    st = 1 : st =    readChannel(8, 0, 2, 1);

    boolgun  = (char)readChannel(7, 0, 1, 1);
    boolbomb = (char)readChannel(6, 0, 1, 1);

    mat();
}

void compass() {
    Wire.beginTransmission(compassAddr);
    Wire.write(0x44);
    Wire.endTransmission();

    Wire.requestFrom(compassAddr, 2);
    while (Wire.available() < 2);

    byte lowbyte = Wire.read();
    byte highbyte = Wire.read();

    azimuth = word(highbyte, lowbyte);
}

void pid() {

}

void neutral() {
    for (int & i : spnano) i = 0;
}

void manual() {
    error = readChannel(3, -100, 100, 0),
    speed = readChannel(1, -100, 100, 0);

    spnano[0] = -speed,
    spnano[1] =  speed,
    spnano[2] = -error,
    spnano[3] =  error;
}

void uartrpi() {
    compass();

    if (rpi.available() > 0) {
        byte strpi = rpi.read();

        if (strpi == 1) for(auto &i : spnano) i = readbyte();
        else {
            if (strpi == 0) error = readbyte(), speed = readbyte();
            if (strpi == 2) goal  = readbyte(), speed = readbyte();

            pid();
        }
        rpi.flush();

        debug();
    }

    
}

void uart2nano() {
    if (timer + 50 < millis()){
        for (auto &i : spnano) nano.print(i + 200);

        nano.print(boolgun + boolbomb);

        timer = millis();
    }
}

void setup() {
    ibus.begin(Serial1); // receiver

    matrix.setIntensity(2); // brightness
    matrix.setRotation (2); // rotation

    Wire.begin();
    Wire.beginTransmission(compassAddr);
    Wire.write(0x01);
    Wire.endTransmission();
    while (Wire.available() > 0) Wire.read();

    Serial.begin(9600); // monitor
    rpi.begin(115200); // uart2rpi
    nano.begin(9600); // strnano

    pinMode(A0, INPUT_PULLUP);
    pinMode(13, OUTPUT);
    digitalWrite(13, 1);

    delay(8000);           // esc init delay
    rpi.flush();
}



void loop() {
    state();

    if (st == 0) neutral(); // neutral     mode
    if (st == 1) uartrpi(); // autonomous mode
    if (st == 2) manual();  // manual      mode

    uart2nano();
}