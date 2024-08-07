#include <Arduino.h>
#include <Wire.h>

namespace compass 
{
    namespace azimuth 
    {
        int16_t converted_value;
        uint16_t raw;
    }

    const int16_t COMPASS_ADDRESS = 0x01;

    class Compass 
    {
    public:

        static void setup()
        {
            Serial.begin(9600);
            Wire.begin(); 
            Wire.beginTransmission(compass::COMPASS_ADDRESS);   
            Wire.write(0x00); 
            Wire.endTransmission(); 
            while(Wire.available() > 0)
                Wire.read();
        }

        static void read_compass_sensor()
        {
            Wire.beginTransmission(compass::COMPASS_ADDRESS);
            Wire.write(0x44);
            Wire.endTransmission();
            Wire.requestFrom(compass::COMPASS_ADDRESS, 2); 
            while(Wire.available() < 2);

            byte lowbyte = Wire.read();  
            byte highbyte = Wire.read();

            compass::azimuth::raw = word(highbyte, lowbyte); 
        }

        static void converted_raw_value()
        {
          if (compass::azimuth::raw > 180)
          {
            compass::azimuth::converted_value = compass::azimuth::raw - 360;
          }
          else 
          {
            compass::azimuth::converted_value = compass::azimuth::raw;
          }
        }

    };
}

void setup()
{
    compass::Compass::setup();
}

void loop()
{
    compass::Compass::read_compass_sensor();
    compass::Compass::converted_raw_value();
    Serial.print("Raw value: ");
    Serial.print(compass::azimuth::raw);
    Serial.println();
    Serial.print("Converted Value: ");
    Serial.print(compass::azimuth::converted_value);
    Serial.println();
    delay(100);
}
