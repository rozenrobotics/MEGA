#include <Arduino.h>
#include <LedControl.h>

const uint8_t MATRIX_PIN_DIN = 7;
const uint8_t MATRIX_PIN_CS  = 6;
const uint8_t MATRIX_PIN_CLK = 5;

namespace indication 
{
    namespace symbols 
    {
        const byte A[8] = {
            0b00000000,
            0b11111110,
            0b11111111,
            0b00011001,
            0b00011001,
            0b11111111,
            0b11111110,
            0b00000000
        };

        const byte M[8] = {
            0b11111100,
            0b11111111,
            0b00000111,
            0b00001110,
            0b00001110,
            0b00000111,
            0b11111111,
            0b11111100
        };

        const byte N[8] = {
            0b11111111,
            0b11111111,
            0b00001110,
            0b00011100,
            0b00111000,
            0b01110000,
            0b11111111,
            0b11111111
        };

        const byte CUBE[8] = {
          0b11111100,
          0b10000110,
          0b10000101,
          0b10000101,
          0b10000101,
          0b11111101,
          0b01000011,
          0b00111111
        };

        const byte NET[8] = {
          0b01010101,
          0b10101010,
          0b01010101,
          0b10101010,
          0b01010101,
          0b10101010,
          0b01010101,
          0b10101010
        };

        const uint8_t POKEBALL[8] = {
          0b00111100,
          0b01011010,
          0b10011001,
          0b10100101,
          0b10100101,
          0b10011001,
          0b01011010,
          0b00111100
        };
    }

    

    const uint8_t BRIGHTNESS = 1;
    LedControl matrix = LedControl(MATRIX_PIN_DIN, MATRIX_PIN_CLK, MATRIX_PIN_CS, 0);

    class Matrix 
    {
        public:

            static void setup()
            {
                matrix.clearDisplay(0);
                matrix.shutdown(0, false);
                matrix.setIntensity(0, indication::BRIGHTNESS);
            }

            inline static void display(const byte symbol[])
            {
                for (uint8_t i = 0; i < 8; ++i)
                {
                    matrix.setRow(0, i, symbol[i]);
                }
            }

            static void blackout()
            {
                matrix.clearDisplay(0);
            }
    };

} // of indication 

void setup()
{
  indication::Matrix::setup();
}

void loop()
{
  indication::Matrix::display(indication::symbols::CUBE);
}
