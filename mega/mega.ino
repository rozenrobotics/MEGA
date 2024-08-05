//#include <lib/Max72xxPanel.h>
//#include <Adafruit_GFX.h>

typedef const uint8_t pin;


pin MATRIX_CS_PIN = 53;

namespace matrix
{	
	typedef const byte symbol;

	symbol A[8] = {
		0b01100110,
		0b01100110,
		0b01111110,
		0b01111110,
		0b01100110,
		0b01100110,
		0b01111110,
		0b00111100,
	};
	symbol M[8] = {
		0b11000011,
		0b11000011,
		0b11000011,
		0b11011011,
		0b11111111,
		0b11111111,
		0b11100111,
		0b11000011,
	};
    symbol N[8] = {
		0b11000011,
		0b11000111,
		0b11001111,
		0b11011111,
		0b11111011,
		0b11110011,
		0b11100011,
		0b11000011,
	};

	static const uint8_t BRIGHTNESS = 2;
	static const uint8_t ROTATION   = 2;

	Max72xxPanel
	m72xxp = Max72xxPanel(MATRIX_CS_PIN, 1, 1);
	m72xxp.setIntensity(matrix::BRIGHTNESS);
	m72xxp.setROtation(matrix::ROTATION);

	class Matrix
	{
	public:
		static void display(symbol symbol[])
		{
			Matrix::blackout();

			for (int i = 0; i < 8; ++i)
				for (int j = 0; j < 8; ++j)
					matrix::m72xxp.drawPixel(j, i, symbol[i] & (1 << j));
			
			Matrix::update();
		}

		static void blackout()
		{
			matrix::m72xxp.fillScreen(LOW);

			Matrix::update();
		}

	private:
		void update()
		{
			matrix::m72xxp.write();
		}

	};
} // matrix end


void setup()
{

}

void loop()
{

}
