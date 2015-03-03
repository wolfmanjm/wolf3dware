/*
Much of this was copied from LiquidTWI2
  LiquidTWI2 High Performance i2c LCD driver for MCP23008 & MCP23017
  hacked by Sam C. Lin / http://www.lincomatic.com
  from
   LiquidTWI by Matt Falcon (FalconFour) / http://falconfour.com
   logic gleaned from Adafruit RGB LCD Shield library
*/

#pragma once

#include "LCDBase.h"

#include <stdint.h>
#include <functional>

class Viki : public LCDBase
{
public:
	Viki();
	virtual ~Viki();
	void home();
	void clear();
	void display();
	void setCursor(uint8_t col, uint8_t row);
	void init();
	void write(const char *line, int len);
	void on_refresh();

	// added viki commands
	void setLed(int led, bool onoff);

	uint8_t readButtons();
	int readEncoderDelta();

	// this is the number of clicks per detent
	int getEncoderResolution()
	{
		return 2;
	}

	void set_variant(int n)
	{
		isPanelolu2 = (n == 1);
	}

	void buzz(long, uint16_t);
	void noCursor();
	void cursor();
	void noBlink();
	void blink();
	void scrollDisplayLeft();
	void scrollDisplayRight();
	void leftToRight();
	void rightToLeft();
	void autoscroll();
	void noAutoscroll();
	void noDisplay();

	enum HAL_FUNCTION_INDEX {
		INIT_I2C,
		WRITE_I2C,
		READ_I2C,
		READ_ENCODER,
		READ_BUTTONS,

		N_HAL_FUNCTIONS
	};
	using HAL_function_t = std::function<uint8_t(uint8_t, uint8_t*, uint16_t)>;
	void assignHALFunction(HAL_FUNCTION_INDEX i, HAL_function_t fnc)
	{
		hal_functions[i] = fnc;
	}

private:
	void send(uint8_t, uint8_t);
	void command(uint8_t value);

	void burstBits16(uint16_t);
	void burstBits8b(uint8_t);

	HAL_function_t hal_functions[N_HAL_FUNCTIONS];

	uint16_t _backlightBits; // only for MCP23017
	char displaymode;
	char displayfunction;
	char displaycontrol;
	uint8_t _numlines;
	uint8_t _currline;
	uint8_t i2c_address;
	bool isPanelolu2;
};
