#include "Viki.h"

#include "../Kernel.h"

// if this is defined we use the R/W poll mode instead of fixed delays
// However at the slower I2C frequency required for Viki long cables it is slower than fixed delay
// taken from LiquidCrystalFast.cpp and implemented for Viki LCD here by Jim Morris
//#define USE_FASTMODE

// readButtons() will only return these bit values
#define VIKI_ALL_BUTTON_BITS (BUTTON_PAUSE|BUTTON_UP|BUTTON_DOWN|BUTTON_LEFT|BUTTON_RIGHT|BUTTON_SELECT)

// 0x20<<1
#define MCP23017_ADDRESS 0x40

// registers
#define MCP23017_IODIRA 0x00
#define MCP23017_IPOLA 0x02
#define MCP23017_GPINTENA 0x04
#define MCP23017_DEFVALA 0x06
#define MCP23017_INTCONA 0x08
#define MCP23017_IOCONA 0x0A
#define MCP23017_GPPUA 0x0C
#define MCP23017_INTFA 0x0E
#define MCP23017_INTCAPA 0x10
#define MCP23017_GPIOA 0x12
#define MCP23017_OLATA 0x14


#define MCP23017_IODIRB 0x01
#define MCP23017_IPOLB 0x03
#define MCP23017_GPINTENB 0x05
#define MCP23017_DEFVALB 0x07
#define MCP23017_INTCONB 0x09
#define MCP23017_IOCONB 0x0B
#define MCP23017_GPPUB 0x0D
#define MCP23017_INTFB 0x0F
#define MCP23017_INTCAPB 0x11
#define MCP23017_GPIOB 0x13
#define MCP23017_OLATB 0x15

//MCP23017 - Adafruit RGB LCD Shield and Viki
// bit pattern for the burstBits function is
//
//  B7 B6 B5 B4 B3 B2 B1 B0 A7 A6 A5 A4 A3 A2 A1 A0 - MCP23017
//  RS RW EN D4 D5 D6 D7 LB LG LR BZ B4 B3 B2 B1 B0
//  15 14 13 12 11 10 9  8  7  6  5  4  3  2  1  0
#define M17_BIT_RS 0x8000
#define M17_BIT_RW 0x4000
#define M17_BIT_EN 0x2000
#define M17_BIT_D4 0x1000
#define M17_BIT_D5 0x0800
#define M17_BIT_D6 0x0400
#define M17_BIT_D7 0x0200
#define M17_BIT_LB 0x0100
#define M17_BIT_LG 0x0080
#define M17_BIT_LR 0x0040
#define M17_BIT_BZ 0x0020 //Added a buzzer on this pin
#define M17_BIT_B4 0x0010
#define M17_BIT_B3 0x0008
#define M17_BIT_B2 0x0004
#define M17_BIT_B1 0x0002
#define M17_BIT_B0 0x0001

// commands
#define LCD_CLEARDISPLAY 0x01
#define LCD_RETURNHOME 0x02
#define LCD_ENTRYMODESET 0x04
#define LCD_DISPLAYCONTROL 0x08
#define LCD_CURSORSHIFT 0x10
#define LCD_FUNCTIONSET 0x20
#define LCD_SETCGRAMADDR 0x40
#define LCD_SETDDRAMADDR 0x80

// flags for display entry mode
#define LCD_ENTRYRIGHT 0x00
#define LCD_ENTRYLEFT 0x02
#define LCD_ENTRYSHIFTINCREMENT 0x01
#define LCD_ENTRYSHIFTDECREMENT 0x00

// flags for display on/off control
#define LCD_DISPLAYON 0x04
#define LCD_DISPLAYOFF 0x00
#define LCD_CURSORON 0x02
#define LCD_CURSOROFF 0x00
#define LCD_BLINKON 0x01
#define LCD_BLINKOFF 0x00

// flags for display/cursor shift
#define LCD_DISPLAYMOVE 0x08
#define LCD_CURSORMOVE 0x00
#define LCD_MOVERIGHT 0x04
#define LCD_MOVELEFT 0x00

// flags for function set
#define LCD_8BITMODE 0x10
#define LCD_4BITMODE 0x00
#define LCD_2LINE 0x08
#define LCD_1LINE 0x00
#define LCD_5x10DOTS 0x04
#define LCD_5x8DOTS 0x00

// flags for backlight control
#define LCD_BACKLIGHT 0x08
#define LCD_NOBACKLIGHT 0x00

#define wait_ms(n) THEKERNEL.delay(n)

Viki::Viki()
{
	// Default values
	i2c_address      = MCP23017_ADDRESS;
	displaycontrol   = 0x00;
	displayfunction  = LCD_4BITMODE | LCD_2LINE | LCD_5x8DOTS; // in case they forget to call begin() at least we have somethin
	displaymode      = 0x00;
	_numlines        = 4;
}

Viki::~Viki() { }

void Viki::init()
{
	// SEE PAGE 45/46 FOR INITIALIZATION SPECIFICATION!
	// according to datasheet, we need at least 40ms after power rises above 2.7V
	// before sending commands. Arduino can turn on way befer 4.5V so we'll wait 50

	uint8_t data[2];

	hal_functions[INIT_I2C](0, nullptr, 0);

	// Setup
	displayfunction = LCD_4BITMODE | LCD_2LINE | LCD_5x8DOTS;
	_backlightBits = M17_BIT_LB | M17_BIT_LG | M17_BIT_LR; // all off

	wait_ms(50);

	// now set up input/output pins in MCP23017
	data[0] = MCP23017_IODIRA;
	data[1] = 0x1F; // buttons input, all others output
	hal_functions[WRITE_I2C](i2c_address, data, 2);


	// set the button pullups
	data[0] = MCP23017_GPPUA;
	data[1] = 0x1F;
	hal_functions[WRITE_I2C](i2c_address, data, 2);

	data[0] = MCP23017_IODIRB;
	data[1] = 0x00; // all pins output
	hal_functions[WRITE_I2C](i2c_address, data, 2);

	//put the LCD into 4 bit mode
	// start with a non-standard command to make it realize we're speaking 4-bit here
	// per LCD datasheet, first command is a single 4-bit burst, 0011.
	//-----
	//  we cannot assume that the LCD panel is powered at the same time as
	//  the arduino, so we have to perform a software reset as per page 45
	//  of the HD44780 datasheet - (kch)
	//-----

	// bit pattern for the burstBits function is
	//
	//  B7 B6 B5 B4 B3 B2 B1 B0 A7 A6 A5 A4 A3 A2 A1 A0 - MCP23017
	//  15 14 13 12 11 10 9  8  7  6  5  4  3  2  1  0
	//  RS RW EN D4 D5 D6 D7 B  G  R     B4 B3 B2 B1 B0
	for (uint8_t i = 0; i < 3; i++) {
		burstBits8b((M17_BIT_EN | M17_BIT_D5 | M17_BIT_D4) >> 8);
		burstBits8b((M17_BIT_D5 | M17_BIT_D4) >> 8);
	}
	burstBits8b((M17_BIT_EN | M17_BIT_D5) >> 8);
	burstBits8b(M17_BIT_D5 >> 8);


	wait_ms(5); // this shouldn't be necessary, but sometimes 16MHz is stupid-fast.

	command(LCD_FUNCTIONSET | displayfunction); // then send 0010NF00 (N=lines, F=font)
	wait_ms(5); // for safe keeping...
	command(LCD_FUNCTIONSET | displayfunction); // ... twice.
	wait_ms(5); // done!

	// turn on the LCD with our defaults. since these libs seem to use personal preference, I like a cursor.
	displaycontrol = (LCD_DISPLAYON | LCD_BACKLIGHT);
	display();
	// clear it off
	clear();

	displaymode = LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT;
	// set the entry mode
	command(LCD_ENTRYMODESET | displaymode);
}

// we use this to burst bits to the GPIO chip whenever we need to. avoids repetitive code.
void Viki::burstBits8b(uint8_t value)
{
	uint8_t data[2];
	data[0] = MCP23017_GPIOB;
	data[1] = value;
	hal_functions[WRITE_I2C](i2c_address, data, 2);
}

// value byte order is BA
void Viki::burstBits16(uint16_t value)
{
	uint8_t data[3];
	data[0] = MCP23017_GPIOA;
	data[1] = value & 0xFF;
	data[2] = value >> 8;
	hal_functions[WRITE_I2C](i2c_address, data, 3);
}

// cycle the buzzer pin at a certain frequency (hz) for a certain duration (ms)
void Viki::buzz(long duration, uint16_t freq)
{
	uint8_t data[2];
	int currentRegister = 0;
	// read gpio register
	data[0] = MCP23017_GPIOA;
	hal_functions[WRITE_I2C](i2c_address, data, 1);
	hal_functions[READ_I2C](i2c_address, data, 1);       // Read from selected Register
	currentRegister = data[0];

	long period = 1000 / freq; // period in ms
	long elapsed_time = 0;
	while (elapsed_time < duration) {
		data[0] = MCP23017_GPIOA;
		data[1] = currentRegister |= M17_BIT_BZ;
		hal_functions[WRITE_I2C](i2c_address, data, 2);

		wait_ms(period / 2);

		data[0] = MCP23017_GPIOA;
		data[1] = currentRegister &= ~M17_BIT_BZ;
		hal_functions[WRITE_I2C](i2c_address, data, 2);

		wait_ms(period / 2);
		elapsed_time += (period);
	}
}

uint8_t Viki::readButtons(void)
{
	uint8_t data[2];
	data[0] = MCP23017_GPIOA;
	hal_functions[WRITE_I2C](i2c_address, data, 1);
	hal_functions[READ_I2C](i2c_address, data, 1);       // Read from selected Register

	// check the aux button
	if(hal_functions[READ_BUTTONS]) {
		uint8_t but;
		hal_functions[READ_BUTTONS](0, &but, 1);
		if(but & 1) data[0] |= BUTTON_PAUSE;
	}

	// if it is the variant Panelolu2 swap the buttons around
	if(isPanelolu2) {
		// the select button bit is on GPA2 not GPA0
		if((data[0]&M17_BIT_B2) == 0) return BUTTON_SELECT;
		return 0; // only one button on Panelolu2 ignore the ena_a and en_b

	} else {
		return (~data[0]) & VIKI_ALL_BUTTON_BITS;
	}
}

void Viki::on_refresh()
{
	// NOP
}

uint16_t Viki::readEncoderPosition()
{
	uint16_t counter;
	hal_functions[READ_ENCODER](0, (uint8_t*)&counter, sizeof(counter));
	return counter;
}

void Viki::clear()
{
	command(LCD_CLEARDISPLAY);  // clear display, set cursor position to zero
#ifndef USE_FASTMODE
	wait_ms(2);  // this command takes a long time!
#endif
}

void Viki::home()
{
	command(LCD_RETURNHOME);  // set cursor position to zero
#ifndef USE_FASTMODE
	wait_ms(2);  // this command takes a long time!
#endif
}

void Viki::setCursor(uint8_t col, uint8_t row)
{
	int row_offsets[] = { 0x00, 0x40, 0x14, 0x54 };
	if ( row > _numlines ) row = _numlines - 1;    // we count rows starting w/0
	command(LCD_SETDDRAMADDR | (col + row_offsets[row]));
}

// Turn the display on/off (quickly)
void Viki::noDisplay()
{
	displaycontrol &= ~LCD_DISPLAYON;
	command(LCD_DISPLAYCONTROL | displaycontrol);
}
void Viki::display()
{
	displaycontrol |= LCD_DISPLAYON;
	command(LCD_DISPLAYCONTROL | displaycontrol);
}

// Turns the underline cursor on/off
void Viki::noCursor()
{
	displaycontrol &= ~LCD_CURSORON;
	command(LCD_DISPLAYCONTROL | displaycontrol);
}
void Viki::cursor()
{
	displaycontrol |= LCD_CURSORON;
	command(LCD_DISPLAYCONTROL | displaycontrol);
}

// Turn on and off the blinking cursor
void Viki::noBlink()
{
	displaycontrol &= ~LCD_BLINKON;
	command(LCD_DISPLAYCONTROL | displaycontrol);
}
void Viki::blink()
{
	displaycontrol |= LCD_BLINKON;
	command(LCD_DISPLAYCONTROL | displaycontrol);
}

// These commands scroll the display without changing the RAM
void Viki::scrollDisplayLeft(void)
{
	command(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVELEFT);
}
void Viki::scrollDisplayRight(void)
{
	command(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVERIGHT);
}

// This is for text that flows Left to Right
void Viki::leftToRight(void)
{
	displaymode |= LCD_ENTRYLEFT;
	command(LCD_ENTRYMODESET | displaymode);
}

// This is for text that flows Right to Left
void Viki::rightToLeft(void)
{
	displaymode &= ~LCD_ENTRYLEFT;
	command(LCD_ENTRYMODESET | displaymode);
}

// This will 'right justify' text from the cursor
void Viki::autoscroll(void)
{
	displaymode |= LCD_ENTRYSHIFTINCREMENT;
	command(LCD_ENTRYMODESET | displaymode);
}

// This will 'left justify' text from the cursor
void Viki::noAutoscroll(void)
{
	displaymode &= ~LCD_ENTRYSHIFTINCREMENT;
	command(LCD_ENTRYMODESET | displaymode);
}

void Viki::command(uint8_t value)
{
	send(value, 0);
}

void Viki::write(const char *line, int len)
{
	for (int i = 0; i < len; ++i) {
		send(*line++, 1);
	}
}

// Sets the indicator leds
void Viki::setLed(int led, bool onoff)
{
	// LED turns on when bit is cleared
	if(onoff) {
		switch(led) {
			case LED_FAN_ON: _backlightBits &= ~M17_BIT_LR; break; // on
			case LED_HOTEND_ON: _backlightBits &= ~M17_BIT_LG; break; // on
			case LED_BED_ON: _backlightBits &= ~M17_BIT_LB; break; // on
		}
	} else {
		switch(led) {
			case LED_FAN_ON: _backlightBits |= M17_BIT_LR; break; // off
			case LED_HOTEND_ON: _backlightBits |= M17_BIT_LG; break; // off
			case LED_BED_ON: _backlightBits |= M17_BIT_LB; break; // off
		}
	}
	burstBits16(_backlightBits);
}

// write either command or data, burst it to the expander over I2C.
void Viki::send(uint8_t value, uint8_t mode)
{
#ifdef USE_FASTMODE
	// polls for ready. not sure on I2C this is any faster

	// set Data pins as input
	char data[2];
	data[0] = MCP23017_IODIRB;
	data[1] = 0x1E;
	hal_functions[WRITE_I2C](i2c_address, data, 2);
	uint8_t b = _backlightBits >> 8;
	burstBits8b((M17_BIT_RW >> 8) | b); // RW hi,RS lo
	char busy;
	data[0] = MCP23017_GPIOB;
	do {
		burstBits8b(((M17_BIT_RW | M17_BIT_EN) >> 8) | b); // EN hi
		hal_functions[WRITE_I2C](i2c_address, data, 1);
		hal_functions[READ_I2C](i2c_address, &busy, 1); // Read D7
		burstBits8b((M17_BIT_RW >> 8) | b); // EN lo
		burstBits8b(((M17_BIT_RW | M17_BIT_EN) >> 8) | b); // EN hi
		burstBits8b((M17_BIT_RW >> 8) | b); // EN lo
	} while ((busy & (M17_BIT_D7 >> 8)) != 0);

	// reset data bits as output
	data[0] = MCP23017_IODIRB;
	data[1] = 0x00;
	hal_functions[WRITE_I2C](i2c_address, data, 2);
	burstBits8b(b); // RW lo

#else
	//  wait_ms(1);
#endif

	// BURST SPEED, OH MY GOD
	// the (now High Speed!) I/O expander pinout
	//  B7 B6 B5 B4 B3 B2 B1 B0 A7 A6 A5 A4 A3 A2 A1 A0 - MCP23017
	//  15 14 13 12 11 10 9  8  7  6  5  4  3  2  1  0
	//  RS RW EN D4 D5 D6 D7 B  G  R     B4 B3 B2 B1 B0

	// n.b. RW bit stays LOW to write
	uint8_t buf = _backlightBits >> 8;
	// send high 4 bits
	if (value & 0x10) buf |= M17_BIT_D4 >> 8;
	if (value & 0x20) buf |= M17_BIT_D5 >> 8;
	if (value & 0x40) buf |= M17_BIT_D6 >> 8;
	if (value & 0x80) buf |= M17_BIT_D7 >> 8;

	if (mode) buf |= (M17_BIT_RS | M17_BIT_EN) >> 8; // RS+EN
	else buf |= M17_BIT_EN >> 8; // EN

	burstBits8b(buf);

	// resend w/ EN turned off
	buf &= ~(M17_BIT_EN >> 8);
	burstBits8b(buf);

	// send low 4 bits
	buf = _backlightBits >> 8;
	// send high 4 bits
	if (value & 0x01) buf |= M17_BIT_D4 >> 8;
	if (value & 0x02) buf |= M17_BIT_D5 >> 8;
	if (value & 0x04) buf |= M17_BIT_D6 >> 8;
	if (value & 0x08) buf |= M17_BIT_D7 >> 8;

	if (mode) buf |= (M17_BIT_RS | M17_BIT_EN) >> 8; // RS+EN
	else buf |= M17_BIT_EN >> 8; // EN

	burstBits8b(buf);

	// resend w/ EN turned off
	buf &= ~(M17_BIT_EN >> 8);
	burstBits8b(buf);
}
