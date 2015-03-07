#pragma once

#include "FreeRTOS.h"
#include "timers.h"

#include <tuple>
#include <string>

class LCDBase;

class StatusScreen
{
public:
	StatusScreen(LCDBase& lcd) : lcd(lcd) {};
	~StatusScreen() {};
	void init();

private:
	static void statusTimerCallback( TimerHandle_t pxTimer );
	std::tuple<float, float, float, float> getPosition();
	std::string getTemps();
	void update();
	LCDBase& lcd;
	TimerHandle_t status_timer_handle{0};
};
