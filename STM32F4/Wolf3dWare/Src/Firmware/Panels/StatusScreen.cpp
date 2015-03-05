#include "StatusScreen.h"
#include "LCDBase.h"

#include "../Kernel.h"
#include "../MotionControl.h"
#include "../Actuator.h"


void StatusScreen::init()
{
	lcd.init();
	// setup timer
	if(status_timer_handle == nullptr) {
		status_timer_handle= xTimerCreate("StatusTimer", pdMS_TO_TICKS(1000/1), pdTRUE, (void*)this, statusTimerCallback);
		xTimerStart(status_timer_handle, 10);
	}
}

// static required so we can marshall the method for the callback
void StatusScreen::statusTimerCallback( TimerHandle_t pxTimer )
{
	StatusScreen *ss = (StatusScreen*)pvTimerGetTimerID( pxTimer );
	ss->update();
}

std::tuple<float, float, float, float> StatusScreen::getPosition()
{
	float x, y, z, e;
	x= THEKERNEL.getMotionControl().getActuator('X').getCurrentPositionInmm();
	y= THEKERNEL.getMotionControl().getActuator('Y').getCurrentPositionInmm();
	z= THEKERNEL.getMotionControl().getActuator('Z').getCurrentPositionInmm();
	e= THEKERNEL.getMotionControl().getActuator('E').getCurrentPositionInmm();
	return std::make_tuple(x, y, z, e);
}

std::tuple<float, float> StatusScreen::getTemps()
{
	// TODO how to get temps as module may not be there
	return std::make_tuple(0.0F, 0.0F);
}

// runs every 1 second and updates the status screen
void StatusScreen::update()
{
	lcd.clear();

	auto pos= getPosition();
	lcd.setCursor(0, 0);
	lcd.printf("X%5.1f Y%5.1f Z%5.1f", std::get<0>(pos), std::get<1>(pos), std::get<2>(pos));
	lcd.setCursor(0, 1);
	lcd.printf("E %5.2f", std::get<3>(pos));

	auto temps= getTemps();
	lcd.setCursor(0, 2);
	lcd.printf("T:%4.1f/%4.1f", std::get<0>(temps), std::get<1>(temps));

	lcd.setCursor(0, 3);
	lcd.printf("%u", lcd.readEncoderPosition());
}
