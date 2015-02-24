#pragma once

#include "FreeRTOS.h"
#include "semphr.h"

#include <mutex>

// the predefined mutexts that have been created and maybe used by this lock
extern SemaphoreHandle_t READY_Q_MUTEX;
extern SemaphoreHandle_t TEMPERATURE_MUTEX;

// locks a global mutex and releases when out of scope
class Lock
{
public:
	Lock(SemaphoreHandle_t xSemaphore) : xSemaphore(xSemaphore){}
	void lock()
	{
		xSemaphoreTake(xSemaphore, portMAX_DELAY);
	}
	void unlock()
	{
		xSemaphoreGive( xSemaphore );
	};

private:
	SemaphoreHandle_t xSemaphore;
};

// just an alias for the std::lock_guard RAII-style lock
using AutoLock = std::lock_guard<Lock>;
