#pragma once

#include "FreeRTOS.h"
#include "semphr.h"

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
	void unLock()
	{
		xSemaphoreGive( xSemaphore );
	};

private:
	SemaphoreHandle_t xSemaphore;
};

class AutoLock
{
public:
	AutoLock(Lock& l) : lock(l) { l.lock(); }
	~AutoLock() { lock.unLock(); }
private:
	Lock& lock;
};
