#pragma once

#include "mbed.h"
#include "rtos.h"

#include <mutex>

// MBED Mutex
using GenericMutex= Mutex;

extern GenericMutex READY_Q_MUTEX;
extern GenericMutex TEMPERATURE_MUTEX;

// wrapper around RTOS specific mutex
class Lock
{
	public:
		Lock(GenericMutex mutex) : the_mutex(mutex){}
		void lock()
		{
			the_mutex.lock();
		}
		void unlock()
		{
			the_mutex.unlock();
		}

	private:
		GenericMutex the_mutex;
};

// locks a global mutex and releases when out of scope
// just an alias for the std::lock_guard RAII-style lock
using AutoLock = std::lock_guard<Lock>;
