#pragma once

#include <mutexType.h>

namespace DFHM {

class patient_lock_guard {
public:
	patient_lock_guard(MutexType& mutex);
	~patient_lock_guard();
	void delayedLock();

private:
	bool _locked = false;
	MutexType& _mutex;
};


inline patient_lock_guard::patient_lock_guard(MutexType& mutex)
	: _mutex(mutex)
{
	_mutex.lock();
	_locked = true;
}

inline patient_lock_guard::~patient_lock_guard()
{
	if (_locked)
		_mutex.unlock();
}

void inline patient_lock_guard::delayedLock()
{
	_mutex.lock();
	_locked = true;
}

}
