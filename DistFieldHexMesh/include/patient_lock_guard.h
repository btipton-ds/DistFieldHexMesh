#pragma once

#include <mutexType.h>

namespace DFHM {

class patient_lock_guard {
public:
	patient_lock_guard(MutexType& mutex, bool lockImmediately = false);
	~patient_lock_guard();
	void delayedLock();

private:
	bool _locked = false;
	std::thread::id _creatorId;
	MutexType& _mutex;
};


inline patient_lock_guard::patient_lock_guard(MutexType& mutex, bool lockImmediately)
	: _mutex(mutex)
	, _creatorId(std::this_thread::get_id())
{
	if (lockImmediately) {
		_mutex.lock();
		_locked = true;
	}
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
