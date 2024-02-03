#pragma once

#include <mutex>

namespace DFHM {

using MutexType = std::recursive_timed_mutex;

class patient_lock_guard {
public:
	patient_lock_guard(MutexType& mutex);
	~patient_lock_guard();

private:
	bool _locked = false;
	MutexType& _mutex;
};


inline patient_lock_guard::patient_lock_guard(MutexType& mutex)
	: _mutex(mutex)
{
#if 0
	_mutex.lock();
#else
	using namespace std::chrono_literals;
	_locked = _mutex.try_lock_for(500ms);
	if (!_locked) {
		assert(!"request for lock timed out");
	}
#endif
}

inline patient_lock_guard::~patient_lock_guard()
{
#if 0
	_mutex.unlock();
#else
	if (_locked)
		_mutex.unlock();
#endif
}

}
