#pragma once

#include <mutexType.h>

namespace DFHM {

class patient_lock_guard {
public:
	patient_lock_guard(MutexType& mutex, std::thread::id creatorId);
	~patient_lock_guard();

private:
	bool _locked = false;
	std::thread::id _creatorId;
	MutexType& _mutex;
};


inline patient_lock_guard::patient_lock_guard(MutexType& mutex, std::thread::id creatorId)
	: _mutex(mutex)
	, _creatorId(creatorId)
{
	_locked = _mutex.lock(_creatorId);
}

inline patient_lock_guard::~patient_lock_guard()
{
	if (_locked)
		_mutex.unlock(_creatorId);
}

}
