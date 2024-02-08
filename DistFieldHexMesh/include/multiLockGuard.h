#pragma once

#include <thread>
#include <mutex>
#include <set>
#include <vector>
#include <Index3D.h>

namespace MultiLock {

class MutexType {
public:
    MutexType() = default;
    MutexType(const MutexType& src) = delete;
    MutexType& operator = (const MutexType& rhs) = delete;

	void lock();
	bool try_lock();
	void unlock();
	bool isLocked() const;
	bool ownedByCorrectThread() const;

private:
    std::thread::id _ownerThreadId;
    size_t _numLocks = 0;
    std::recursive_timed_mutex _mutex;
};

class lockable_object {
public:
	lockable_object();
	lockable_object(const lockable_object& src);
	lockable_object& operator = (const lockable_object& rhs);

	virtual const DFHM::Index3D& getBlockIdx() const = 0;
	virtual MultiLock::MutexType& getMutex() const = 0;
	virtual bool isMutexLocked() const = 0;

	// Include (this) in allRequiredLockables
	virtual void getRequredLockableObjects(std::set<MultiLock::lockable_object*>& allRequiredLockables) const = 0;
};

class WaitQueue;

class lock_guard {
public:
	inline lock_guard(MutexType& mut) 
		: _mutex(mut)
	{
		_mutex.lock();
		_locked = true;
	}

	inline ~lock_guard()
	{
		if (_locked)
			_mutex.unlock();
	}

	void lock() {
		if (!_locked) {
			_mutex.lock();
			_locked = true;
		}
	}
	void unlock() {
		if (_locked) {
			_mutex.unlock();
			_locked = false;
		}
	}

private:
	MutexType& _mutex;

	bool _locked = false;
};

class lock_guard_multi
{
// This operates on the principal that it doesn't matter which thread locks, holds and releases the mutex as long as your lock OWNS all the 
// mutexes in the locking thread
// It also processes all the lock requests in the order they were recieved. This means that a request for a lot of locks will get handled vs
// lots of requests for a single lock.
public:
	lock_guard_multi(lockable_object* pLockable);
	lock_guard_multi(const lockable_object& lockable);
	~lock_guard_multi();

	MutexType& getMutex() const;
	bool attemptLock();
private:
	void waitForAllLocks(const lockable_object* pLockable);
	bool tryToLockAll();
	void unlockStack(std::vector<lockable_object*>& locks);

	mutable MutexType _mutex; // This set by us and cleared by the wait queue

	const lockable_object* _pLockable;
	size_t _numTries = 0;
	std::thread::id _holdingThreadId;
	bool _isLocked = false;
	std::vector<lockable_object*> _requestedLocks;
	std::vector<lockable_object*> _lockStack;
};

inline MutexType& lock_guard_multi::getMutex() const
{
	return _mutex;
}


}