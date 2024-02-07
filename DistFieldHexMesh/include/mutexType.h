#pragma once

#include <chrono>
#include <mutex>

namespace DFHM {

class MutexType {
public:
    MutexType() = default;
    MutexType(const MutexType& src) = delete;
    MutexType& operator = (const MutexType& rhs) = delete;

    inline _NODISCARD_TRY_CHANGE_STATE bool lock(std::thread::id creatorId) {
        const auto time = std::chrono::microseconds(10);
        if (_mutex.try_lock_for(time)) {
            if (_numLocks == 0)
                _ownerThreadId = creatorId;
            else
                assert(_ownerThreadId == creatorId);
            _numLocks++;
            return true;
        }
        return false;
    }

    inline void unlock(std::thread::id creatorId) {
        bool ownedByRightThread = (creatorId == _ownerThreadId);
        if (!ownedByRightThread) {
            assert(!"Hey!! Why are you unlocking a mutex you don't hold?");
            return;
        }

        if (_numLocks > 0) {
            _numLocks--;
        }
        if (_numLocks == 0)
            _ownerThreadId = {};
        _mutex.unlock();
    }

    inline bool isLocked() const
    {
        bool enoughLocks = _numLocks > 0;
        if (!enoughLocks) {
            assert(!"wrong number of mutex locks");
        }
        bool ownedByRightThread = std::this_thread::get_id() == _ownerThreadId;
        if (!ownedByRightThread) {
            assert(!"mutex owned by another thread");
        }
        return enoughLocks && ownedByRightThread;
    }

private:
    std::thread::id _ownerThreadId;
    size_t _numLocks = 0;
	std::recursive_timed_mutex _mutex;
};

}