#pragma once

#include <chrono>
#include <mutex>

namespace DFHM {

class MutexType {
public:
    template <class _Rep, class _Period>
    inline _NODISCARD_TRY_CHANGE_STATE bool try_lock_for(const std::chrono::duration<_Rep, _Period>& _Rel_time) {
        if (_mutex.try_lock_for(_Rel_time)) {
            _numLocks++;
            return true;
        }
        return false;
    }

    inline void unlock() {
        assert(_numLocks > 0);
        _mutex.unlock();
        _numLocks--;
    }

    inline bool isLocked() const
    {
        return _numLocks > 0;
    }

private:
    size_t _numLocks = 0;
	std::recursive_timed_mutex _mutex;
};

}