
#include <list>
#include <multiLockGuard.h>
#include <Index3D.h>

using namespace std;
using namespace DFHM;
using namespace MultiLock;

#define WRITE_RESULTS 1

class MultiLock::WaitQueue
{
public:
	enum LockState {
		LOCK_WAITING,
		LOCK_ATTEMPT_IN_PROGRESS,
		LOCK_REJECTED,
		LOCK_COMPLETE,
	};

	struct GuardPtrRec {
		GuardPtrRec(lock_guard_multi* ptr)
			: _ptr(ptr)
		{}
		operator lock_guard_multi* ()
		{
			return _ptr;
		}
		const bool operator == (const GuardPtrRec& rhs) const
		{
			return _ptr == rhs._ptr;
		}
		const bool operator == (const lock_guard_multi* rhs) const
		{
			return this->_ptr == rhs;
		}

		LockState _lockState = LOCK_WAITING;
		lock_guard_multi* _ptr;
	};

	WaitQueue();
	~WaitQueue();

	shared_ptr<GuardPtrRec> add(lock_guard_multi* p);
	bool inQueue(const lock_guard_multi* p) const;
	void freedLocks(bool lockReleased);

private:
	shared_ptr<GuardPtrRec> _pendingRequest;

	void threadFunc();
	static void threadFuncStat(WaitQueue* self);
	void processAddQueue();
	void processWaitQueue();
	void removeNTS(const shared_ptr<GuardPtrRec>& pVal);

	bool _running = true, _stopped = false, _paused = false;
	size_t _runningThreads = 0, _numFailedWaitQueuePasses = 0;
	mutable MutexType _addQueueMutex;
	vector<shared_ptr<GuardPtrRec>> _waitQueue, _addQueue; // must be vectors so they are processed in FIFO order

	thread* _pThread;
};

namespace
{
	shared_ptr<WaitQueue> g_pWaitQueue = make_shared<WaitQueue>();
#if WRITE_RESULTS
	mutex g_CoutMutex;
#endif
}

/*********************************************************************************/
/*********************************************************************************/
/*********************************************************************************/

void MutexType::lock() {
	_mutex.lock();
	auto thisThreadId = this_thread::get_id();
	if (_numLocks == 0)
		_ownerThreadId = thisThreadId;
	else
		assert(_ownerThreadId == thisThreadId);
	_numLocks++;
}

bool MutexType::try_lock() {
	if (_mutex.try_lock()) {
		auto thisThreadId = this_thread::get_id();
		if (_numLocks == 0)
			_ownerThreadId = thisThreadId;
		else
			assert(_ownerThreadId == thisThreadId);
		_numLocks++;
		return true;
	}
	return false;
}

void MutexType::unlock() {
	if (!ownedByCorrectThread()) {
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

bool MutexType::isLocked() const
{
	bool enoughLocks = _numLocks > 0;
	if (!enoughLocks) {
		assert(!"wrong number of mutex locks");
	}

	if (!ownedByCorrectThread()) {
		assert(!"mutex owned by another thread");
	}
	return enoughLocks && ownedByCorrectThread();
}

bool MutexType::ownedByCorrectThread() const
{
	return (_numLocks == 0) || (std::this_thread::get_id() == _ownerThreadId);
}

lockable_object::lockable_object()
{

}
lockable_object::lockable_object(const lockable_object& src)
{

}

lockable_object& lockable_object::operator = (const lockable_object& rhs)
{
	return *this;
}

lock_guard_multi::lock_guard_multi(lockable_object* pLockable)
	: _pLockable(pLockable)
{
#if WRITE_RESULTS
	{
		std::lock_guard g(g_CoutMutex);
		cout << "Attempting to lock block: " << _pLockable->getBlockIdx() << "\n";
	}
#endif
	waitForAllLocks(pLockable);
#if WRITE_RESULTS
	{
		std::lock_guard g(g_CoutMutex);
		cout << "Locked block: " << _pLockable->getBlockIdx() << "\n";
	}
#endif
}

lock_guard_multi::lock_guard_multi(const lockable_object& lockable)
	: _pLockable(&lockable)
{
#if WRITE_RESULTS
	{
		std::lock_guard g(g_CoutMutex);
		auto idx = _pLockable->getBlockIdx();
		cout << "Attempting to lock polyhedron in block: " << idx << "\n";
		if (idx == Index3D(2, 1, 0)) {
			int dbgIdx = 1;
		}
	}
#endif
	waitForAllLocks(&lockable);
#if WRITE_RESULTS
	{
		std::lock_guard g(g_CoutMutex);
		cout << "Locked polyhedron in block: " << _pLockable->getBlockIdx() << "\n";
	}
#endif
}

lock_guard_multi::~lock_guard_multi()
{
	if (!_isLocked)
		return;
	unlockStack(_requestedLocks);
	g_pWaitQueue->freedLocks(true);
#if WRITE_RESULTS
	{
		std::lock_guard g(g_CoutMutex);
		cout << "Unlocked block: " << _pLockable->getBlockIdx() << "\n";
	}
#endif
}

bool lock_guard_multi::attemptLock()
{
	if (tryToLockAll()) {
		this_thread::yield();
		return true;
	}
	unlockStack(_lockStack);
	_lockStack.clear();
	g_pWaitQueue->freedLocks(false);

	this_thread::yield();
	_numTries++;
	if (_numTries > 10) {
		int dbgBreak = 1;
	}
	return false;
}

void lock_guard_multi::waitForAllLocks(const lockable_object* pLockable)
{
	set<lockable_object*> lockableSet;
	pLockable->getRequredLockableObjects(lockableSet);
	_requestedLocks.insert(_requestedLocks.end(), lockableSet.begin(), lockableSet.end());

	// This takes _waitQueueMutex and will release it when we have all the locks
	auto pGuardRec = g_pWaitQueue->add(this); // This adds to the pending list but does not set the lock

	// MUST wait in our thread, not the wait queue thread.
	this_thread::yield();
	while (!g_pWaitQueue->inQueue(this))
		this_thread::yield();

	// Wait queue now holds our lock and we wait for it to be released
	while (g_pWaitQueue->inQueue(this)) {
		assert(pGuardRec->_ptr == this);
		lock_guard g(pGuardRec->_ptr->getMutex()); // we hold our mutex now in our thread
		if (pGuardRec->_lockState == WaitQueue::LOCK_ATTEMPT_IN_PROGRESS) {
			if (attemptLock()) {
				pGuardRec->_lockState = WaitQueue::LOCK_COMPLETE;
				return;
			}
			pGuardRec->_lockState = WaitQueue::LOCK_REJECTED;
			this_thread::sleep_for(chrono::microseconds(500));
		}
	}

}

bool lock_guard_multi::tryToLockAll()
{
	_isLocked = true;

	_lockStack.clear();
	for (auto p : _requestedLocks) {
		auto& mutex = p->getMutex();
		if (mutex.try_lock()) {
			_lockStack.push_back(p);
		}
		else {
			_isLocked = false;
			break;
		}
	}
	return _isLocked;
}

void lock_guard_multi::unlockStack(std::vector<lockable_object*>& locks)
{
	for (auto iter = locks.rbegin(); iter != locks.rend(); iter++) {
		auto p = *iter;
		p->getMutex().unlock();
	}
}

WaitQueue::WaitQueue()
{
	_pThread = new thread(&threadFuncStat, this);
}

WaitQueue::~WaitQueue() 
{
	_running = false;

	// wait for the thread to exit
	while (!_stopped) {
		this_thread::sleep_for(chrono::milliseconds(10));
	}
}

shared_ptr<WaitQueue::GuardPtrRec> WaitQueue::add(lock_guard_multi* p)
{
	lock_guard g(_addQueueMutex);
	auto pGuard = make_shared<GuardPtrRec>(p);
	_addQueue.push_back(pGuard);
	_paused = false;
	_numFailedWaitQueuePasses = 0;

	return pGuard;
}

bool WaitQueue::inQueue(const lock_guard_multi* p) const
{
	lock_guard g(_addQueueMutex);
	for (const auto& pRec : _addQueue) {
		if (pRec->_ptr == p)
			return false;
	}
	return true;
}

void WaitQueue::freedLocks(bool lockReleased)
{
	lock_guard g(_addQueueMutex);
	if (lockReleased)
		_runningThreads--;
	_paused = false;
	_numFailedWaitQueuePasses = 0;
}

void WaitQueue::removeNTS(const shared_ptr<GuardPtrRec>& pVal)
{
	for (size_t i = 0; i < _waitQueue.size(); i++) {
		if (_waitQueue[i] == pVal) {
			_waitQueue.erase(_waitQueue.begin() + i);
			break;
		}
	}
}

void WaitQueue::processAddQueue()
{
	lock_guard g(_addQueueMutex);

	for (const auto& pRec : _addQueue) {
		pRec->_ptr->getMutex().lock();
		_waitQueue.push_back(pRec);
	}
	_addQueue.clear();
}

void WaitQueue::processWaitQueue()
{
	// We can only process one request at a time
	if (_pendingRequest) {
		if (_pendingRequest->_lockState == LOCK_COMPLETE) {
			removeNTS(_pendingRequest);
			_pendingRequest = nullptr;
			lock_guard g(_addQueueMutex);
			_runningThreads++;
		} else if (_pendingRequest->_lockState == LOCK_REJECTED) {
			_pendingRequest->_ptr->getMutex().lock();
			_pendingRequest = nullptr;
		} else if (_pendingRequest->_lockState = LOCK_ATTEMPT_IN_PROGRESS) {
			this_thread::yield();
			return;
		}
	}

	// We have a pending request, so don't start another one
	assert(!_pendingRequest);

	for (size_t i = 0; i < _waitQueue.size(); i++) {
		auto pRec = _waitQueue[i];
		if (pRec->_lockState == LOCK_REJECTED)
			continue;
		if (pRec->_lockState == LOCK_WAITING) {
			_pendingRequest = pRec;
			_pendingRequest->_lockState = LOCK_ATTEMPT_IN_PROGRESS;
			_pendingRequest->_ptr->getMutex().unlock();
			this_thread::yield();
			return;
		}
	}
	if (!_pendingRequest && !_waitQueue.empty()) {
		for (size_t i = 0; i < _waitQueue.size(); i++) {
			_waitQueue[i]->_lockState = LOCK_WAITING;
		}
		// This will be cleared when a lock_guard_multi's destructor is called
		lock_guard g(_addQueueMutex);
		_numFailedWaitQueuePasses++;
		if (_numFailedWaitQueuePasses > 3)
			_paused = true;
	}
}

void WaitQueue::threadFunc()
{
	while (_running) {
		if (_paused) {
			this_thread::sleep_for(chrono::milliseconds(20));
			continue;
		}

		processAddQueue();
		processWaitQueue();

		bool isEmpty;
		{
			lock_guard g(_addQueueMutex);
			isEmpty = _waitQueue.empty() && _addQueue.empty();
		}
		if (isEmpty)
			this_thread::sleep_for(chrono::milliseconds(5));
	}
	_stopped = true;
}

void WaitQueue::threadFuncStat(WaitQueue* self)
{
	self->threadFunc();
}

