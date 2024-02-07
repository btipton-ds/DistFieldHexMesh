
#include <mutex>
#include <list>
#include <multiLockGuard.h>
#include <Index3D.h>
#include <block.h>

using namespace std;
using namespace DFHM;

#define WRITE_RESULTS 0

namespace
{
#if WRITE_RESULTS
	mutex g_CoutMutex;
#endif
	mutex g_requestMultiLockMutex;
	list<const Block*> g_waitQueue;
}

MultiLockGuard::MultiLockGuard(const Block* pBlock, thread::id creatorId)
	: _pBlock(pBlock)
	, _creatorId(creatorId)
{
	set<Index3D> indexSet;
	_pBlock->getAdjacentBlockIndices(indexSet);
	create(indexSet, creatorId);
}

MultiLockGuard::MultiLockGuard(const Polyhedron& cell, thread::id creatorId)
	: _pBlock(cell.getBlockPtr())
	, _creatorId(creatorId)
{
	set<Index3D> adjBlockSet = cell.getAdjacentBlockIndices_UNSAFE();

	create(adjBlockSet, creatorId);
}

void MultiLockGuard::create(set<Index3D>& indexSet, thread::id creatorId)
{
	bool inWaitQueue = false;
	for (const auto& idx : indexSet)
		_lockedBlocks.push_back(_pBlock->getOwner(idx));

#if WRITE_RESULTS
	{
		lock_guard g(g_CoutMutex);
		cout << "Attempting to lock block: " << _pBlock->getBlockIdx() << "\n";
	}
#endif

	/*
	* TODO There's a dead lock condition during attempting to lock so many mutexes.
	* In theory, as long as only one thread is requesting locks and does not block releasing of locks
	* It should not need the g_waitQueue or ordering of locks.
	* 
	* Unfortunately, it hasn't worked yet.
	*/
	{
		lock_guard mlg(g_requestMultiLockMutex);
		g_waitQueue.push_back(_pBlock);
		inWaitQueue = true;
	}
	while (inWaitQueue) {
		bool isFirst = false;
		{
			lock_guard mlg(g_requestMultiLockMutex);
			isFirst = g_waitQueue.front() == _pBlock;
		}
		// wait our turn
		if (isFirst) {
			while (!_isLocked) {
				vector<const Block*> lockedBlocks;
				if (tryToLockAll(lockedBlocks)) {

					lock_guard g(g_requestMultiLockMutex);
					g_waitQueue.pop_front();
					inWaitQueue = false;
					break;
				}
				else
					unlockStack(lockedBlocks);
			}
		}
	}
	// Lock ascending
	if (_isLocked) {
#if WRITE_RESULTS
		lock_guard g(g_CoutMutex);
		cout << "Locked block: " << _pBlock->getBlockIdx() << "\n";
#endif
	}
	else
		assert(!"request for lock timed out");
}

MultiLockGuard::~MultiLockGuard()
{
	if (!_isLocked)
		return;
	unlockStack(_lockedBlocks);
#if WRITE_RESULTS
	lock_guard g(g_CoutMutex);
	cout << "Unlocked block: " << _pBlock->getBlockIdx() << "\n";
#endif
}

bool MultiLockGuard::tryToLockAll(vector<const Block*>& lockedBlocks)
{
	lock_guard g(g_requestMultiLockMutex);
	_isLocked = true;
	for (auto iter = _lockedBlocks.begin(); iter != _lockedBlocks.end(); iter++) {
		auto p = *iter;
		auto& mutex = p->getMutex();
		if (mutex.lock(_creatorId)) {
			assert(mutex.isLocked());
			lockedBlocks.push_back(p);
		}
		else {
			_isLocked = false;
			break;
		}
	}
	return _isLocked;
}

void MultiLockGuard::unlockStack(const vector<const Block*>& lockedBlocks)
{
	for (auto iter = lockedBlocks.rbegin(); iter != lockedBlocks.rend(); iter++) {
		auto p = *iter;
		p->getMutex().unlock(_creatorId);
	}

}
