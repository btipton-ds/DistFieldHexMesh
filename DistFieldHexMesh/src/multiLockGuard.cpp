
#include <mutex>
#include <list>
#include <multiLockGuard.h>
#include <Index3D.h>
#include <block.h>

using namespace std;
using namespace DFHM;

namespace
{
	mutex g_CoutMutex;
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
	set<Index3DId> adjCellSet = cell.getAdjacentCells();
	adjCellSet.insert(cell.getId());

	set<Index3D> adjBlockSet;
	for (const auto& cellId : adjCellSet) {
		adjBlockSet.insert(cellId.blockIdx());
	}
	create(adjBlockSet, creatorId);
}

void MultiLockGuard::create(set<Index3D>& indexSet, thread::id creatorId)
{
	bool inWaitQueue = false;
	for (const auto& idx : indexSet)
		_lockedBlocks.push_back(_pBlock->getOwner(idx));

	{
		lock_guard g(g_CoutMutex);
		cout << "Attempting to lock block: " << _pBlock->getBlockIdx() << "\n";
	}

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
				this_thread::sleep_for(chrono::milliseconds(50));
			}
		}
		if (inWaitQueue)
			this_thread::sleep_for(chrono::milliseconds(100));
	}
	// Lock ascending
	if (_isLocked) {
		lock_guard g(g_CoutMutex);
		cout << "Locked block: " << _pBlock->getBlockIdx() << "\n";
	}
	else
		assert(!"request for lock timed out");
}

MultiLockGuard::~MultiLockGuard()
{
	if (!_isLocked)
		return;
	unlockStack(_lockedBlocks);
	lock_guard g(g_CoutMutex);
	cout << "Unlocked block: " << _pBlock->getBlockIdx() << "\n";
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
