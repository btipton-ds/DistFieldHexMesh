#pragma once

#include <thread>
#include <set>
#include <vector>

namespace DFHM {

class Index3D;
class Block;
class Polyhedron;

class MultiLockGuard
{
public:
	MultiLockGuard(const Block* pBlock, std::thread::id creatorId = std::this_thread::get_id());
	MultiLockGuard(const Polyhedron& cell, std::thread::id creatorId = std::this_thread::get_id());
	~MultiLockGuard();
private:
	void create(std::set<Index3D>& indexSet, std::thread::id creatorId);
	bool tryToLockAll(std::vector<const Block*>& lockStack);
	void unlockStack(const std::vector<const Block*>& lockStack);

	std::thread::id _creatorId;
	bool _isLocked = false;
	const Block* _pBlock;
	std::vector<const Block*> _lockedBlocks;
};

}