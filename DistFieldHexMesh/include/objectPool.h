#pragma once

#include <vector>
#include <tm_vector3.h>
#include <segmentedVector.h>
#include <objectPoolId.h>

namespace DFHM {

template<class T>
class ObjectPool {
public:
	ObjectPool() = default;

	size_t getNumThreads() const;
	void setNumThreads(size_t val);

	void testReset();

	void free(const ObjectPoolId& id); // Permanently delete it
	void unload(const ObjectPoolId& id); // Free the memory, but keep the id

	ObjectPoolId create(const ObjectPoolId& id);
	ObjectPoolId create(T*& pObj, const ObjectPoolId& id);
	ObjectPoolId getObj(const ObjectPoolId& id, T*& pObj, bool allocateIfNeeded);
	const T* getObj(const ObjectPoolId& id) const;
	T* getObj(const ObjectPoolId& id);

	size_t getNumAllocated(size_t threadIndex = -1) const;
	size_t getNumAvailable(size_t threadIndex = -1) const;
	size_t getNumAvailableIds(size_t threadIndex = -1) const;
	size_t getNumUnloaded(size_t threadIndex = -1) const;
private:

	struct ThreadLocalData {
		void free(size_t id); // Permanently delete it
		void unload(size_t id); // Free the memory, but keep the id

		size_t create(size_t id = -1);
		size_t create(T*& pObj, size_t id = -1);
		size_t getObj(size_t id, T*& pObj, bool allocateIfNeeded);
		const T* getObj(size_t id) const;
		T* getObj(size_t id);

		size_t getNumAllocated() const;
		size_t getNumAvailable() const;
		size_t getNumAvailableIds() const;
		size_t getNumUnloaded() const;

		std::vector<size_t> _idToIndexMap;
		std::vector<size_t> _availableData;
		std::vector<size_t> _availableIds;
		SegmentedVector<T, 10> _pool;
	};

	std::vector<ThreadLocalData> _threadLocalData;
};

template<class T>
size_t ObjectPool<T>::getNumThreads() const
{
	return _threadLocalData.size();
}

template<class T>
void ObjectPool<T>::setNumThreads(size_t val)
{
	assert(_threadLocalData.empty());
	_threadLocalData.resize(val);
	for (auto& tld : _threadLocalData)
		tld._idToIndexMap.reserve(1000);

}

template<class T>
void ObjectPool<T>::testReset()
{
	for (auto& tld : _threadLocalData) {
		tld._idToIndexMap.clear();
		tld._availableData.clear();
		tld._availableIds.clear();
		tld._pool.clear();
	}
}

template<class T>
inline void ObjectPool<T>::free(const ObjectPoolId& id)
{
	if (id.getThreadIndex() < _threadLocalData.size())
		_threadLocalData[id.getThreadIndex()].free(id.getIndex());
}

template<class T>
inline void ObjectPool<T>::unload(const ObjectPoolId& id)
{
	if (id.getThreadIndex() < _threadLocalData.size())
		_threadLocalData[id.getThreadIndex()].unload(id.getIndex());
}

template<class T>
inline ObjectPoolId ObjectPool<T>::create(const ObjectPoolId& id)
{
	size_t index = -1;
	if (id.getThreadIndex() < _threadLocalData.size())
		index = _threadLocalData[id.getThreadIndex()].create(id.getIndex());
	return ObjectPoolId(index, id.getThreadIndex());
}

template<class T>
inline ObjectPoolId ObjectPool<T>::create(T*& pObj, const ObjectPoolId& id)
{
	size_t index = -1;
	if (id.getThreadIndex() < _threadLocalData.size())
		index = _threadLocalData[id.getThreadIndex()].create(pObj, id.getIndex());
	return ObjectPoolId(index, id.getThreadIndex());

}

template<class T>
inline ObjectPoolId ObjectPool<T>::getObj(const ObjectPoolId& id, T*& pObj, bool allocateIfNeeded)
{
	size_t index = -1;
	if (id.getThreadIndex() < _threadLocalData.size())
		index = _threadLocalData[id.getThreadIndex()].getObj(id.getIndex(), pObj, allocateIfNeeded);
	return ObjectPoolId(index, id.getThreadIndex());
}

template<class T>
inline const T* ObjectPool<T>::getObj(const ObjectPoolId& id) const
{
	if (id.getThreadIndex() < _threadLocalData.size())
		return _threadLocalData[id.getThreadIndex()].getObj(id.getIndex());

	return nullptr;
}

template<class T>
inline T* ObjectPool<T>::getObj(const ObjectPoolId& id)
{
	if (id.getThreadIndex() < _threadLocalData.size()) {
		return _threadLocalData[id.getThreadIndex()].getObj(id.getIndex());
	}

	return nullptr;
}

template<class T>
size_t ObjectPool<T>::getNumAllocated(size_t threadIndex) const
{
	if (threadIndex < _threadLocalData.size())
		return _threadLocalData[threadIndex].getNumAllocated();
	
	size_t total = 0;
	for (const auto& tld : _threadLocalData) {
		total += tld.getNumAllocated();
	}

	return total;
}

template<class T>
size_t ObjectPool<T>::getNumAvailable(size_t threadIndex) const
{
	if (threadIndex < _threadLocalData.size())
		return _threadLocalData[threadIndex].getNumAvailable();

	size_t total = 0;
	for (const auto& tld : _threadLocalData) {
		total += tld.getNumAvailable();
	}

	return total;
}

template<class T>
size_t ObjectPool<T>::getNumAvailableIds(size_t threadIndex) const
{
	if (threadIndex < _threadLocalData.size())
		return _threadLocalData[threadIndex].getNumAvailableIds();

	size_t total = 0;
	for (const auto& tld : _threadLocalData) {
		total += tld.getNumAvailableIds();
	}

	return total;
}

template<class T>
size_t ObjectPool<T>::getNumUnloaded(size_t threadIndex) const
{
	if (threadIndex < _threadLocalData.size())
		return _threadLocalData[threadIndex].getNumUnloaded();

	size_t total = 0;
	for (const auto& tld : _threadLocalData) {
		total += tld.getNumUnloaded();
	}

	return total;
}

template<class T>
void ObjectPool<T>::ThreadLocalData::free(size_t id)
{
	if (id > _idToIndexMap.size())
		return;

	size_t index = _idToIndexMap[id];
	if (index != -1) {
		_availableData.push_back(index);
		_availableIds.push_back(id);
		_idToIndexMap[id] = -1;
	}
}

template<class T>
void ObjectPool<T>::ThreadLocalData::unload(size_t id)
{
	size_t index = _idToIndexMap[id];
	_pool[index] = {};
	if (index != -1)
		_availableData.push_back(index);
	_idToIndexMap[id] = -1;
}

template<class T>
inline size_t ObjectPool<T>::ThreadLocalData::create(size_t id)
{
	T* pObj;
	return create(pObj, id);
}

template<class T>
size_t ObjectPool<T>::ThreadLocalData::create(T*& pObj, size_t id)
{
	size_t index = -1;
	if (id >= _idToIndexMap.size()) {
		if (!_availableIds.empty()) {
			id = _availableIds.back();
			_availableIds.pop_back();
		}
		else {
			id = _idToIndexMap.size();
			_idToIndexMap.push_back(-1);
		}
	}

	index = _idToIndexMap[id];
	if (index >= _pool.size()) {
		if (_availableData.empty()) {
			_idToIndexMap[id] = index = _pool.size();
			_pool.push_back(T());
		}
		else {
			_idToIndexMap[id] = index = _availableData.back();
			_availableData.pop_back();
			_pool[index] = T();
		}
	}
	pObj = &_pool[index];

	return id;
}

template<class T>
size_t ObjectPool<T>::ThreadLocalData::getObj(size_t id, T*& pObj, bool allocateIfNeeded)
{
	size_t result = -1;
	if (id != -1 && id < _idToIndexMap.size())
		result = id;

	// Create also locks, so don't lock again
	if (result == -1)
		result = create();

	pObj = &_pool[_idToIndexMap[result]];

	return result;
}

template<class T>
const T* ObjectPool<T>::ThreadLocalData::getObj(size_t id) const
{
	if (id != -1 && id < _idToIndexMap.size()) {
		return &_pool[_idToIndexMap[id]];
	}
	return nullptr;
}

template<class T>
T* ObjectPool<T>::ThreadLocalData::getObj(size_t id)
{
	if (id != -1 && id < _idToIndexMap.size()) {
		return &_pool[_idToIndexMap[id]];
	}
	return nullptr;
}


template<class T>
size_t ObjectPool<T>::ThreadLocalData::getNumAllocated() const
{
	size_t num = 0;
	for (size_t idx : _idToIndexMap) {
		if (idx != -1)
			num++;
	}
	return num;
}

template<class T>
size_t ObjectPool<T>::ThreadLocalData::getNumAvailable() const
{
	return _availableData.size();
}

template<class T>
size_t ObjectPool<T>::ThreadLocalData::getNumAvailableIds() const
{
	return _availableIds.size();
}

template<class T>
size_t ObjectPool<T>::ThreadLocalData::getNumUnloaded() const
{
	return _idToIndexMap.size() - getNumAllocated();
}

}