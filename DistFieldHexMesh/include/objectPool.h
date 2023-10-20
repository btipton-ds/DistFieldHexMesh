#pragma once

#include <mutex>
#include <vector>
#include <tm_vector3.h>

namespace DFHM {

template<class T, int BITS>
class SegmentedVector {
public:
	T& operator[](size_t idx);
	const T& operator[](size_t idx) const;
	void push_back(const T& val);
	size_t size() const;
	void reserve(size_t size);
	void clear();

	std::recursive_mutex& getMutex() const;
private:
	mutable std::recursive_mutex _mutex;

	std::vector<std::shared_ptr<std::vector<T>>> _data;
};

template<class T, int BITS>
T& SegmentedVector<T, BITS>::operator[](size_t idx)
{
	const size_t mask = (((size_t)1) << BITS) - 1;
	std::lock_guard<std::recursive_mutex> lock(_mutex);
	size_t idx0 = idx >> BITS;
	size_t idx1 = idx & mask;
	return _data[idx0]->operator[](idx1);
}

template<class T, int BITS>
const T& SegmentedVector<T, BITS>::operator[](size_t idx) const
{
	const size_t mask = (((size_t)1) << BITS) - 1;
	std::lock_guard<std::recursive_mutex> lock(_mutex);
	size_t idx0 = idx >> BITS;
	size_t idx1 = idx & mask;
	return _data[idx0]->operator[](idx1);
}

template<class T, int BITS>
void SegmentedVector<T, BITS>::push_back(const T& val)
{
	const size_t maxSize = (((size_t)1) << BITS);
	std::lock_guard<std::recursive_mutex> lock(_mutex);
	if (_data.empty() || _data.back()->size() >= maxSize) {
		auto pNew = std::make_shared <std::vector<T>>();
		pNew->reserve(maxSize);
		pNew->push_back(val);
		_data.push_back(pNew);
	} else {
		_data.back()->push_back(val);
	}
}

template<class T, int BITS>
size_t SegmentedVector<T, BITS>::size() const
{
	std::lock_guard<std::recursive_mutex> lock(_mutex);
	size_t result = 0;
	for (auto p : _data)
		result += p->size();
	return result;
}

template<class T, int BITS>
inline std::recursive_mutex& SegmentedVector<T, BITS>::getMutex() const
{
	return _mutex;
}

template<class T, int BITS>
void SegmentedVector<T, BITS>::clear()
{
	_data.clear();
}

template<class T>
class ObjectPool {
public:
	ObjectPool();

	void testReset();

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
private:
	ObjectPool(const ObjectPool& src) = default;

	std::vector<size_t> _idToIndexMap;
	std::vector<size_t> _availableData;
	std::vector<size_t> _availableIds;
	SegmentedVector<T, 8> _pool;
};

template<class T>
ObjectPool<T>::ObjectPool()
{
	std::lock_guard<std::recursive_mutex> lock(_pool.getMutex());
	_idToIndexMap.reserve(1000);
}

template<class T>
void ObjectPool<T>::testReset()
{
	std::lock_guard<std::recursive_mutex> lock(_pool.getMutex());
	_idToIndexMap.clear();
	_availableData.clear();
	_availableIds.clear();
	_pool.clear();
}

template<class T>
void ObjectPool<T>::free(size_t id)
{
	if (id > _idToIndexMap.size())
		return;
	std::lock_guard<std::recursive_mutex> lock(_pool.getMutex());
	size_t index = _idToIndexMap[id];
	if (index != -1) {
		_availableData.push_back(index);
		_availableIds.push_back(id);
		_idToIndexMap[id] = -1;
	}
}

template<class T>
void ObjectPool<T>::unload(size_t id)
{
	std::lock_guard<std::recursive_mutex> lock(_pool.getMutex());
	size_t index = _idToIndexMap[id];
	_pool[index] = {};
	if (index != -1) 
		_availableData.push_back(index);
	_idToIndexMap[id] = -1;
}

template<class T>
inline size_t ObjectPool<T>::create(size_t id /* = -1*/)
{
	T* pObj;
	return create(pObj, id);
}

template<class T>
size_t ObjectPool<T>::create(T*& pObj, size_t id /* = -1*/)
{
	std::lock_guard<std::recursive_mutex> lock(_pool.getMutex());

	size_t index = -1;
	if (id >= _idToIndexMap.size()) {
		if (!_availableIds.empty()) {
			id = _availableIds.back();
			_availableIds.pop_back();
		} else {
			id = _idToIndexMap.size();
			_idToIndexMap.push_back(-1);
		}
	}

	index = _idToIndexMap[id];
	if (index >= _pool.size()) {
		if (_availableData.empty()) {
			_idToIndexMap[id] = index = _pool.size();
			_pool.push_back(T());
		} else {
			_idToIndexMap[id] = index = _availableData.back();
			_availableData.pop_back();
			_pool[index] = T();
		}
	}
	pObj = &_pool[index];

	return id;
}

template<class T>
size_t ObjectPool<T>::getObj(size_t id, T*& pObj, bool allocateIfNeeded)
{
	size_t result = -1;
	if (id != -1 && id < _idToIndexMap.size())
		result = id;

	// Create also locks, so don't lock again
	if (result == -1) 
		result = create();
	
	{
		std::lock_guard<std::recursive_mutex> lock(_pool.getMutex());
		pObj = &_pool[_idToIndexMap[result]];
	}
	return result;
}

template<class T>
const T* ObjectPool<T>::getObj(size_t id) const
{
	std::lock_guard<std::recursive_mutex> lock(_pool.getMutex());
	if (id != -1 && id < _idToIndexMap.size()) {
		return &_pool[_idToIndexMap[id]];
	}
	return nullptr;
}

template<class T>
T* ObjectPool<T>::getObj(size_t id)
{
	std::lock_guard<std::recursive_mutex> lock(_pool.getMutex());
	if (id != -1 && id < _idToIndexMap.size()) {
		return &_pool[_idToIndexMap[id]];
	}
	return nullptr;
}

template<class T>
size_t ObjectPool<T>::getNumAllocated() const
{
	size_t num = 0;
	for (size_t idx : _idToIndexMap) {
		if (idx != -1)
			num++;
	}
	return num;
}

template<class T>
size_t ObjectPool<T>::getNumAvailable() const
{
	return _availableData.size();
}

template<class T>
size_t ObjectPool<T>::getNumAvailableIds() const
{
	return _availableIds.size();
}

template<class T>
size_t ObjectPool<T>::getNumUnloaded() const
{
	return _idToIndexMap.size() - getNumAllocated();
}

class Cell;
class Block;
class Polygon;
class Polyhedron;
class Volume;
class Vertex;

using VolumePtr = std::shared_ptr<Volume>;

class DataPool
{
protected:
	static ObjectPool<Vertex> _vertexPool;
	static ObjectPool<Polygon> _polygonPool;
	static ObjectPool<Polyhedron> _polyhedronPool;
	static ObjectPool<Cell> _cellPool;
	static ObjectPool<Block> _blockPool;
};

}