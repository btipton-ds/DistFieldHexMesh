#pragma once

#include <mutex>
#include <vector>
#include <tm_vector3.h>

namespace DFHM {

template<class T>
class ObjectPool {
public:
	ObjectPool();
	ObjectPool(const ObjectPool& src) = default;
	void free(size_t id); // Permanently delete it
	void unload(size_t id); // Free the memory, but keep the id

	size_t create(size_t id = -1);
	size_t getObj(size_t id, T*& pObj, bool allocateIfNeeded);
	const T* getObj(size_t id) const;
	T* getObj(size_t id);

private:
	std::mutex _mutex;
	std::vector<size_t> _idToIndexMap;
	std::vector<size_t> _availableData;
	std::vector<size_t> _availableIds;
	std::vector<T> _pool;
};

template<class T>
ObjectPool<T>::ObjectPool()
{
	std::lock_guard<std::mutex> lock(_mutex);
	_pool.reserve(1000);
	_idToIndexMap.reserve(1000);
}

template<class T>
void ObjectPool<T>::free(size_t id)
{
	std::lock_guard<std::mutex> lock(_mutex);
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
	std::lock_guard<std::mutex> lock(_mutex);
	size_t index = _idToIndexMap[id];
	_pool[index] = {};
	if (index != -1) 
		_availableData.push_back(index);
	_idToIndexMap[id] = -1;
}

template<class T>
size_t ObjectPool<T>::create(size_t id /* = -1*/)
{
	std::lock_guard<std::mutex> lock(_mutex);

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

	return id;
}

template<class T>
size_t ObjectPool<T>::getObj(size_t id, T*& pObj, bool allocateIfNeeded)
{
	std::lock_guard<std::mutex> lock(_mutex);
	size_t result = -1;
	if (id != -1 && id < _idToIndexMap.size()) {
		result = id;
	} else {
		result = create();
	}
	pObj = &_pool[_idToIndexMap[result]];
	return result;
}

template<class T>
const T* ObjectPool<T>::getObj(size_t id) const
{
	std::lock_guard<std::mutex> lock(_mutex);
	if (id != -1 && id < _idToIndexMap.size()) {
		return &_pool[_idToIndexMap[id]];
	}
	return nullptr;
}

template<class T>
T* ObjectPool<T>::getObj(size_t id)
{
	std::lock_guard<std::mutex> lock(_mutex);
	if (id != -1 && id < _idToIndexMap.size()) {
		return &_pool[_idToIndexMap[id]];
	}
	return nullptr;
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