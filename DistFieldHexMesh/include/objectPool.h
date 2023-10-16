#pragma once

namespace DFHM {

template<class T>
class ObjectPool {
public:
	ObjectPool();
	ObjectPool(const ObjectPool& src) = default;
	void free(size_t index);

	size_t create();
	size_t getObj(size_t index, T*& pObj, bool allocateIfNeeded);
	const T* getObj(size_t index) const;
	T* getObj(size_t index);
private:
	std::mutex _mutex;
	std::vector<size_t> _available;
	std::vector<T> _pool;
};

template<class T>
inline ObjectPool<T>::ObjectPool()
{
	std::lock_guard<std::mutex> lock(_mutex);
	_pool.reserve(1000);
}

template<class T>
inline void ObjectPool<T>::free(size_t index)
{
	std::lock_guard<std::mutex> lock(_mutex);
	_available.push_back(index);
}

template<class T>
size_t ObjectPool<T>::create()
{
	std::lock_guard<std::mutex> lock(_mutex);
	size_t result = _pool.size();
	_pool.push_back(T());
	return result;
}

template<class T>
size_t ObjectPool<T>::getObj(size_t index, T*& pObj, bool allocateIfNeeded)
{
	std::lock_guard<std::mutex> lock(_mutex);
	size_t result = -1;
	if (index != -1 && index < _pool.size()) {
		result = index;
	}
	else {
		if (_available.empty()) {
			result = _pool.size();
			_pool.push_back(T());
		}
		else {
			result = _available.back();
			_available.pop_back();
			{
				_pool[result] = T();
			}
		}
	}
	pObj = &_pool[result];
	return result;
}

template<class T>
inline const T* ObjectPool<T>::getObj(size_t index) const
{
	std::lock_guard<std::mutex> lock(_mutex);
	if (index != -1 && index < _pool.size()) {
		return &_pool[index];
	}
	return nullptr;
}

template<class T>
inline T* ObjectPool<T>::getObj(size_t index)
{
	std::lock_guard<std::mutex> lock(_mutex);
	if (index != -1 && index < _pool.size()) {
		return &_pool[index];
	}
	return nullptr;
}

class Cell;
class Block;
class Polygon;
class Polyhedron;
class Volume;

using VolumePtr = std::shared_ptr<Volume>;

class DataPool
{
protected:
	static ObjectPool<Polygon> _polygonPool;
	static ObjectPool<Polyhedron> _polyhedronPool;
	static ObjectPool<Cell> _cellPool;
	static ObjectPool<Block> _blockPool;
};

}