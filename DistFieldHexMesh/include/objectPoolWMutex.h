#pragma once

#include <objectPool.h>

namespace DFHM {

template<class T>
class ObjectPoolWMutex {
public:
	ObjectPoolWMutex(bool supportsReverseLookup, size_t objectSegmentSize = 512);
	ObjectPoolWMutex(const ObjectPoolWMutex& rhs);

	bool free(size_t id); // Permanently delete it
	bool removeFromLookup(size_t id);
	void addToLookup(size_t id);
	size_t findId(const T& obj) const;

	Index3DId findOrAdd(const Index3D& blockIdx, const T& obj, size_t id = -1);

	const T* get(size_t idx) const;
	T* get(size_t idx);

	const T* get(const T& obj) const;
	T* get(const T& obj);

	template<class F>
	void iterateInOrder(F fLambda) const
	{
		std::lock_guard g(_mutex);
		_data.iterateInOrder(fLambda);
	}

	const T& operator[](size_t id) const;
	T& operator[](size_t id);

	bool empty() const;
	size_t size() const;

	std::recursive_mutex& getMutex() const;

	void lock() const;
	bool tryLock() const;
	void unlock() const;
private:
	mutable std::thread::id _lockedId = std::this_thread::get_id();

	mutable std::recursive_mutex _mutex;
	ObjectPool<T> _data;
};

template<class T>
inline ObjectPoolWMutex<T>::ObjectPoolWMutex(bool supportsReverseLookup, size_t objectSegmentSize)
	: _data(supportsReverseLookup, objectSegmentSize)
{
}

template<class T>
inline ObjectPoolWMutex<T>::ObjectPoolWMutex(const ObjectPoolWMutex& rhs)
	: _data(rhs._data)
{
}

template<class T>
bool ObjectPoolWMutex<T>::free(size_t id) // Permanently delete it
{
	std::lock_guard g(_mutex);
	return _data.free(id);
}

template<class T>
inline bool ObjectPoolWMutex<T>::removeFromLookup(size_t id)
{
	std::lock_guard g(_mutex);
	return _data.removeFromLookup(id);
}

template<class T>
inline void ObjectPoolWMutex<T>::addToLookup(size_t id)
{
	std::lock_guard g(_mutex);
	_data.addToLookup(id);
}

template<class T>
inline size_t ObjectPoolWMutex<T>::findId(const T& obj) const
{
	std::lock_guard g(_mutex);
	return _data.findId(obj);
}

template<class T>
inline Index3DId ObjectPoolWMutex<T>::findOrAdd(const Index3D& blockIdx, const T& vert, size_t id)
{
	std::lock_guard g(_mutex);
	return _data.findOrAdd(blockIdx, vert, id);
}

template<class T>
inline const T* ObjectPoolWMutex<T>::get(size_t idx) const
{
	if (_lockedId != std::this_thread::get_id())
		throw std::runtime_error("object pool mutex is not locked.");
	return _data.get(idx);
}

template<class T>
inline T* ObjectPoolWMutex<T>::get(size_t idx)
{
	if (_lockedId != std::this_thread::get_id())
		throw std::runtime_error("object pool mutex is not locked.");
	return _data.get(idx);
}

template<class T>
inline const T* ObjectPoolWMutex<T>::get(const T& obj) const
{
	return _data.get(obj);
}

template<class T>
inline T* ObjectPoolWMutex<T>::get(const T& obj)
{
	return _data.get(obj);
}

template<class T>
inline const T& ObjectPoolWMutex<T>::operator[](size_t id) const
{
	if (_lockedId != std::this_thread::get_id())
		throw std::runtime_error("object pool mutex is not locked.");
	return _data[id];
}

template<class T>
inline T& ObjectPoolWMutex<T>::operator[](size_t id)
{
	if (_lockedId != std::this_thread::get_id())
		throw std::runtime_error("object pool mutex is not locked.");
	return _data[id];
}

template<class T>
bool ObjectPoolWMutex<T>::empty() const
{
	return _data.empty();
}


template<class T>
size_t ObjectPoolWMutex<T>::size() const
{
	std::lock_guard g(_mutex);
	return _data.size();
}

template<class T>
std::recursive_mutex& ObjectPoolWMutex<T>::getMutex() const
{
	return _mutex;
}

template<class T>
inline void ObjectPoolWMutex<T>::lock() const {
	_mutex.lock();
	_lockedId = std::this_thread::get_id();
}

template<class T>
inline bool ObjectPoolWMutex<T>::tryLock() const {
	bool result = _mutex.tryLock();
	if (result) {
		_lockedId = std::this_thread::get_id();
	}
}

template<class T>
inline void ObjectPoolWMutex<T>::unlock() const {
	if (_lockedId != std::this_thread::get_id())
		throw std::runtime_error("object pool mutex is not locked.");
	_mutex.unlock();
}

}