#pragma once

#include <patient_lock_guard.h>
#include <objectPool.h>

namespace DFHM {

template<class T>
class ObjectPoolWMutex {
public:
	ObjectPoolWMutex(ObjectPoolOwner* pBlock, bool supportsReverseLookup, size_t objectSegmentSize = 512);
	ObjectPoolWMutex(const ObjectPoolWMutex& rhs);

	bool free(const Index3DId& id); // Permanently delete it
	bool removeFromLookup(const Index3DId& id);
	void addToLookup(const Index3DId& id);
	Index3DId findId(const T& obj) const;
	bool exists(const Index3DId& id) const;

	Index3DId findOrAdd(const T& obj, const Index3DId& currentId = Index3DId());

	const T* get(const Index3DId& id ) const;
	T* get(const Index3DId& id);

	const T* get(const T& obj) const;
	T* get(const T& obj);

	template<class F>
	void iterateInOrderTS(F fLambda) const;
	template<class F>
	void iterateInOrderTS(F fLambda);

	const T& operator[](const Index3DId& id) const;
	T& operator[](const Index3DId& id);

	bool empty() const;
	size_t size() const;

	MutexType& getMutex() const;

	void lock() const;
	bool tryLock() const;
	void unlock() const;
private:
	mutable std::thread::id _lockedId = std::this_thread::get_id();

	mutable MutexType _mutex;
	ObjectPool<T> _data;
};

template<class T>
inline ObjectPoolWMutex<T>::ObjectPoolWMutex(ObjectPoolOwner* pBlock, bool supportsReverseLookup, size_t objectSegmentSize)
	: _data(pBlock, supportsReverseLookup, objectSegmentSize)
{
}

template<class T>
inline ObjectPoolWMutex<T>::ObjectPoolWMutex(const ObjectPoolWMutex& rhs)
	: _data(rhs._data)
{
}

template<class T>
bool ObjectPoolWMutex<T>::free(const Index3DId& id) // Permanently delete it
{
	patient_lock_guard g(_mutex);
	return _data.free(id);
}

template<class T>
inline bool ObjectPoolWMutex<T>::removeFromLookup(const Index3DId& id)
{
	patient_lock_guard g(_mutex);
	return _data.removeFromLookup(id);
}

template<class T>
inline void ObjectPoolWMutex<T>::addToLookup(const Index3DId& id)
{
	patient_lock_guard g(_mutex);
	_data.addToLookup(id);
}

template<class T>
inline Index3DId ObjectPoolWMutex<T>::findId(const T& obj) const
{
	patient_lock_guard g(_mutex);
	return _data.findId(obj);
}

template<class T>
bool ObjectPoolWMutex<T>::exists(const Index3DId& id) const
{
	patient_lock_guard g(_mutex);
	return _data.exists(id);
}

template<class T>
inline Index3DId ObjectPoolWMutex<T>::findOrAdd(const T& obj, const Index3DId& currentId)
{
	patient_lock_guard g(_mutex);
	return _data.findOrAdd(obj, currentId);
}

template<class T>
inline const T* ObjectPoolWMutex<T>::get(const Index3DId& id) const
{
	if (exists(id))
		return _data.get(id);
	return nullptr;
}

template<class T>
inline T* ObjectPoolWMutex<T>::get(const Index3DId& id)
{
	if (exists(id))
		return _data.get(id);
	return nullptr;
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
template<class F>
inline void ObjectPoolWMutex<T>::iterateInOrderTS(F fLambda) const
{
	size_t numIds = size();

	const auto& blockIdx = _data.getBlockPtr()->getBlockIdx();
	for (size_t id = 0; id < numIds; id++) {
		auto* pEntry = _data.get(Index3DId(blockIdx, id));
		if (pEntry) {
			patient_lock_guard g(pEntry->getMutex());
			fLambda(*pEntry);
		}
	}
}

template<class T>
template<class F>
inline void ObjectPoolWMutex<T>::iterateInOrderTS(F fLambda)
{
	size_t numIds = size();

	const auto& blockIdx = _data.getBlockPtr()->getBlockIdx();
	for (size_t id = 0; id < numIds; id++) {
		auto* pEntry = _data.get(Index3DId(blockIdx, id));
		if (pEntry) {
			patient_lock_guard g(pEntry->getMutex());
			fLambda(*pEntry);
		}
	}
}


template<class T>
inline const T& ObjectPoolWMutex<T>::operator[](const Index3DId& id) const
{
	return _data[id];
}

template<class T>
inline T& ObjectPoolWMutex<T>::operator[](const Index3DId& id)
{
	return _data[id];
}

template<class T>
bool ObjectPoolWMutex<T>::empty() const
{
	patient_lock_guard g(_mutex);
	return _data.empty();
}


template<class T>
size_t ObjectPoolWMutex<T>::size() const
{
	patient_lock_guard g(_mutex);
	return _data.size();
}

template<class T>
MutexType& ObjectPoolWMutex<T>::getMutex() const
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