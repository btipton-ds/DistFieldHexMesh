#pragma once

#include <vector>
#include <tm_vector3.h>
#include <segmentedVector.h>
#include <objectPoolId.h>
#include <threadLocalData.h>
#include <stdexcept>

namespace DFHM {

template<class T>
class ObjectPool {
public:
	ObjectPool(bool needReverseLookup);

	size_t getNumThreads() const;
	virtual void setNumThreads(size_t val);

	void testReset();

	void free(const ObjectPoolId& id); // Permanently delete it
	void unload(const ObjectPoolId& id); // Free the memory, but keep the id

	ObjectPoolId findId(const T& obj) const;
	bool idExists(const ObjectPoolId& id) const;

	ObjectPoolId add(const T& obj, const ObjectPoolId& id = ObjectPoolId());

	const T* get(const ObjectPoolId& id) const;
	T* get(const ObjectPoolId& id);

	const T* get(const T& obj) const;
	T* get(const T& obj);

	const T& operator[](const ObjectPoolId& id) const;
	T& operator[](const ObjectPoolId& id);

	template<class F>
	void iterateInOrder(F fLambda) const
	{
		for (size_t threadNum = 0; threadNum < _threadLocalData.size(); threadNum++) {
			_threadLocalData[threadNum].iterateInOrder(threadNum, fLambda);
		}
	}

	template<class F>
	void iterateOverThread(size_t threadNum, F fLambda) const
	{
		const ThreadLocalDataT10& entry = _threadLocalData[threadNum];
		entry.iterateInOrder(threadNum, fLambda);
	}

	size_t getNumAllocated(size_t threadIndex = -1) const;
	size_t getNumAvailable(size_t threadIndex = -1) const;
	size_t getNumAvailableIds(size_t threadIndex = -1) const;
	size_t getNumUnloaded(size_t threadIndex = -1) const;
private:
	using ThreadLocalDataT10 = DFHM_ObjectPool::ThreadLocalData<T, 10>;
	bool _needReverseLookup;

	std::vector<ThreadLocalDataT10> _threadLocalData;
};

template<class T>
inline ObjectPool<T>::ObjectPool(bool needReverseLookup)
{
	_needReverseLookup = needReverseLookup;
}

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
		tld.init(_needReverseLookup, 1000);
}

template<class T>
void ObjectPool<T>::testReset()
{
	for (auto& tld : _threadLocalData) {
		tld.reset();
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
ObjectPoolId ObjectPool<T>::findId(const T& obj) const
{
	if (!_needReverseLookup) {
		throw std::runtime_error("reverseLookup is not enabled for this pool.");
	}
	for (size_t threadNum = 0; threadNum < _threadLocalData.size(); threadNum++) {
		size_t index = _threadLocalData[threadNum].find(obj);
		if (index != -1)
			return ObjectPoolId(index, threadNum);
	}
	return ObjectPoolId();
}

template<class T>
bool ObjectPool<T>::idExists(const ObjectPoolId& id) const
{
	return (id.getThreadIndex() < _threadLocalData.size()) && _threadLocalData[id.getThreadIndex()].idExists(id.getIndex());
}

template<class T>
inline ObjectPoolId ObjectPool<T>::add(const T& pObj, const ObjectPoolId& id)
{
	size_t index = -1;
	if (id.getThreadIndex() < _threadLocalData.size())
		index = _threadLocalData[id.getThreadIndex()].add(pObj, id.getIndex());
	return ObjectPoolId(index, id.getThreadIndex());

}

template<class T>
const T* ObjectPool<T>::get(const ObjectPoolId& id) const
{
	if (id.getThreadIndex() < _threadLocalData.size()) {
		ThreadLocalDataT10& t = _threadLocalData[id.getThreadIndex()];
		return &t[id.getIndex()];
	}
	return nullptr;
}

template<class T>
T* ObjectPool<T>::get(const ObjectPoolId& id)
{
	if (id.getThreadIndex() < _threadLocalData.size()) {
		ThreadLocalDataT10& t = _threadLocalData[id.getThreadIndex()];
		return &t[id.getIndex()];
	}
	return nullptr;
}

template<class T>
const T* ObjectPool<T>::get(const T& obj) const
{
	if (_needReverseLookup) {
		for (size_t threadNum = 0; threadNum < _threadLocalData.size(); threadNum++) {
			const T* ptr = _threadLocalData[threadNum].get(obj);
			if (ptr)
				return ptr;
		}

	}
	return nullptr;
}

template<class T>
T* ObjectPool<T>::get(const T& obj)
{
	if (_needReverseLookup) {
		for (size_t threadNum = 0; threadNum < _threadLocalData.size(); threadNum++) {
			T* ptr = _threadLocalData[threadNum].get(obj);
			if (ptr)
				return ptr;
		}
	}
	return nullptr;
}

template<class T>
inline T& ObjectPool<T>::operator[](const ObjectPoolId& id)
{
	if (id.getThreadIndex() < _threadLocalData.size()) {
		ThreadLocalDataT10& t = _threadLocalData[id.getThreadIndex()];
		return t[id.getIndex()];

	}
	throw std::runtime_error("ObjectPool<T>::operator[] out of range");
}

template<class T>
inline const T& ObjectPool<T>::operator[](const ObjectPoolId& id) const
{
	if (id.getThreadIndex() < _threadLocalData.size())
		return _threadLocalData[id.getThreadIndex()][id.getIndex()];
	throw std::runtime_error("ObjectPool<T>::operator[] out of range");
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



}
