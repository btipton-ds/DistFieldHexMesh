#pragma once

#include <vector>
#include <tm_vector3.h>
#include <segmentedVector.h>
#include <objectPoolId.h>
#include <threadLocalData.h>

namespace DFHM {

template<class T>
class ObjectPool {
public:
	ObjectPool() = default;

	size_t getNumThreads() const;
	virtual void setNumThreads(size_t val);

	void testReset();

	void free(const ObjectPoolId& id); // Permanently delete it
	void unload(const ObjectPoolId& id); // Free the memory, but keep the id

	bool idExists(const ObjectPoolId& id) const;
	virtual ObjectPoolId add(const T& obj, const ObjectPoolId& id = ObjectPoolId());
	const T& operator[](const ObjectPoolId& id) const;
	T& operator[](const ObjectPoolId& id);

	template<class F>
	void iterateInOrder(F fLambda) const
	{
		for (auto& data : _threadLocalData) {
			data.iterateInOrder(fLambda);
		}
	}

	size_t getNumAllocated(size_t threadIndex = -1) const;
	size_t getNumAvailable(size_t threadIndex = -1) const;
	size_t getNumAvailableIds(size_t threadIndex = -1) const;
	size_t getNumUnloaded(size_t threadIndex = -1) const;
private:
	using ThreadLocalDataT10 = DFHM_ObjectPool::ThreadLocalData<T, 10>;

	std::vector<ThreadLocalDataT10> _threadLocalData;
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
		tld.reserve(1000);
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
inline T& ObjectPool<T>::operator[](const ObjectPoolId& id)
{
	if (id.getThreadIndex() < _threadLocalData.size()) {
		ThreadLocalDataT10& t = _threadLocalData[id.getThreadIndex()];
		return t[id.getIndex()];

	}
	throw std::exception("ObjectPool<T>::operator[] out of range");
}

template<class T>
inline const T& ObjectPool<T>::operator[](const ObjectPoolId& id) const
{
	if (id.getThreadIndex() < _threadLocalData.size())
		return _threadLocalData[id.getThreadIndex()][id.getIndex()];
	throw std::exception("ObjectPool<T>::operator[] out of range");
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