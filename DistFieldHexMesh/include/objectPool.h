#pragma once

#include <vector>
#include <tm_vector3.h>
#include <segmentedVector.h>
#include <objectPoolId.h>
#include <threadLocalData.h>
#include <stdexcept>

namespace DFHM {

#if 1
template<class T>
class ObjectPool {
public:
	ObjectPool(bool supportsReverseLookup);

	void testReset();
	void free(size_t id); // Permanently delete it

	void resize(size_t size);

	size_t findId(const T& obj) const;
	bool exists(size_t id) const;

	size_t add(const T& obj, size_t id = -1);

	const T* get(size_t id) const;
	T* get(size_t id);

	const T* get(const T& obj) const;
	T* get(const T& obj);

	const T& operator[](size_t id) const;
	T& operator[](size_t id);

	template<class F>
	void iterateInOrder(F fLambda) const
	{
		for (size_t id = 0; id < _idToIndexMap.size(); id++) {
			size_t index = _idToIndexMap[id];
			if (index < _data.size()) {
				fLambda(id, _data[index]);
			}
		}
	}

	bool empty() const
	{
		return size() == 0;
	}

	size_t size() const
	{
		return getNumAllocated();
	}

	size_t getNumAllocated() const
	{
		return _data.size() - _availableIndices.size();
	}

	size_t getNumAvailable() const
	{
		return _availableIndices.size();
	}

	size_t getNumUnloaded(size_t threadIndex = -1) const
	{
		return -1;
	}

private:
	struct CompareFunctor
	{
		inline CompareFunctor(const ObjectPool& owner)
		: _owner(owner)
		{
		}

		inline bool operator () (size_t lhs, size_t rhs) const
		{
			const T* pLHS = _owner.get(lhs);
			const T* pRHS = _owner.get(rhs);
			if (pLHS && pRHS)
				return *pLHS < *pRHS;
			throw std::runtime_error("comparing deleted object(s)");
		}

		const ObjectPool& _owner;
	};

	bool _supportsReverseLookup;
	std::vector<size_t> 
		_idToIndexMap,
		_availableIndices;
	std::vector<T> _data;

	// This oddball indirection was used so that the map of obj to id can use the vector of objects without duplicating the storage.
	// It's ugly, and a bit risky, but it avoids duplicating the storage of vertices, polygons and polyhedra.
	std::map<size_t, size_t, CompareFunctor> _objToIdMap;
	thread_local static const T* _tl_pCompareObj;
};

template<class T>
inline ObjectPool<T>::ObjectPool(bool supportsReverseLookup)
	: _objToIdMap(CompareFunctor(*this))
{
	_supportsReverseLookup = supportsReverseLookup;
}

template<class T>
void ObjectPool<T>::testReset()
{
}

template<class T>
void ObjectPool<T>::free(size_t id)
{
	if (id >= _idToIndexMap.size())
		return;

	size_t index = _idToIndexMap[id];
	if (index >= _data.size())
		return;

	_idToIndexMap[id] = -1; // Clear this id so it won't be used again
	_data[index] = T();
	_availableIndices.push_back(index);
}

template<class T>
void ObjectPool<T>::resize(size_t size)
{
	if (size > _idToIndexMap.size()) {
		for (size_t id = _idToIndexMap.size() - 1; id < size; id++) {
			_idToIndexMap.push_back(id);
		}
	}
}

template<class T>
size_t ObjectPool<T>::findId(const T& obj) const 
{
	if (_supportsReverseLookup) {
		_tl_pCompareObj = &obj;
		auto iter = _objToIdMap.find(-1);
		if (iter != _objToIdMap.end())
			return iter->second;
	}
	return -1;
}

template<class T>
inline bool ObjectPool<T>::exists(size_t id) const
{
	return (id < _idToIndexMap.size() && _idToIndexMap[id] < _data.size());
}

template<class T>
size_t ObjectPool<T>::add(const T& obj, size_t id)
{
	size_t result = -1;
	size_t index = -1;
	if (id < _idToIndexMap.size()) {
		result = id;
		index = _idToIndexMap[id];
		if (index < _data.size()) {
			_data[index] = obj;
		} else {
			if (_availableIndices.empty())
				index = _data.size();
			else {
				index = _availableIndices.back();
				_availableIndices.pop_back();
			}
			_idToIndexMap[id] = index;
			_data[index] = obj;
		}
	} else {
		result = _idToIndexMap.size();
		if (_availableIndices.empty()) {
			index = _data.size();
			_idToIndexMap.push_back(index);
			_data.push_back(obj);
		} else {
			index = _availableIndices.back();
			_availableIndices.pop_back();
			_data[index] = obj;
		}

	}

	return result;
}

template<class T>
const T* ObjectPool<T>::get(size_t id) const
{
	if (id == -1)
		return _tl_pCompareObj;
	else if (id < _idToIndexMap.size()) {
		size_t index = _idToIndexMap[id];
		if (index < _data.size()) {
			return &_data[index];
		}
	}
	return nullptr;
}

template<class T>
T* ObjectPool<T>::get(size_t id)
{
	if (id < _idToIndexMap.size()) {
		size_t index = _idToIndexMap[id];
		if (index < _data.size()) {
			return &_data[index];
		}
	}
	return nullptr;
}

template<class T>
const T* ObjectPool<T>::get(const T& obj) const
{
	size_t id = findId(obj);
	return get(id);
}

template<class T>
T* ObjectPool<T>::get(const T& obj)
{
	size_t id = findId(obj);
	return get(id);
}

template<class T>
const T& ObjectPool<T>::operator[](size_t id) const
{
	return _data[_idToIndexMap[id]];
}

template<class T>
T& ObjectPool<T>::operator[](size_t id)
{
	return _data[_idToIndexMap[id]];
}

#else

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
	bool _supportsReverseLookup;

	std::vector<ThreadLocalDataT10> _threadLocalData;
};

template<class T>
inline ObjectPool<T>::ObjectPool(bool needReverseLookup)
{
	_supportsReverseLookup = needReverseLookup;
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
		tld.init(_supportsReverseLookup, 1000);
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
	if (!_supportsReverseLookup) {
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
	if (_supportsReverseLookup) {
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
	if (_supportsReverseLookup) {
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

#endif

}
