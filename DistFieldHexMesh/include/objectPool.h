#pragma once

#include <vector>
#include <map>
#include <mutex>
#include <tm_vector3.h>
#include <stdexcept>

namespace DFHM {

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

	bool empty() const;
	size_t size() const;
	size_t getNumAllocated() const;
	size_t getNumAvailable() const;
	size_t getNumUnloaded() const;

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
		_idToIndexMap.resize(size);
		_data.reserve(size);
		for (size_t id = 0; id < size; id++) {
			_idToIndexMap[id] = _data.size();
			_data.push_back(T());
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
size_t ObjectPool<T>::add(const T& obj, size_t currentId)
{
	if (_supportsReverseLookup) {
		size_t id = findId(obj);
		if (id != -1)
			return id;
	}

	size_t result = -1;
	size_t index = -1;
	if (currentId < _idToIndexMap.size()) {
		result = currentId;
		index = _idToIndexMap[result];
		if (index < _data.size()) {
			_data[index] = obj;
		} else {
			if (_availableIndices.empty())
				index = _data.size();
			else {
				index = _availableIndices.back();
				_availableIndices.pop_back();
			}
			_idToIndexMap[result] = index;
			if (index >= _data.size())
				_data.resize(index + 1);
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
			if (index >= _data.size())
				_data.resize(index + 1);
			_data[index] = obj;
		}

	}

	if (_supportsReverseLookup) {
		_objToIdMap.insert(std::make_pair(result, result));
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

template<class T>
bool ObjectPool<T>::empty() const
{
	return size() == 0;
}

template<class T>
size_t ObjectPool<T>::size() const
{
	return getNumAllocated();
}

template<class T>
size_t ObjectPool<T>::getNumAllocated() const
{
	return _data.size() - _availableIndices.size();
}

template<class T>
size_t ObjectPool<T>::getNumAvailable() const
{
	return _availableIndices.size();
}

template<class T>
size_t ObjectPool<T>::getNumUnloaded() const
{
	return -1;
}

}
