#pragma once

#include <vector>
#include <map>
#include <mutex>
#include <stdexcept>
#include <Index3D.h>
#include <tm_vector3.h>

namespace DFHM {

class ObjectPoolOwner {
public:
	virtual const Index3D& getBlockIdx() const = 0;
};

template<class T>
class ObjectPool {
public:
	ObjectPool(bool supportsReverseLookup, size_t objectSegmentSize = 512);

	void testReset();
	bool free(size_t id); // Permanently delete it
	bool removeFromLookup(size_t id);
	void addToLookup(size_t id);

	void resize(size_t size);

	size_t findId(const T& obj) const;
	bool exists(size_t id) const;

	Index3DId findOrAdd(ObjectPoolOwner* pBlock, const T& obj, size_t id = -1);

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
			if (index != -1) {
				fLambda(id, *getEntry(index));
			}
		}
	}

	template<class F>
	void iterateInOrder(F fLambda)
	{
		for (size_t id = 0; id < _idToIndexMap.size(); id++) {
			size_t index = _idToIndexMap[id];
			if (index != -1) {
				fLambda(id, *getEntry(index));
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

	bool calIndices(size_t index, size_t& segNum, size_t& segIdx) const;
	T* getEntry(size_t index);
	const T* getEntry(size_t index) const;

	bool _supportsReverseLookup;
	const size_t _objectSegmentSize;
	std::vector<size_t> 
		_idToIndexMap,
		_availableIndices;

	using ObjectSegPtr = std::shared_ptr<std::vector<T>>;
	std::vector<ObjectSegPtr> _objectSegs;

	// This oddball indirection was used so that the map of obj to id can use the vector of objects without duplicating the storage.
	// It's ugly, and a bit risky, but it avoids duplicating the storage of vertices, polygons and polyhedra.
	std::map<size_t, size_t, CompareFunctor> _objToIdMap;
	thread_local static const T* _tl_pCompareObj;
};

template<class T>
inline ObjectPool<T>::ObjectPool(bool supportsReverseLookup, size_t objectSegmentSize)
	: _objToIdMap(CompareFunctor(*this))
	, _objectSegmentSize(objectSegmentSize)
{
	_supportsReverseLookup = supportsReverseLookup;
}

template<class T>
void ObjectPool<T>::testReset()
{
}

template<class T>
inline bool ObjectPool<T>::calIndices(size_t index, size_t& segNum, size_t& segIdx) const
{
	segNum = index / _objectSegmentSize;
	segIdx = index % _objectSegmentSize;
	return segNum < _objectSegs.size() && segIdx < (*_objectSegs[segNum]).size();
}

template<class T>
inline T* ObjectPool<T>::getEntry(size_t index)
{
	size_t segNum, segIdx;
	if (calIndices(index, segNum, segIdx)) {
		return &((*_objectSegs[segNum])[segIdx]);
	}
	return nullptr;
}

template<class T>
inline const T* ObjectPool<T>::getEntry(size_t index) const
{
	size_t segNum, segIdx;
	if (calIndices(index, segNum, segIdx)) {
		return &((*_objectSegs[segNum])[segIdx]);
	}
	return nullptr;
}

template<class T>
bool ObjectPool<T>::free(size_t id)
{
	if (id >= _idToIndexMap.size())
		return false;

	size_t index = _idToIndexMap[id];
	if (index >= _objectSegs.size())
		return false;

	removeFromLookup(id);

	T* p = getEntry(index);
	if (p) {
		*p = T();
	}

	_idToIndexMap[id] = -1; // Clear this id so it won't be used again
	_availableIndices.push_back(index);

	return true;
}

template<class T>
inline bool ObjectPool<T>::removeFromLookup(size_t id)
{
	if (_supportsReverseLookup) {
		auto iter = _objToIdMap.find(id);
		if (iter != _objToIdMap.end()) {
			_objToIdMap.erase(iter);
			return true;
		}
	}

	return false;
}

template<class T>
inline void ObjectPool<T>::addToLookup(size_t id)
{
	if (_supportsReverseLookup) {
		_objToIdMap.insert(std::make_pair(id, id));
	}

}

template<class T>
void ObjectPool<T>::resize(size_t size)
{
	if (size > _idToIndexMap.size()) {
		_idToIndexMap.resize(size);
		size_t numSegs = size / _objectSegmentSize + 1;
		_objectSegs.resize(numSegs);
		for (size_t id = 0; id < size; id++) {
			_idToIndexMap[id] = _objectSegs.size();
		}
		size_t remaining = size;
		for (size_t i = 0; i < numSegs; i++) {
			// Reserve the segment size so the array won't resize during use
			_objectSegs[i] = std::make_shared<std::vector<T>>();
			_objectSegs[i]->reserve(_objectSegmentSize);
			if (size > _objectSegmentSize) {
				_objectSegs[i]->resize(_objectSegmentSize);
				size -= _objectSegmentSize;
			} else {
				_objectSegs[i]->resize(size);
				size = 0;
			}
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
	bool result (id < _idToIndexMap.size() && _idToIndexMap[id] != -1);
	if (!result) {
		int dbgBreak = 1;
	}

	return result;
}

template<class T>
Index3DId ObjectPool<T>::findOrAdd(ObjectPoolOwner* pBlock, const T& obj, size_t currentId)
{
	if (_supportsReverseLookup) {
		size_t id = findId(obj);
		if (id != -1)
			return Index3DId(pBlock->getBlockIdx(), id);
	}

	size_t result = -1, index = -1, segNum = -1, segIdx = -1;
	if (currentId < _idToIndexMap.size()) {
		result = currentId;
		index = _idToIndexMap[result];

		calIndices(index, segNum, segIdx);
		if (segNum >= _objectSegs.size()) {
			// Reserve the segment size so the array won't resize during use
			_objectSegs.push_back(std::make_shared<std::vector<T>>());
			_objectSegs.back()->reserve(_objectSegmentSize);
		}
		auto& segData = *_objectSegs[segNum];

		if (segIdx < segData.size()) {
			segData[segIdx] = obj;
		}
		else {
			if (_availableIndices.empty())
				index = _objectSegs.size();
			else {
				index = _availableIndices.back();
				_availableIndices.pop_back();
			}
			_idToIndexMap[result] = index;

			calIndices(index, segNum, segIdx);
			if (segNum >= _objectSegs.size()) {
				// Reserve the segment size so the array won't resize during use
				_objectSegs.push_back(std::make_shared<std::vector<T>>());
				_objectSegs.back()->reserve(_objectSegmentSize);
			}
			auto& segData = *_objectSegs[segNum];

			if (index >= segData.size())
				segData.resize(index + 1);
			segData[segIdx] = obj;
		}
	}
	else {
		result = _idToIndexMap.size();
		if (_availableIndices.empty()) {
			if (_objectSegs.empty() || _objectSegs.back()->size() >= _objectSegmentSize) {
				// Reserve the segment size so the array won't resize during use
				_objectSegs.push_back(make_shared<std::vector<T>>());
				_objectSegs.back()->reserve(_objectSegmentSize);
			}
			segNum = _objectSegs.size() - 1;

			auto& segData = *_objectSegs.back();

			segIdx = segData.size();
			segData.push_back(obj);
			index = segNum * _objectSegmentSize + segIdx;
			_idToIndexMap.push_back(index);
		}
		else {
			index = _availableIndices.back();
			_availableIndices.pop_back();

			calIndices(index, segNum, segIdx);
			if (segNum >= _objectSegs.size()) {
				// Reserve the segment size so the array won't resize during use
				_objectSegs.push_back(std::make_shared<std::vector<T>>());
				_objectSegs.back()->reserve(_objectSegmentSize);
			}
			auto& segData = *_objectSegs[segNum];

			if (segIdx >= segData.size())
				segData.resize(index + 1);
			segData[segIdx] = obj;
		}
	}

	getEntry(result)->setId(pBlock, result);

	addToLookup(result);

	return Index3DId(pBlock->getBlockIdx(), result);
}

template<class T>
const T* ObjectPool<T>::get(size_t id) const
{
	if (id == -1)
		return _tl_pCompareObj;
	else if (id < _idToIndexMap.size()) {
		size_t index = _idToIndexMap[id];
		return getEntry(index);
	}
	return nullptr;
}

template<class T>
T* ObjectPool<T>::get(size_t id)
{
	if (id == -1)
		return (T*)_tl_pCompareObj;
	else if (id < _idToIndexMap.size()) {
		size_t index = _idToIndexMap[id];
		return getEntry(index);
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
	return *getEntry(_idToIndexMap[id]);
}

template<class T>
T& ObjectPool<T>::operator[](size_t id)
{
	return *getEntry(_idToIndexMap[id]);
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
	size_t size = 0;
	for (const auto& p : _objectSegs) {
		if (p)
			size += p->size();
	}
	return size - _availableIndices.size();
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
