#pragma once

/*
This file is part of the DistFieldHexMesh application/library.

	The DistFieldHexMesh application/library is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	The DistFieldHexMesh application/library is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	This link provides the exact terms of the GPL license <https://www.gnu.org/licenses/>.

	The author's interpretation of GPL 3 is that if you receive money for the use or distribution of the DistFieldHexMesh application/library or a derivative product, GPL 3 no longer applies.

	Under those circumstances, the author expects and may legally pursue a reasoble share of the income. To avoid the complexity of agreements and negotiation, the author makes
	no specific demands in this regard. Compensation of roughly 1% of net or $5 per user license seems appropriate, but is not legally binding.

	In lay terms, if you make a profit by using the DistFieldHexMesh application/library (violating the spirit of Open Source Software), I expect a reasonable share for my efforts.

	Robert R Tipton - Author

	Dark Sky Innovative Solutions http://darkskyinnovation.com/
*/

#include <memory>
#include <string>
#include <vector>
#include <map>
#include <iostream>
#include <mutexType.h>
#include <stdexcept>
#include <tm_vector3.h>
#include <Index3D.h>
#include <patient_lock_guard.h>

namespace DFHM {

class Block;
class Volume;
class Logger;

// Have to store the pointer so we can call block functions. Just the block index is not enough
class ObjectPoolOwner {
public:
	virtual const Index3D& getBlockIdx() const = 0;

	virtual Volume* getVolume() = 0;
	virtual const Volume* getVolume() const = 0;

	virtual const Block* getOwner(const Index3D& blockIdx) const = 0;
	virtual Block* getOwner(const Index3D& blockIdx) = 0;

	std::shared_ptr<Logger> getLogger() const;
	std::string getLoggerNumericCode() const;

	void disableDestructors();
private:
	friend class ObjectPoolOwnerUser;
	bool _isBeingDestroyed = false;
	mutable std::string _filename;
};

class ObjectPoolOwnerUser {
public:
	ObjectPoolOwnerUser() = default;
	ObjectPoolOwnerUser(const ObjectPoolOwnerUser& src);
	ObjectPoolOwnerUser(const ObjectPoolOwner* poolOwner, size_t id = -1);

	ObjectPoolOwnerUser& operator = (const ObjectPoolOwnerUser& rhs);

	const Block* getBlockPtr() const;
	Block* getBlockPtr();

	void setId(const ObjectPoolOwner* poolOwner, size_t id);
	const Index3DId& getId() const;

protected:
	Index3DId _thisId;

	bool isOwnerBeingDestroyed() const;

private:
	template<class T>
	friend class ObjectPool;
	ObjectPoolOwner* _pPoolOwner = nullptr;

};

template<class T>
class ObjectPool {
public:
	ObjectPool(ObjectPoolOwner* pPoolOwner, bool supportsReverseLookup, size_t objectSegmentSize = 512);
	ObjectPool(ObjectPoolOwner* pPoolOwner, const ObjectPool& src);

	void clear();

	const ObjectPoolOwner* getBlockPtr() const;
	ObjectPoolOwner* getBlockPtr();

	void testReset();
	bool free(const Index3DId& id); // Permanently delete it
	bool removeFromLookup(const Index3DId& id);
	void addToLookup(const Index3DId& id);

	void resize(size_t size);

	size_t getRawIndex(const Index3DId& id) const;
	Index3DId findId(const T& obj) const;
	bool exists(const Index3DId& id) const;

	Index3DId findOrAdd(const T& obj, const Index3DId& id = Index3DId());

	const T* getSize_t(size_t id) const;

	const T* get(const Index3DId& id) const;
	T* get(const Index3DId& id);

	const T* get(const T& obj) const;
	T* get(const T& obj);

	const T& operator[](const Index3DId& id) const;
	T& operator[](const Index3DId& id);

	template<class F>
	void iterateInOrder(F fLambda) const;

	template<class F>
	void iterateInOrder(F fLambda);

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
			const T* pLHS = _owner.getSize_t(lhs);
			const T* pRHS = _owner.getSize_t(rhs);
			if (pLHS && pRHS)
				return *pLHS < *pRHS;
			throw std::runtime_error("comparing deleted object(s)");
		}

		const ObjectPool& _owner;
	};

	Index3DId add(const T& obj, const Index3DId& id = Index3DId());
	size_t storeAndReturnIndex(const T& obj);
	bool calIndices(size_t index, size_t& segNum, size_t& segIdx) const;
	T* getEntry(size_t index);
	const T* getEntry(size_t index) const;

	bool _supportsReverseLookup;
	ObjectPoolOwner* _pPoolOwner;
	const size_t _objectSegmentSize;
	std::vector<size_t> 
		_idToIndexMap,
		_availableIndices;

	using ObjectSegPtr = std::shared_ptr<std::vector<T>>;
	std::vector<ObjectSegPtr> _objectSegs;

	// This oddball indirection was used so that the map of obj to id can use the vector of objects without duplicating the storage.
	// It's ugly, and a bit risky, but it avoids duplicating the storage of vertices and polygons.
	std::map<size_t, size_t, CompareFunctor> _objToIdMap;
	thread_local static const T* _tl_pCompareObj;
};

template<class T>
ObjectPool<T>::ObjectPool(ObjectPoolOwner* pPoolOwner, bool supportsReverseLookup, size_t objectSegmentSize)
	: _pPoolOwner(pPoolOwner)
	, _objToIdMap(CompareFunctor(*this))
	, _objectSegmentSize(objectSegmentSize)
	, _supportsReverseLookup(supportsReverseLookup)
{
	assert(_pPoolOwner);
}

template<class T>
ObjectPool<T>::ObjectPool(ObjectPoolOwner* pPoolOwner, const ObjectPool& src)
	: _pPoolOwner(pPoolOwner)
	, _objToIdMap(CompareFunctor(*this))
	, _objectSegmentSize(src._objectSegmentSize)
	, _supportsReverseLookup(src._supportsReverseLookup)
	, _idToIndexMap(src._idToIndexMap)
	, _availableIndices(src._availableIndices)
	, _objectSegs(src._objectSegs)
{
	iterateInOrder([this](T& obj) {
		obj._pPoolOwner = _pPoolOwner;
	});
}
template<class T>
void ObjectPool<T>::clear()
{
	_idToIndexMap.clear();
	_objToIdMap.clear();
	_objectSegs.clear();
}

template<class T>
inline const ObjectPoolOwner* ObjectPool<T>::getBlockPtr() const
{
	return _pPoolOwner;
}

template<class T>
inline ObjectPoolOwner* ObjectPool<T>::getBlockPtr()
{
	return _pPoolOwner;
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
	// TODO pass the elementId, not the index. Compute the index here to avoid confusion.
	size_t segNum, segIdx;
	if (calIndices(index, segNum, segIdx)) {
		return &((*_objectSegs[segNum])[segIdx]);
	}
	return nullptr;
}

template<class T>
inline const T* ObjectPool<T>::getEntry(size_t index) const
{
	// TODO pass the elementId, not the index. Compute the index here to avoid confusion.
	size_t segNum, segIdx;
	if (calIndices(index, segNum, segIdx)) {
		return &((*_objectSegs[segNum])[segIdx]);
	}
	return nullptr;
}

template<class T>
bool ObjectPool<T>::free(const Index3DId& globalId)
{
	removeFromLookup(globalId);

	size_t id = globalId.elementId();
	if (id >= _idToIndexMap.size())
		return false;

	size_t index = _idToIndexMap[id];

	T* p = getEntry(index);
	if (p) {
		*p = T();
		_idToIndexMap[id] = -1; // Clear this id so it won't be used again
		_availableIndices.push_back(index);

		return true;
	}
	return false;
}

template<class T>
inline bool ObjectPool<T>::removeFromLookup(const Index3DId& id)
{
	if (_supportsReverseLookup) {
		auto iter = _objToIdMap.find(id.elementId());
		if (iter != _objToIdMap.end()) {
			_objToIdMap.erase(iter);
			return true;
		}
	}

	return false;
}

template<class T>
inline void ObjectPool<T>::addToLookup(const Index3DId& id)
{
	if (_supportsReverseLookup) {
		_objToIdMap.insert(std::make_pair(id.elementId(), id.elementId()));
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
			}
			else {
				_objectSegs[i]->resize(size);
				size = 0;
			}
		}
	}
}

template<class T>
inline size_t ObjectPool<T>::getRawIndex(const Index3DId& id) const
{
	return _idToIndexMap[id.elementId()];
}

template<class T>
Index3DId ObjectPool<T>::findId(const T& obj) const
{
	if (_supportsReverseLookup) {
		_tl_pCompareObj = &obj;
		auto iter = _objToIdMap.find(-1);
		if (iter != _objToIdMap.end())
			return Index3DId(_pPoolOwner->getBlockIdx(), iter->second);
	}
	return Index3DId();
}

template<class T>
inline bool ObjectPool<T>::exists(const Index3DId& id) const
{
	bool result(id.elementId() < _idToIndexMap.size() && _idToIndexMap[id.elementId()] != -1);
	if (!result) {
		int dbgBreak = 1;
	}

	return result;
}

template<class T>
Index3DId ObjectPool<T>::findOrAdd(const T& obj, const Index3DId& currentId)
{
	if (_supportsReverseLookup) {
		auto id = findId(obj);
		if (id.isValid())
			return id;
	}
	Index3DId result = add(obj, currentId);

	addToLookup(result);

	return result;
}

template<class T>
size_t ObjectPool<T>::storeAndReturnIndex(const T& obj)
{
	size_t index = -1, segNum = -1, segIdx = -1;
	if (_availableIndices.empty()) {
		if (_objectSegs.empty() || _objectSegs.back()->size() >= _objectSegmentSize) {
			// Reserve the segment size so the array won't resize during use
			_objectSegs.push_back(std::make_shared<std::vector<T>>());
			_objectSegs.back()->reserve(_objectSegmentSize);
		}
		segNum = _objectSegs.size() - 1;

		auto& segData = *_objectSegs.back();

		segIdx = segData.size();
		segData.push_back(obj);
		index = segNum * _objectSegmentSize + segIdx;
	} else {
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
			segData.resize(segIdx + 1);

#ifdef _DEBUG
		size_t segNum1, segIdx1;
		assert(calIndices(index, segNum1, segIdx1) && segNum1 == segNum && segIdx1 == segIdx);
#endif // _DEBUG

		segData[segIdx] = obj;
	}
	return index;
}

template<class T>
Index3DId ObjectPool<T>::add(const T& obj, const Index3DId& currentId)
{
	size_t result = -1, index = -1, segNum = -1, segIdx = -1;
	if (currentId.isValid()) {
		Index3DId dbgId(0, 7, 5, 1);
		if (currentId == dbgId) {
			int dbgBreak = 1;
		}
		// This has never been used or tested. Probably obsolete and needs to be replaced.
		result = currentId.elementId();
		index = storeAndReturnIndex(obj);

		if (result >= _idToIndexMap.size())
			_idToIndexMap.resize(result + 1, -1);
		_idToIndexMap[result] = index;
	} else {
		result = _idToIndexMap.size();
		index = storeAndReturnIndex(obj);
		_idToIndexMap.push_back(index);
	}

	assert(getEntry(_idToIndexMap[result]));
	getEntry(_idToIndexMap[result])->setId(_pPoolOwner, result);

	return Index3DId(_pPoolOwner->getBlockIdx(), result);
}

template<class T>
const T* ObjectPool<T>::getSize_t(size_t id) const
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
const T* ObjectPool<T>::get(const Index3DId& id) const
{
	if (!id.isValid())
		return _tl_pCompareObj;
	else if ((id.blockIdx() == _pPoolOwner->getBlockIdx()) && id.elementId() < _idToIndexMap.size()) {
		size_t index = _idToIndexMap[id.elementId()];
		return getEntry(index);
	}
	return nullptr;
}

template<class T>
T* ObjectPool<T>::get(const Index3DId& id)
{
	if (!id.isValid())
		return (T*)_tl_pCompareObj;
	else if ((id.blockIdx() == _pPoolOwner->getBlockIdx()) && id.elementId() < _idToIndexMap.size()) {
		size_t index = _idToIndexMap[id.elementId()];
		return getEntry(index);
	}
	return nullptr;
}

template<class T>
const T* ObjectPool<T>::get(const T& obj) const
{
	auto id = findId(obj);
	return get(id);
}

template<class T>
T* ObjectPool<T>::get(const T& obj)
{
	auto id = findId(obj);
	return get(id);
}

template<class T>
const T& ObjectPool<T>::operator[](const Index3DId& id) const
{
	assert(id.blockIdx() == _pPoolOwner->getBlockIdx());
	return *getEntry(_idToIndexMap[id.elementId()]);
}

template<class T>
T& ObjectPool<T>::operator[](const Index3DId& id)
{
	assert(id.blockIdx() == _pPoolOwner->getBlockIdx());
	return *getEntry(_idToIndexMap[id.elementId()]);
}

template<class T>
template<class F>
void ObjectPool<T>::iterateInOrder(F fLambda) const
{
	auto temp = _idToIndexMap; // Don't modify this while we're in the loop
	for (size_t id = 0; id < temp.size(); id++) {
		size_t index = temp[id];
		if (index != -1) {
			auto p = getEntry(index);
			fLambda(*p);
		}
	}
}

template<class T>
template<class F>
void ObjectPool<T>::iterateInOrder(F fLambda)
{
	auto temp = _idToIndexMap; // Don't modify this while we're in the loop
	for (size_t id = 0; id < temp.size(); id++) {
		size_t index = temp[id];
		if (index != -1) {
			auto p = getEntry(index);
			fLambda(*p);
		}
	}
}

template<class T>
inline bool ObjectPool<T>::empty() const
{
	return size() == 0;
}

template<class T>
inline size_t ObjectPool<T>::size() const
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
