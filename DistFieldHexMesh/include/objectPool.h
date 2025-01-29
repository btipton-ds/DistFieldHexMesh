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
#include <tm_ioUtil.h>
#include <index3D.h>
#include <pool_map.h>
#include <patient_lock_guard.h>

namespace DFHM {

class Block;
class Volume;
class Logger;

// Have to store the pointer so we can call block functions. Just the block index is not enough
class ObjectPoolOwner {
public:
	static void setThreadBlockIdx(const Index3D& blockIdx);
	const Index3D& getThreadBlockIdx() const;

	virtual const Index3D& getBlockIdx() const = 0;

	virtual Volume* getVolume() = 0;
	virtual const Volume* getVolume() const = 0;

	virtual const Block* getOwner(const Index3D& blockIdx) const = 0;
	virtual Block* getOwner(const Index3D& blockIdx) = 0;

	std::shared_ptr<Logger> getLogger() const;
	std::string getLoggerNumericCode() const;
	static std::string getLoggerNumericCode(const Index3D& id);
	static std::string getLoggerNumericCode(const Index3DId& id);

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
	virtual ~ObjectPoolOwnerUser();
	virtual void clear();

	ObjectPoolOwnerUser& operator = (const ObjectPoolOwnerUser& rhs);

	void setPoolOwner(ObjectPoolOwner* pPoolOwner);
	const Block* getOurBlockPtr() const;
	Block* getOurBlockPtr();

	const Block* getBlockPtr() const;
	Block* getBlockPtr();

	const Block* getOwnerBlockPtr(const Index3D& idx) const;
	Block* getOwnerBlockPtr(const Index3D& idx);

	void setId(const ObjectPoolOwner* poolOwner, size_t id);
	const Index3DId& getId() const;

	virtual void remapId(const std::vector<size_t>& idRemap, const Index3D& srcDims);
	virtual bool verifyIndices(const Index3D& idx) const;

protected:
	bool isOwnerBeingDestroyed() const;

	void remap(const std::vector<size_t>& idRemap, const Index3D& srcDims, MTC::set<Index3DId>& vals);
	void remap(const std::vector<size_t>& idRemap, const Index3D& srcDims, MTC::vector<Index3DId>& vals);
	template<class T>
	void remap(const std::vector<size_t>& idRemap, const Index3D& srcDims, MTC::map<T, Index3DId>& vals);

	Index3DId _thisId;

private:
	template<class T>
	friend class ObjectPool;
	ObjectPoolOwner* _pPoolOwner = nullptr;
};

template<class T>
class ObjectPool {
public:
	ObjectPool(ObjectPoolOwner* pPoolOwner, bool supportsReverseLookup, size_t objectSegmentSize = 16);
	ObjectPool(ObjectPoolOwner* pPoolOwner, const ObjectPool& src);
	~ObjectPool();

	void clear();

	const ObjectPoolOwner* getBlockPtr() const;
	ObjectPoolOwner* getBlockPtr();

	void testReset();
	bool free(const Index3DId& id); // Permanently delete it
	bool removeFromLookup(const Index3DId& id);
	void addToLookup(const Index3DId& id);

	void resize(size_t size);
	size_t numBytes() const;

	size_t getRawIndex(const Index3DId& id) const;
	Index3DId findId(const T& obj) const;
	bool exists(const Index3DId& id) const;

	Index3DId findOrAdd(const T& obj, const Index3DId& id = Index3DId());

	const T* getByElementIndex(size_t id) const;

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

	void write(std::ostream& out) const;
	void read(std::istream& in);

private:
	struct CompareFunctor
	{
		inline CompareFunctor(const ObjectPool& owner)
		: _owner(owner)
		{
		}

		inline bool operator () (size_t lhs, size_t rhs) const
		{
			const T* pLHS = _owner.getByElementIndex(lhs);
			const T* pRHS = _owner.getByElementIndex(rhs);
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
	const size_t _objectSegmentSize; // May want to tune this so the first segment only holds 1 cell since most blocks only contain one cell. We're allocating a lot of unused memory in sparse blocks
	std::vector<size_t> 
		_idToIndexMap,
		_availableIndices;

	using ObjectSegPtr = std::shared_ptr<std::vector<T>>;
	std::vector<ObjectSegPtr> _objectSegs;

	// This oddball indirection was used so that the map of obj to id can use the vector of objects without duplicating the storage.
	// It's ugly, and a bit risky, but it avoids duplicating the storage of vertices and polygons.
	std::map<size_t, size_t, CompareFunctor> _objToIdMap; // TODO, may want to change this to a sorted array with bisection lookup for space savings
															// Can't use a simple array because of accumulated dead ids at the head.
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
{
	_objectSegs.reserve(src._objectSegs.size());
	for (size_t i = 0; i < src._objectSegs.size(); i++) {
		const auto& pSrcVec = src._objectSegs[i];
		const auto& srcVec = *pSrcVec;
		_objectSegs.push_back(std::make_shared<std::vector<T>>(srcVec));
	}
	iterateInOrder([this](const Index3DId& id, T& obj) {
		obj._pPoolOwner = _pPoolOwner;
	});
}

template<class T>
ObjectPool<T>::~ObjectPool()
{
	for (size_t i = 0; i < _objectSegs.size(); i++) {
		_objectSegs[i] = nullptr;
	}
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
	return segNum < _objectSegs.size() && segIdx < _objectSegs[segNum]->size();
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
	if (!exists(globalId))
		return false;

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
size_t ObjectPool<T>::numBytes() const
{
	size_t result = sizeof(ObjectPool<T>);
	result += _idToIndexMap.size() * sizeof(size_t);
	result += _availableIndices.size() * sizeof(size_t);
	for (const auto& pSeg : _objectSegs) {

		result += pSeg->size() * sizeof(T);
	}

	result += _objToIdMap.size() * sizeof(std::pair<size_t, size_t>);

	return result;
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

#if DEBUG_BREAKS && defined(_DEBUG)
	if (!result) {
		int dbgBreak = 1;
	}
#endif // DEBUG_BREAKS


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
			_objectSegs.push_back(std::make_shared<std::vector<T>>());
		}
		segNum = _objectSegs.size() - 1;

		auto& segData = *_objectSegs.back();
		if (segData.capacity() == 0)
			segData.reserve(_objectSegmentSize);

		segIdx = segData.size();
		segData.push_back(obj);
		segData.back().setPoolOwner(_pPoolOwner);
		index = segNum * _objectSegmentSize + segIdx;
	} else {
		index = _availableIndices.back();
		_availableIndices.pop_back();

		calIndices(index, segNum, segIdx);
		if (segNum >= _objectSegs.size()) {
			_objectSegs.push_back(std::make_shared<std::vector<T>>());
		}
		auto& segData = *_objectSegs[segNum];
		if (segData.capacity() == 0)
			segData.reserve(_objectSegmentSize);

		if (segIdx >= segData.size())
			segData.resize(segIdx + 1);

		segData[segIdx] = obj;

		// Set the stored copy's owner to our pool owner
		segData[segIdx].setPoolOwner(_pPoolOwner);
	}
	return index;
}

template<class T>
Index3DId ObjectPool<T>::add(const T& obj, const Index3DId& currentId)
{
	size_t result = -1, index = -1, segNum = -1, segIdx = -1;
	if (currentId.isValid()) {
#if DEBUG_BREAKS && defined(_DEBUG)
		Index3DId dbgId(0, 7, 5, 1);
		if (currentId == dbgId) {
			int dbgBreak = 1;
		}
#endif
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
const T* ObjectPool<T>::getByElementIndex(size_t id) const
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
	auto blkIdx = _pPoolOwner->getBlockIdx();
	size_t num = _idToIndexMap.size(); // _idToIndexMap can't decrease in size
	for (size_t idx = 0; idx < num; idx++) {
		size_t index = _idToIndexMap[idx];
		if (index != -1) {
			Index3DId id(blkIdx, idx);
			auto p = getEntry(index);
			fLambda(id, *p);
		}
	}
}

template<class T>
template<class F>
void ObjectPool<T>::iterateInOrder(F fLambda)
{
	auto blkIdx = _pPoolOwner->getBlockIdx();
	size_t num = _idToIndexMap.size(); // _idToIndexMap can't decrease in size
	for (size_t idx = 0; idx < num; idx++) {
		size_t index = _idToIndexMap[idx];
		if (index != -1) {
			Index3DId id(blkIdx, idx);
			auto p = getEntry(index);
			fLambda(id , *p);
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

template<class T>
void ObjectPool<T>::write(std::ostream& out) const
{
	uint8_t version = 0;
	out.write((char*)&version, sizeof(version));

	out.write((char*)&_supportsReverseLookup, sizeof(_supportsReverseLookup));
	out.write((char*)&_objectSegmentSize, sizeof(_objectSegmentSize));

	size_t numObjs = 0;
	iterateInOrder([&numObjs](const Index3DId& id, const T& obj) {
		numObjs++;
	});
	out.write((char*)&numObjs, sizeof(numObjs));

	iterateInOrder([&out](const Index3DId& id, const T& obj) {
		id.write(out);
		obj.write(out);
	});
}

template<class T>
void ObjectPool<T>::read(std::istream& in)
{
	uint8_t version = -1;
	in.read((char*)&version, sizeof(version));

	in.read((char*)&_supportsReverseLookup, sizeof(_supportsReverseLookup));
	in.read((char*)&_objectSegmentSize, sizeof(_objectSegmentSize));

	size_t numObjs = -1;
	in.read((char*)&numObjs, sizeof(numObjs));

	for (size_t i = 0; i < numObjs; i++) {
		Index3DId currentId;
		currentId.read(in);
		Index3DId result = add(T(), currentId);
		assert(result == currentId);
		auto& obj = operator[](currentId);
		obj.read(in);
	}

	if (_supportsReverseLookup) {
		iterateInOrder([this](const Index3DId& id, const T& obj) {
			addToLookup(id);
		});
	}
}

}
