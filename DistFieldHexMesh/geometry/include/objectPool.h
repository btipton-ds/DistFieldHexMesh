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
#include <fastBisectionSet.h>
#include <fastBisectionMap.h>
#include <patient_lock_guard.h>
#include <MultiCoreUtil.h>

namespace DFHM {

class Block;
class PolyMesh;
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

	virtual const PolyMesh* getPolyMeshPtr() const;
	virtual PolyMesh* getPolyMeshPtr();

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
	ObjectPoolOwnerUser(const ObjectPoolOwner* poolOwner);
	virtual ~ObjectPoolOwnerUser();
	virtual void clear();

	ObjectPoolOwnerUser& operator = (const ObjectPoolOwnerUser& rhs);

	void setPoolOwner(ObjectPoolOwner* pPoolOwner);

	const Block* getOurBlockPtr() const;
	Block* getOurBlockPtr();

	const PolyMesh* getPolyMeshPtr() const;
	PolyMesh* getPolyMeshPtr();

	const Block* getBlockPtr() const;
	Block* getBlockPtr();

	const Block* getOwnerBlockPtr(const Index3D& idx) const;
	Block* getOwnerBlockPtr(const Index3D& idx);

	void setOwner(const ObjectPoolOwner* poolOwner, const Index3DId& id);

	virtual const Index3DId& getId() const = 0;
	virtual void postAddToPoolActions();
	virtual void remapId(const std::vector<size_t>& idRemap, const Index3D& srcDims) = 0;

protected:
	virtual void setId(const Index3DId& id) = 0;

	bool isOwnerBeingDestroyed() const;

	void remap(const std::vector<size_t>& idRemap, const Index3D& srcDims, Index3DId& val);
	void remap(const std::vector<size_t>& idRemap, const Index3D& srcDims, FastBisectionSet<Index3DId>& vals);
	void remap(const std::vector<size_t>& idRemap, const Index3D& srcDims, MTC::set<Index3DId>& vals);
	void remap(const std::vector<size_t>& idRemap, const Index3D& srcDims, MTC::vector<Index3DId>& vals);
	template<class T>
	void remap(const std::vector<size_t>& idRemap, const Index3D& srcDims, MTC::map<T, Index3DId>& vals);

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
	void addToLookup(const T& obj);

	void resize(size_t size);
	size_t numBytes() const;

	Index3DId findId(const T& obj) const;
	bool exists(const Index3DId& id) const;

	Index3DId findOrAdd(const T& obj, const Index3DId& id = Index3DId());

	const T* getObjPtrByElementIndex(size_t id) const;
	T* getObjPtrByElementIndex(size_t id);

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

	template<class F>
	void iterateInOrder(MultiCore::ThreadPool& tp, size_t numThreads, F fLambda);

	bool empty() const;
	size_t size() const;
	size_t getNumAllocated() const;
	size_t getNumAvailable() const;
	size_t getNumUnloaded() const;

	void write(std::ostream& out) const;
	void setSupportsReverseLookup(bool val);
	void read(std::istream& in);

private:
	/*
	ObjIndex is a shallow wrapper around size_t to reduce confusion of the two indices.
	This forces the compiler to assist in making sure the indices to get swapped accidentally.
	There is no ObjIndex(size_t src) constructor to prevent automatic promotion from size_t to ObjIndex when passed as a parameter.
	This makes it extremely difficult to accidentally pass a size_t where an ObjIndex is required.
	*/
	class ObjIndex {
	public:
		ObjIndex() = default;
		ObjIndex(const ObjIndex& src);

		operator size_t() const;

		ObjIndex& operator = (size_t rhs);
		ObjIndex& operator = (const ObjIndex& rhs);
	private:
		size_t _idx = -1;
	};


	struct CompareFunctor
	{
		inline CompareFunctor(const ObjectPool& owner)
		: _owner(owner)
		{
		}

		inline bool operator () (const FBMPair<ObjIndex,size_t>& lhObjIdx, const FBMPair<ObjIndex, size_t>& rhObjIdx) const
		{
			const T* pLHS = _owner.getEntryFromObjIndex(lhObjIdx.first);
			const T* pRHS = _owner.getEntryFromObjIndex(rhObjIdx.first);
			if (pLHS && pRHS)
				return *pLHS < *pRHS;

			assert(!"comparing deleted object(s)");
			std::string msg = std::string(__FILE__) + ":" + std::to_string(__LINE__) + std::string(" comparing deleted object(s)");
			throw std::runtime_error(msg);
		}

		const ObjectPool& _owner;
	};

	T* add(const T& obj, const Index3DId& id = Index3DId());
	size_t findElementIndexObj(const T& obj) const;
	ObjIndex storeAndReturnObjIndex(const T& obj);
	bool calObjSegIndices(const ObjIndex& objIdx, size_t& segNum, size_t& segIdx) const;

	T* getEntryFromObjIndex(const ObjIndex& objIdx);
	const T* getEntryFromObjIndex(const ObjIndex& objIdx) const;

	T* getEntryFromElementIndex(size_t elementIndex);
	const T* getEntryFromElementIndex(size_t elementIndex) const;

	bool _supportsReverseLookup;
	ObjectPoolOwner* _pPoolOwner;
	const size_t _objectSegmentSize; // May want to tune this so the first segment only holds 1 cell since most blocks only contain one cell. We're allocating a lot of unused memory in sparse blocks
	std::vector<ObjIndex> 
		_elementIndexToObjIndexMap,
		_availableObjIndices;

	using ObjectSegPtr = std::shared_ptr<std::vector<T>>;
	std::vector<ObjectSegPtr> _objSegmentPtrs;

	// This oddball indirection was used so that the map of obj to id can use the vector of objects without duplicating the storage.
	// It's ugly, and a bit risky, but it avoids duplicating the storage of vertices and polygons.
#if OBJECT_POOL_USE_STD_MAP
	std::map<ObjIndex, size_t, CompareFunctor> _objToElementIndexMap;
#else
	FastBisectionMap_with_comp<ObjIndex, size_t, CompareFunctor> _objToElementIndexMap;
#endif
	// TODO, may want to change this to a sorted array with bisection lookup for space savings
															// Attempted this on 2/5/25 and gave up after several hours using FastBisectionMap. It's trickier than it looks.
															// Probably due to _objToElementIndexMap.find not reporting missing entries properly.
															// Can't use a simple array because of accumulated dead ids at the head.
	thread_local static T* _tl_pCompareObj;
};

template<class T>
ObjectPool<T>::ObjectPool(ObjectPoolOwner* pPoolOwner, bool supportsReverseLookup, size_t objectSegmentSize)
	: _pPoolOwner(pPoolOwner)
	, _objToElementIndexMap(CompareFunctor(*this))
	, _objectSegmentSize(objectSegmentSize)
	, _supportsReverseLookup(supportsReverseLookup)
{
	assert(_pPoolOwner);
}

template<class T>
ObjectPool<T>::ObjectPool(ObjectPoolOwner* pPoolOwner, const ObjectPool& src)
	: _pPoolOwner(pPoolOwner)
	, _objToElementIndexMap(CompareFunctor(*this))
	, _objectSegmentSize(src._objectSegmentSize)
	, _supportsReverseLookup(src._supportsReverseLookup)
	, _elementIndexToObjIndexMap(src._elementIndexToObjIndexMap)
	, _availableObjIndices(src._availableObjIndices)
{
	_objSegmentPtrs.reserve(src._objSegmentPtrs.size());
	for (size_t i = 0; i < src._objSegmentPtrs.size(); i++) {
		const auto& pSrcVec = src._objSegmentPtrs[i];
		const auto& srcVec = *pSrcVec;
		_objSegmentPtrs.push_back(std::make_shared<std::vector<T>>(srcVec));
	}
	iterateInOrder([this](const Index3DId& id, T& obj) {
		obj._pPoolOwner = _pPoolOwner;
	});
}

template<class T>
ObjectPool<T>::~ObjectPool()
{
	for (size_t i = 0; i < _objSegmentPtrs.size(); i++) {
		_objSegmentPtrs[i] = nullptr;
	}
}

template<class T>
void ObjectPool<T>::clear()
{
	_elementIndexToObjIndexMap.clear();
	_availableObjIndices.clear();
	_objToElementIndexMap.clear();
	_objSegmentPtrs.clear();
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
inline bool ObjectPool<T>::calObjSegIndices(const ObjIndex& objIdx, size_t& segNum, size_t& segIdx) const
{
	size_t idx = objIdx;
	assert(idx != -1);
	segNum = idx / _objectSegmentSize;
	segIdx = idx % _objectSegmentSize;
	return segNum < _objSegmentPtrs.size() && segIdx < _objSegmentPtrs[segNum]->size();
}

template<class T>
T* ObjectPool<T>::getEntryFromElementIndex(size_t elementIndex)
{
	T* p = nullptr;
	if (elementIndex < _elementIndexToObjIndexMap.size()) {
		const ObjIndex& objIndex = _elementIndexToObjIndexMap[elementIndex];
		return getEntryFromObjIndex(objIndex);
	}
	return p;
}

template<class T>
const T* ObjectPool<T>::getEntryFromElementIndex(size_t elementIndex) const
{
	T* p = nullptr;
	if (elementIndex < _elementIndexToObjIndexMap.size()) {
		const ObjIndex& objIndex = _elementIndexToObjIndexMap[elementIndex];
		return getEntryFromObjIndex(objIndex);
	}
	return p;
}

template<class T>
inline T* ObjectPool<T>::getEntryFromObjIndex(const ObjIndex& objIdx)
{
	T* p = nullptr;
	if (objIdx != -1) {
		size_t segNum, segIdx;
		if (calObjSegIndices(objIdx, segNum, segIdx)) {
//			assert(segNum < _objSegmentPtrs.size() && segIdx < _objSegmentPtrs[segNum]->size());
			p = &((*_objSegmentPtrs[segNum])[segIdx]);
		}
		assert(p);
	}
	return p;
}

template<class T>
inline const T* ObjectPool<T>::getEntryFromObjIndex(const ObjIndex& objIdx) const
{
	const T* p = nullptr;
	if (objIdx == -1)
		return _tl_pCompareObj;
	else {
		size_t segNum, segIdx;
		if (calObjSegIndices(objIdx, segNum, segIdx)) {
//			assert(segNum < _objSegmentPtrs.size() && segIdx < _objSegmentPtrs[segNum]->size());
			p = &((*_objSegmentPtrs[segNum])[segIdx]);
		}

		assert(p);
	}
	return p;
}

template<class T>
bool ObjectPool<T>::free(const Index3DId& globalId)
{
	if (!exists(globalId))
		return false;

	T* p = getEntryFromElementIndex(globalId.elementId());
	if (p) {
		removeFromLookup(globalId);

		*p = {};
		const ObjIndex objIdx = _elementIndexToObjIndexMap[globalId.elementId()];
		_elementIndexToObjIndexMap[globalId.elementId()] = ObjIndex(); // Clear this id so it won't be used again
		_availableObjIndices.push_back(objIdx);

		return true;
	}
	return false;
}

template<class T>
inline bool ObjectPool<T>::removeFromLookup(const Index3DId& id)
{
	if (_supportsReverseLookup) {
		const ObjIndex& objIdx = _elementIndexToObjIndexMap[id.elementId()];
		const auto p = getEntryFromObjIndex(objIdx);
		assert(p);
		if (p) {
			_objToElementIndexMap.erase(objIdx);
			assert(!findId(*p).isValid());
		}
	}

	return false;
}

template<class T>
inline void ObjectPool<T>::addToLookup(const T& obj)
{
	if (_supportsReverseLookup) {
		auto id = obj.getId();
//		assert(id.elementId() < _elementIndexToObjIndexMap.size());
		const ObjIndex& objIdx = _elementIndexToObjIndexMap[id.elementId()];
#if OBJECT_POOL_USE_STD_MAP
		_objToElementIndexMap.insert(make_pair(objIdx, id.elementId()));
#else
		_objToElementIndexMap.insert(objIdx, id.elementId());
#endif
		auto testId = findId(obj);
		assert(testId == id); // make sure we can find ourself
	}

}

template<class T>
void ObjectPool<T>::resize(size_t size)
{
	if (size > _elementIndexToObjIndexMap.size()) {
		_elementIndexToObjIndexMap.resize(size);
		size_t numSegs = size / _objectSegmentSize + 1;
		_objSegmentPtrs.resize(numSegs);
		for (size_t id = 0; id < size; id++) {
			_elementIndexToObjIndexMap[id] = _objSegmentPtrs.size();
		}
		size_t remaining = size;
		for (size_t i = 0; i < numSegs; i++) {
			// Reserve the segment size so the array won't resize during use
			_objSegmentPtrs[i] = std::make_shared<std::vector<T>>();
			_objSegmentPtrs[i]->reserve(_objectSegmentSize);
			if (size > _objectSegmentSize) {
				_objSegmentPtrs[i]->resize(_objectSegmentSize);
				size -= _objectSegmentSize;
			}
			else {
				_objSegmentPtrs[i]->resize(size);
				size = 0;
			}
		}
	}
}

template<class T>
size_t ObjectPool<T>::numBytes() const
{
	size_t result = sizeof(ObjectPool<T>);
	result += _elementIndexToObjIndexMap.size() * sizeof(size_t);
	result += _availableObjIndices.size() * sizeof(size_t);
	for (const auto& pSeg : _objSegmentPtrs) {

		result += pSeg->size() * sizeof(T);
	}

	result += _objToElementIndexMap.size() * sizeof(std::pair<size_t, size_t>);

	return result;
}

template<class T>
size_t ObjectPool<T>::findElementIndexObj(const T& obj) const 
{
	size_t result = -1;
	if (_supportsReverseLookup) {
		_tl_pCompareObj = (T*) &obj;

		auto pPriorOwner = _tl_pCompareObj->_pPoolOwner;
		_tl_pCompareObj->_pPoolOwner = _pPoolOwner;

		auto iter = _objToElementIndexMap.find(ObjIndex());
		if (iter != _objToElementIndexMap.end()) {
			result = iter->second;
		}

		_tl_pCompareObj->_pPoolOwner = pPriorOwner;
	}
	return result;
}

template<class T>
Index3DId ObjectPool<T>::findId(const T& obj) const
{
	size_t elementIdx = findElementIndexObj(obj);
	if (elementIdx < _elementIndexToObjIndexMap.size()) {
		return Index3DId(_pPoolOwner->getBlockIdx(), elementIdx);
	}
	return Index3DId();
}

template<class T>
inline bool ObjectPool<T>::exists(const Index3DId& id) const
{
	bool result(id.elementId() < _elementIndexToObjIndexMap.size() && _elementIndexToObjIndexMap[id.elementId()] != -1);

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
	T* pObj = add(obj, currentId);

	addToLookup(*pObj);

	return pObj->getId();
}

template<class T>
ObjectPool<T>::ObjIndex ObjectPool<T>::storeAndReturnObjIndex(const T& obj)
{
	size_t segNum = -1, segIdx = -1;
	ObjIndex objIdx;

	if (_availableObjIndices.empty()) {
		if (_objSegmentPtrs.empty() || _objSegmentPtrs.back()->size() >= _objectSegmentSize) {
			_objSegmentPtrs.push_back(std::make_shared<std::vector<T>>());
		}
		segNum = _objSegmentPtrs.size() - 1;

		auto& segData = *_objSegmentPtrs.back();
		if (segData.capacity() == 0)
			segData.reserve(_objectSegmentSize);

		segIdx = segData.size();
		segData.push_back(obj);
		segData.back().setPoolOwner(_pPoolOwner);
		objIdx = segNum * _objectSegmentSize + segIdx;
	} else {
		objIdx = _availableObjIndices.back();
		_availableObjIndices.pop_back();

		calObjSegIndices(objIdx, segNum, segIdx);
		if (segNum >= _objSegmentPtrs.size()) {
			_objSegmentPtrs.push_back(std::make_shared<std::vector<T>>());
		}
		auto& segData = *_objSegmentPtrs[segNum];
		if (segData.capacity() == 0)
			segData.reserve(_objectSegmentSize);

		if (segIdx >= segData.size())
			segData.resize(segIdx + 1);

		segData[segIdx] = obj;

		// Set the stored copy's owner to our pool owner
		segData[segIdx].setPoolOwner(_pPoolOwner);
	}
	return objIdx;
}

template<class T>
T* ObjectPool<T>::add(const T& obj, const Index3DId& currentId)
{
	size_t result = -1, segNum = -1, segIdx = -1;
	ObjIndex objIdx;
	if (currentId.isValid()) {
#if DEBUG_BREAKS && defined(_DEBUG)
		Index3DId dbgId(0, 7, 5, 1);
		if (currentId == dbgId) {
			int dbgBreak = 1;
		}
#endif
		// This has never been used or tested. Probably obsolete and needs to be replaced.
		result = currentId.elementId();
		objIdx = storeAndReturnObjIndex(obj);

		if (result >= _elementIndexToObjIndexMap.size())
			_elementIndexToObjIndexMap.resize(result + 1);
		_elementIndexToObjIndexMap[result] = objIdx;
	} else {
		result = _elementIndexToObjIndexMap.size();
		objIdx = storeAndReturnObjIndex(obj);
		_elementIndexToObjIndexMap.push_back(objIdx);
	}

	auto pNewEntry = getEntryFromObjIndex(_elementIndexToObjIndexMap[result]);
	assert(pNewEntry);
	Index3DId newId(_pPoolOwner->getBlockIdx(), result);
	pNewEntry->setOwner(_pPoolOwner, newId);
	pNewEntry->postAddToPoolActions();

	return pNewEntry;
}

template<class T>
const T* ObjectPool<T>::getObjPtrByElementIndex(size_t id) const
{
	if (id < _elementIndexToObjIndexMap.size()) {
		const ObjIndex& objIdx = _elementIndexToObjIndexMap[id];
		return getEntryFromObjIndex(objIdx);
	}
	return nullptr;
}

template<class T>
T* ObjectPool<T>::getObjPtrByElementIndex(size_t id)
{
	if (id < _elementIndexToObjIndexMap.size()) {
		const auto& objIdx = _elementIndexToObjIndexMap[id];
		return getEntryFromObjIndex(objIdx);
	}
	return nullptr;
}

template<class T>
inline const T* ObjectPool<T>::get(const Index3DId& id) const
{
	if (!id.isValid())
		return (T*)_tl_pCompareObj;
	else
		return getEntryFromElementIndex(id.elementId());
}

template<class T>
T* ObjectPool<T>::get(const Index3DId& id)
{
	if (!id.isValid())
		return (T*)_tl_pCompareObj;
	else
		return getEntryFromElementIndex(id.elementId());
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
	return *getEntryFromObjIndex(_elementIndexToObjIndexMap[id.elementId()]);
}

template<class T>
T& ObjectPool<T>::operator[](const Index3DId& id)
{
	assert(id.blockIdx() == _pPoolOwner->getBlockIdx());
	return *getEntryFromObjIndex(_elementIndexToObjIndexMap[id.elementId()]);
}

template<class T>
template<class F>
void ObjectPool<T>::iterateInOrder(F fLambda) const
{
	auto blkIdx = _pPoolOwner->getBlockIdx();
	size_t num = _elementIndexToObjIndexMap.size(); // _elementIndexToObjIndexMap can't decrease in size
	for (size_t idx = 0; idx < num; idx++) {
		const ObjIndex& objIdx = _elementIndexToObjIndexMap[idx];
		if (objIdx != -1) {
			auto p = getEntryFromObjIndex(objIdx);
			if (p) {
				Index3DId id(blkIdx, idx);
				fLambda(id, *p);
			}
		}
	}
}

template<class T>
template<class F>
void ObjectPool<T>::iterateInOrder(F fLambda)
{
	auto blkIdx = _pPoolOwner->getBlockIdx();
	size_t num = _elementIndexToObjIndexMap.size(); // _elementIndexToObjIndexMap can't decrease in size
	for (size_t idx = 0; idx < num; idx++) {
		auto objIdx = _elementIndexToObjIndexMap[idx];
		if (objIdx != -1) {
			auto p = getEntryFromObjIndex(objIdx);
			if (p) {
				Index3DId id(blkIdx, idx);
				fLambda(id, *p);
			}
		}
	}
}

template<class T>
template<class F>
void ObjectPool<T>::iterateInOrder(MultiCore::ThreadPool& tp, size_t numThreads, F fLambda)
{
	auto blkIdx = _pPoolOwner->getBlockIdx();
	size_t num = _elementIndexToObjIndexMap.size(); // _elementIndexToObjIndexMap can't decrease in size
	for (size_t idx = 0; idx < num; idx++) {
		const ObjIndex& objIdx = _elementIndexToObjIndexMap[idx];
		if (objIdx != -1) {
			auto p = getEntryFromObjIndex(objIdx);
			if (p) {
				Index3DId id(blkIdx, idx);
				fLambda(id, *p);
			}
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
	for (const auto& p : _objSegmentPtrs) {
		if (p)
			size += p->size();
	}
	return size - _availableObjIndices.size();
}

template<class T>
size_t ObjectPool<T>::getNumAvailable() const
{
	return _availableObjIndices.size();
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
void ObjectPool<T>::setSupportsReverseLookup(bool val)
{
	_supportsReverseLookup = val;
	_objToElementIndexMap.clear();
	if (_supportsReverseLookup) {
		iterateInOrder([this](const Index3DId& id, T& obj) {
			addToLookup(obj);
		});
	}
}

template<class T>
void ObjectPool<T>::read(std::istream& in)
{
	uint8_t version = -1;
	in.read((char*)&version, sizeof(version));

	in.read((char*)&_objectSegmentSize, sizeof(_objectSegmentSize));

	size_t numObjs = -1;
	in.read((char*)&numObjs, sizeof(numObjs));

	for (size_t i = 0; i < numObjs; i++) {
		Index3DId currentId;
		currentId.read(in);
		T* pObj = add(T(), currentId);
		assert(pObj->getId() == currentId);
		pObj->read(in);
		addToLookup(*pObj);
	}
}

template<class T>
inline ObjectPool<T>::ObjIndex::ObjIndex(const ObjIndex& src)
	:_idx (src._idx)
{
}

template<class T>
inline ObjectPool<T>::ObjIndex::operator size_t() const
{
	return _idx;
}

template<class T>
inline ObjectPool<T>::ObjIndex& ObjectPool<T>::ObjIndex::operator = (size_t rhs)
{
	_idx = rhs;
	return *this;
}

template<class T>
inline ObjectPool<T>::ObjIndex& ObjectPool<T>::ObjIndex::operator = (const ObjIndex& rhs) {
	_idx = rhs._idx;
	return *this;
}

}
