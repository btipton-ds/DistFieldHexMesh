#pragma once

#include <vector>
#include <map>
#include <mutex>
#include <stdexcept>
#include <tm_vector3.h>
#include <Index3D.h>
#include <patient_lock_guard.h>

namespace DFHM {

class Block;

// Have to store the pointer so we can call block functions. Just the block index is not enough
class ObjectPoolOwner {
public:
	class ScopedGranularLock {
	public:
		ScopedGranularLock(ObjectPoolOwner& self, bool val);
		~ScopedGranularLock();
	private:
		ObjectPoolOwner& _self;
		bool _wasLocked;
	};
	virtual const Index3D& getBlockIdx() const = 0;

	bool isGranularLocking() const;

private:
	friend ObjectPoolOwner;

	// TODO granular locking worked in the old code for block creation, but occasionally has race conditions now.
	void setGranularLocking(bool val);

	bool _isGranularLocking = false;
};

class ObjectPoolOwnerUser {
public:
	ObjectPoolOwnerUser() = default;
	ObjectPoolOwnerUser(const ObjectPoolOwnerUser& src);
	ObjectPoolOwnerUser(const ObjectPoolOwner* poolOwner, size_t id = -1);

	ObjectPoolOwnerUser& operator = (const ObjectPoolOwnerUser& rhs);

	Block* getBlockPtr();
	const Block* getBlockPtr() const;
	void setId(const ObjectPoolOwner* poolOwner, size_t id);

	MutexType& getMutex() const;
protected:
	Index3DId _thisId;

private:
	ObjectPoolOwner* _pPoolOwner = nullptr;
	mutable MutexType _mutex;
};

template<class T>
class ObjectPool {
public:
	ObjectPool(ObjectPoolOwner* pPoolOwner, bool supportsReverseLookup, size_t objectSegmentSize = 512);

	const ObjectPoolOwner* getBlockPtr() const;
	ObjectPoolOwner* getBlockPtr();

	void testReset();
	bool free(size_t id); // Permanently delete it
	bool removeFromLookup(const Index3DId& id);
	void addToLookup(const Index3DId& id);

	void resize(size_t size);

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

	mutable MutexType _mutex;
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
inline ObjectPool<T>::ObjectPool(ObjectPoolOwner* pPoolOwner, bool supportsReverseLookup, size_t objectSegmentSize)
	: _pPoolOwner(pPoolOwner)
	, _objToIdMap(CompareFunctor(*this))
	, _objectSegmentSize(objectSegmentSize)
{
	_supportsReverseLookup = supportsReverseLookup;
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
	patient_lock_guard g(_mutex, _pPoolOwner->isGranularLocking());

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
inline bool ObjectPool<T>::removeFromLookup(const Index3DId& id)
{
	if (_supportsReverseLookup) {
		patient_lock_guard g(_mutex, _pPoolOwner->isGranularLocking());
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
		patient_lock_guard g(_mutex, _pPoolOwner->isGranularLocking());
		_objToIdMap.insert(std::make_pair(id.elementId(), id.elementId()));
	}

}

template<class T>
void ObjectPool<T>::resize(size_t size)
{
	patient_lock_guard g(_mutex, _pPoolOwner->isGranularLocking());
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
Index3DId ObjectPool<T>::findId(const T& obj) const
{
	if (_supportsReverseLookup) {
		patient_lock_guard g(_mutex, _pPoolOwner->isGranularLocking());
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
	patient_lock_guard g(_mutex, _pPoolOwner->isGranularLocking());
	bool result(id.elementId() < _idToIndexMap.size() && _idToIndexMap[id.elementId()] != -1);
	if (!result) {
		int dbgBreak = 1;
	}

	return result;
}

template<class T>
Index3DId ObjectPool<T>::findOrAdd(const T& obj, const Index3DId& currentId)
{
	patient_lock_guard g(_mutex, _pPoolOwner->isGranularLocking());
	if (_supportsReverseLookup) {
		auto id = findId(obj);
		if (id.isValid())
			return id;
	}

	size_t result = -1, index = -1, segNum = -1, segIdx = -1;
	if ((currentId.blockIdx() == _pPoolOwner->getBlockIdx()) && currentId.elementId() < _idToIndexMap.size()) {
		result = currentId.elementId();
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
				_objectSegs.push_back(std::make_shared<std::vector<T>>());
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

	getEntry(result)->setId(_pPoolOwner, result);

	addToLookup(Index3DId(_pPoolOwner->getBlockIdx(), result));

	return Index3DId(_pPoolOwner->getBlockIdx(), result);
}

template<class T>
const T* ObjectPool<T>::getSize_t(size_t id) const
{
	patient_lock_guard g(_mutex, _pPoolOwner->isGranularLocking());
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
	patient_lock_guard g(_mutex, _pPoolOwner->isGranularLocking());
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
	patient_lock_guard g(_mutex, _pPoolOwner->isGranularLocking());
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
	patient_lock_guard g(_mutex, _pPoolOwner->isGranularLocking());
	assert(id.blockIdx() == _pPoolOwner->getBlockIdx());
	return *getEntry(_idToIndexMap[id.elementId()]);
}

template<class T>
T& ObjectPool<T>::operator[](const Index3DId& id)
{
	patient_lock_guard g(_mutex, _pPoolOwner->isGranularLocking());
	assert(id.blockIdx() == _pPoolOwner->getBlockIdx());
	return *getEntry(_idToIndexMap[id.elementId()]);
}

template<class T>
template<class F>
void ObjectPool<T>::iterateInOrder(F fLambda) const
{
	size_t num;
	{
		patient_lock_guard g(_mutex, _pPoolOwner->isGranularLocking());
		num = _idToIndexMap.size();
	}
	for (size_t id = 0; id < num; id++) {
		size_t index;
		{
			patient_lock_guard g(_mutex, _pPoolOwner->isGranularLocking());
			index = _idToIndexMap[id];
		}
		if (index != -1) {
			auto p = getEntry(index);
			patient_lock_guard g(p->getMutex(), _pPoolOwner->isGranularLocking());
			fLambda(*p);
		}
	}
}

template<class T>
template<class F>
void ObjectPool<T>::iterateInOrder(F fLambda)
{
	size_t num;
	{
		patient_lock_guard g(_mutex, _pPoolOwner->isGranularLocking());
		num = _idToIndexMap.size();
	}
	for (size_t id = 0; id < num; id++) {
		size_t index;
		{
			patient_lock_guard g(_mutex, _pPoolOwner->isGranularLocking());
			index = _idToIndexMap[id];
		}
		if (index != -1) {
			auto p = getEntry(index);
			patient_lock_guard g(p->getMutex(), _pPoolOwner->isGranularLocking());
			fLambda(*p);
		}
	}
}

template<class T>
bool ObjectPool<T>::empty() const
{
	patient_lock_guard g(_mutex, _pPoolOwner->isGranularLocking());
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
	patient_lock_guard g(_mutex, _pPoolOwner->isGranularLocking());
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
	patient_lock_guard g(_mutex, _pPoolOwner->isGranularLocking());
	return _availableIndices.size();
}

template<class T>
size_t ObjectPool<T>::getNumUnloaded() const
{
	return -1;
}

}
