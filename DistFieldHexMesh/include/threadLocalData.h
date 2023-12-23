#pragma once

#include <vector>
#include <segmentedVector.h>

namespace DFHM_ObjectPool {
	using namespace DFHM;

	template<class T, int BITS>
	class ThreadLocalData {
	public:
		void free(size_t id); // Permanently delete it
		void unload(size_t id); // Free the memory, but keep the id

		void reset();
		void reserve(size_t size);
		bool idExists(size_t id) const;
		size_t add(const T& obj, size_t id = -1);
		const T& operator[](size_t id) const;
		T& operator[](size_t id);

		size_t getNumAllocated() const;
		size_t getNumAvailable() const;
		size_t getNumAvailableIds() const;
		size_t getNumUnloaded() const;

	private:
		std::vector<size_t> _idToIndexMap;
		std::vector<size_t> _availableData;
		std::vector<size_t> _availableIds;
		SegmentedVector<T, BITS> _pool;
	};

	template<class T, int BITS>
	void ThreadLocalData<T, BITS>::free(size_t id)
	{
		if (id > _idToIndexMap.size())
			return;

		size_t index = _idToIndexMap[id];
		if (index != -1) {
			_availableData.push_back(index);
			_availableIds.push_back(id);
			_idToIndexMap[id] = -1;
		}
	}

	template<class T, int BITS>
	void ThreadLocalData<T, BITS>::unload(size_t id)
	{
		size_t index = _idToIndexMap[id];
		_pool[index] = {};
		if (index != -1)
			_availableData.push_back(index);
		_idToIndexMap[id] = -1;
	}

	template<class T, int BITS>
	inline void ThreadLocalData<T, BITS>::reset() {
		_idToIndexMap.clear();
		_availableData.clear();
		_availableIds.clear();
		_pool.clear();
	}

	template<class T, int BITS>
	inline void ThreadLocalData<T, BITS>::reserve(size_t size)
	{
		_idToIndexMap.reserve(size);
	}

	template<class T, int BITS>
	inline bool ThreadLocalData<T, BITS>::idExists(size_t id) const
	{
		return id < _idToIndexMap.size() && _idToIndexMap[id] != -1;
	}

	template<class T, int BITS>
	size_t ThreadLocalData<T, BITS>::add(const T& obj, size_t id)
	{
		size_t index = -1;

		// Id does not exist, create one
		if (id >= _idToIndexMap.size()) {
			if (!_availableIds.empty()) {
				id = _availableIds.back();
				_availableIds.pop_back();
			}
			else {
				if (id == -1)
					id = _idToIndexMap.size();
				if (id >= _idToIndexMap.size())
					_idToIndexMap.resize(id + 1, -1);
			}
		}

		index = _idToIndexMap[id];
		if (index >= _pool.size()) {
			if (_availableData.empty()) {
				_idToIndexMap[id] = index = _pool.size();
				_pool.push_back(obj);
			}
			else {
				_idToIndexMap[id] = index = _availableData.back();
				_availableData.pop_back();
				_pool[index] = obj;
			}
		}

		return id;
	}

	template<class T, int BITS>
	T& ThreadLocalData<T, BITS>::operator[](size_t id)
	{
		if (id != -1 && id < _idToIndexMap.size())
			return _pool[_idToIndexMap[id]];

		throw std::exception("ThreadLocalData<T, BITS>::operator[] out of bounds");
	}

	template<class T, int BITS>
	const T& ThreadLocalData<T, BITS>::operator[](size_t id) const
	{
		if (id != -1 && id < _idToIndexMap.size())
			return _pool[_idToIndexMap[id]];

		throw std::exception("ThreadLocalData<T, BITS>::operator[] out of bounds");
	}

	template<class T, int BITS>
	size_t ThreadLocalData<T, BITS>::getNumAllocated() const
	{
		size_t num = 0;
		for (size_t idx : _idToIndexMap) {
			if (idx != -1)
				num++;
		}
		return num;
	}

	template<class T, int BITS>
	size_t ThreadLocalData<T, BITS>::getNumAvailable() const
	{
		return _availableData.size();
	}

	template<class T, int BITS>
	size_t ThreadLocalData<T, BITS>::getNumAvailableIds() const
	{
		return _availableIds.size();
	}

	template<class T, int BITS>
	size_t ThreadLocalData<T, BITS>::getNumUnloaded() const
	{
		return _idToIndexMap.size() - getNumAllocated();
	}

}
