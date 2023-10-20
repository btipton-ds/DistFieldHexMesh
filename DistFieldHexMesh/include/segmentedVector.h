#pragma once

#include <mutex>
#include <vector>
#include <tm_vector3.h>

namespace DFHM {

template<class T, int BITS>
class SegmentedVector {
public:
	T& operator[](size_t idx);
	const T& operator[](size_t idx) const;
	void push_back(const T& val);
	size_t size() const;
	void reserve(size_t size);
	void clear();

	std::recursive_mutex& getMutex() const;
private:
	mutable std::recursive_mutex _mutex;

	std::vector<std::shared_ptr<std::vector<T>>> _data;
};

template<class T, int BITS>
T& SegmentedVector<T, BITS>::operator[](size_t idx)
{
	const size_t mask = (((size_t)1) << BITS) - 1;
	std::lock_guard<std::recursive_mutex> lock(_mutex);
	size_t idx0 = idx >> BITS;
	size_t idx1 = idx & mask;
	return _data[idx0]->operator[](idx1);
}

template<class T, int BITS>
const T& SegmentedVector<T, BITS>::operator[](size_t idx) const
{
	const size_t mask = (((size_t)1) << BITS) - 1;
	std::lock_guard<std::recursive_mutex> lock(_mutex);
	size_t idx0 = idx >> BITS;
	size_t idx1 = idx & mask;
	return _data[idx0]->operator[](idx1);
}

template<class T, int BITS>
void SegmentedVector<T, BITS>::push_back(const T& val)
{
	const size_t maxSize = (((size_t)1) << BITS);
	std::lock_guard<std::recursive_mutex> lock(_mutex);
	if (_data.empty() || _data.back()->size() >= maxSize) {
		auto pNew = std::make_shared <std::vector<T>>();
		pNew->reserve(maxSize);
		pNew->push_back(val);
		_data.push_back(pNew);
	}
	else {
		_data.back()->push_back(val);
	}
}

template<class T, int BITS>
size_t SegmentedVector<T, BITS>::size() const
{
	std::lock_guard<std::recursive_mutex> lock(_mutex);
	size_t result = 0;
	for (auto p : _data)
		result += p->size();
	return result;
}

template<class T, int BITS>
inline std::recursive_mutex& SegmentedVector<T, BITS>::getMutex() const
{
	return _mutex;
}

template<class T, int BITS>
void SegmentedVector<T, BITS>::clear()
{
	_data.clear();
}

}