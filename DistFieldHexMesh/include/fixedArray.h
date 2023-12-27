#pragma once

#include <vector>
#include <memory>

namespace DFHM {

template<class T, int BITS>
class FixedArray {
public:
	static size_t calMask();
	static size_t calMaxSize();
	FixedArray();
	FixedArray(const FixedArray& src) = default;

	const T& operator[](size_t index) const;
	T& operator[](size_t index);

	void clear();
	bool empty() const;
	size_t size() const;
	size_t push_back(const T& val);

private:
	size_t _currentSize = 0;
	std::shared_ptr<T[]> _data; // NOT std::array or std:vector by design. array is fixed size and vector can be resized - both are bad in the use case!
	std::vector<size_t> _availableSlots;
};

template<class T, int BITS>
inline size_t FixedArray<T, BITS>::calMask()
{
	return calMaxSize() - 1;
}

template<class T, int BITS>
inline size_t FixedArray<T, BITS>::calMaxSize()
{
	return (((size_t)1) << BITS);
}

template<class T, int BITS>
inline FixedArray<T, BITS>::FixedArray()
{
	std::shared_ptr<T[]> p(new T[calMaxSize()]()); // make_shared<T[]> does not work on VC++ as of 12/23;
	_data = p;
}


template<class T, int BITS>
const T& FixedArray<T, BITS>::operator[](size_t index) const
{
	return _data[index];
}

template<class T, int BITS>
T& FixedArray<T, BITS>::operator[](size_t index)
{
	return _data[index];
}

template<class T, int BITS>
inline void FixedArray<T, BITS>::clear()
{
	_data = std::make_shared<T[]>(calMaxSize());
	_currentSize = 0;
}

template<class T, int BITS>
inline bool FixedArray<T, BITS>::empty() const
{
	return _currentSize == 0;
}

template<class T, int BITS>
inline size_t FixedArray<T, BITS>::size() const
{
	return _currentSize;
}

template<class T, int BITS>
size_t FixedArray<T, BITS>::push_back(const T& val)
{
	_data[_currentSize++] = val;
	return _currentSize;
}

}
