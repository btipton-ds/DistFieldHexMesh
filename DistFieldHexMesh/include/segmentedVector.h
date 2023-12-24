#pragma once

#include <array>
#include <vector>
#include <tm_vector3.h>
#include <fixedArray.h>

namespace DFHM {

template<class T, int BITS>
class SegmentedVector {
public:
	T& operator[](size_t idx);
	const T& operator[](size_t idx) const;
	void push_back(const T& val);
	size_t size() const;
	void clear();

	template<class FUNC>
	void iterateInOrder(FUNC f) const
	{
		for (const std::shared_ptr<FixedArrayTB>& ptr : _data) {
			const FixedArrayTB& fa = *ptr;
			for (size_t i = 0; i < fa.size(); i++) {
				f(fa[i]);
			}
		}
	}

private:
	using FixedArrayTB = FixedArray<T, BITS>;
	std::vector<std::shared_ptr<FixedArrayTB>> _data;
};

template<class T, int BITS>
T& SegmentedVector<T, BITS>::operator[](size_t idx)
{
	const size_t mask = FixedArrayTB::calMask();

	size_t idx0 = idx >> BITS;
	size_t idx1 = idx & mask;
	return _data[idx0]->operator[](idx1);
}

template<class T, int BITS>
const T& SegmentedVector<T, BITS>::operator[](size_t idx) const
{
	const size_t mask = FixedArrayTB::calMask();
	size_t idx0 = idx >> BITS;
	size_t idx1 = idx & mask;
	return _data[idx0]->operator[](idx1);
}

template<class T, int BITS>
void SegmentedVector<T, BITS>::push_back(const T& val)
{
	const size_t maxSize = (((size_t)1) << BITS);
	if (_data.empty() || _data.back()->size() >= maxSize) {
		auto pNew = std::make_shared <FixedArrayTB>();
		_data.push_back(pNew);
	}
	_data.back()->push_back(val);
}

template<class T, int BITS>
size_t SegmentedVector<T, BITS>::size() const
{
	size_t result = 0;
	for (auto p : _data)
		result += p->size();
	return result;
}

template<class T, int BITS>
void SegmentedVector<T, BITS>::clear()
{
	_data.clear();
}

}