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

	Copyright Robert R Tipton, 2022, all rights reserved except those granted in prior license statement.

	Dark Sky Innovative Solutions http://darkskyinnovation.com/
*/

#include <defines.h>
#include <vector>
#include <set>
#include <algorithm>

#if FAST_BISECTION_VALIDATION_ENABLED
#include <assert.h>
#endif

namespace DFHM {

template<class VAL>
struct FastBisectionSetDefComparator {
	inline bool operator()(const VAL& lhs, const VAL& rhs) const
	{
		return lhs < rhs;
	}
};

template<class VAL, class COMP>
class FastBisectionSet_with_comp {
public:

	FastBisectionSet_with_comp();
	FastBisectionSet_with_comp(const COMP& comp);

	size_t size() const;
	void clear();
	bool empty() const;
	bool contains(const VAL& id) const;
	const std::vector<VAL>& asVector() const;
	FastBisectionSet_with_comp& operator = (const FastBisectionSet_with_comp& rhs);
	FastBisectionSet_with_comp& operator = (const std::vector<VAL>& rhs);
	FastBisectionSet_with_comp& operator = (const std::set<VAL>& rhs);

	void insert(const VAL& VAL);
	template<class ITER>
	void insert(const ITER& iterBegin, const ITER& iterEnd)
	{
		for (auto iter = iterBegin; iter != iterEnd; iter++)
			insert(*iter);
	}

	size_t find(const VAL& id) const;
	void erase(const VAL& VAL);
	const VAL& operator[](const size_t& idx) const;
	const VAL* data() const;

	std::vector<VAL>::const_iterator begin() const;
	std::vector<VAL>::const_iterator end() const;

	bool isSorted() const;

private:
	void findIdx(const VAL& id, size_t& idx, size_t& idx0, size_t& idx1) const;
	bool isEqual(const VAL& lhs, const VAL& rhs) const;

	COMP _comp;
	mutable bool _sorted = true;
	mutable std::vector<VAL> _vals;
};

template<class VAL, class COMP>
inline bool FastBisectionSet_with_comp<VAL, COMP>::isEqual(const VAL& lhs, const VAL& rhs) const
{
	return !_comp(lhs, rhs) && !_comp(rhs, lhs); // !(a < b && a > b)
}

template<class VAL, class COMP>
inline FastBisectionSet_with_comp<VAL, COMP>::FastBisectionSet_with_comp()
	: _comp(COMP())
{
}

template<class VAL, class COMP>
inline FastBisectionSet_with_comp<VAL, COMP>::FastBisectionSet_with_comp(const COMP& comp)
	: _comp(comp)
{
}

template<class VAL, class COMP>
inline size_t FastBisectionSet_with_comp<VAL, COMP>::size() const
{
	return _vals.size();
}

template<class VAL, class COMP>
inline void FastBisectionSet_with_comp<VAL, COMP>::clear()
{
	_vals = {};
}

template<class VAL, class COMP>
inline bool FastBisectionSet_with_comp<VAL, COMP>::empty() const
{
	return _vals.empty();
}

template<class VAL, class COMP>
bool FastBisectionSet_with_comp<VAL, COMP>::contains(const VAL& id) const
{
#if FAST_BISECTION_VALIDATION_ENABLED
	assert(isSorted());
#endif
	size_t idx0, idx1;
	size_t idx;
	findIdx(id, idx, idx0, idx1);
	return idx < _vals.size();
}

template<class VAL, class COMP>
inline const std::vector<VAL>& FastBisectionSet_with_comp<VAL, COMP>::asVector() const
{
	if (!_sorted) {
		COMP comp;
		std::sort(_vals.begin(), _vals.end(), comp);
		_vals.shrink_to_fit();
		_sorted = true;
	}
#if FAST_BISECTION_VALIDATION_ENABLED
	assert(isSorted());
#endif
	return _vals;
}

template<class VAL, class COMP>
FastBisectionSet_with_comp<VAL, COMP>& FastBisectionSet_with_comp<VAL, COMP>::operator = (const FastBisectionSet_with_comp& rhs)
{
	_vals = rhs._vals;
	_sorted = rhs._sorted;
#if FAST_BISECTION_VALIDATION_ENABLED
	assert(isSorted());
#endif
	return  *this;
}

template<class VAL, class COMP>
FastBisectionSet_with_comp<VAL, COMP>& FastBisectionSet_with_comp<VAL, COMP>::operator = (const std::vector<VAL>& rhs)
{
	_vals = rhs;
	_sorted = false;
#if FAST_BISECTION_VALIDATION_ENABLED
	assert(isSorted());
#endif
	return  *this;
}

template<class VAL, class COMP>
FastBisectionSet_with_comp<VAL, COMP>& FastBisectionSet_with_comp<VAL, COMP>::operator = (const std::set<VAL>& rhs)
{
	_vals.clear();
	for (const auto& id : rhs)
		_vals.push_back(id);
	_sorted = false;
#if FAST_BISECTION_VALIDATION_ENABLED
	assert(isSorted());
#endif
	return  *this;
}

template<class VAL, class COMP>
void FastBisectionSet_with_comp<VAL, COMP>::insert(const VAL& newEntry)
{
	size_t idx0, idx1;
	size_t idx;
	findIdx(newEntry, idx, idx0, idx1);
	if (idx < _vals.size() && isEqual(_vals[idx], newEntry))
		return;

	if (_vals.empty()) {
		_vals.push_back(newEntry);
	} else if (_comp(newEntry, _vals.front())) {
		_vals.insert(_vals.begin(), newEntry);
	} else if (idx0 < _vals.size()) {
		bool inserted = false;
		for (size_t i = idx0; i < idx1 && i + 1 < _vals.size(); i++) {
			if (i == 0 && _comp(newEntry, _vals[i])) {
				_vals.insert(_vals.begin(), newEntry);
				inserted = true;
				break;
			} else if (_comp(_vals[i], newEntry) && _comp(newEntry, _vals[i + 1])) {
				_vals.insert(_vals.begin() + i + 1, newEntry);
				inserted = true;
				break;
			}
		}

		if (!inserted) {
			_sorted = _comp(_vals.back(), newEntry);
			if (!_sorted) {
				int dbgBreak = 1;
			}
			_vals.push_back(newEntry);
		}
	} else {
		// idx past the back, so adding to the back won't break the sort
		_vals.push_back(newEntry);
	}

#if FAST_BISECTION_VALIDATION_ENABLED
	assert(isSorted());
#endif
}

template<class VAL, class COMP>
size_t FastBisectionSet_with_comp<VAL, COMP>::find(const VAL& id) const
{
#if FAST_BISECTION_VALIDATION_ENABLED
	assert(isSorted());
#endif
	size_t idx0, idx1;
	size_t idx;
	findIdx(id, idx, idx0, idx1);
	return idx;
}

template<class VAL, class COMP>
inline void FastBisectionSet_with_comp<VAL, COMP>::erase(const VAL& id)
{
#if FAST_BISECTION_VALIDATION_ENABLED
	assert(isSorted());
#endif
	size_t idx = find(id);
	if (idx < _vals.size()) {
		_vals.erase(_vals.begin() + idx);
	}
}

template<class VAL, class COMP>
inline const VAL& FastBisectionSet_with_comp<VAL, COMP>::operator[](const size_t& idx) const
{
#if FAST_BISECTION_VALIDATION_ENABLED
	assert(isSorted());
#endif
	return _vals[idx];
}

template<class VAL, class COMP>
inline const VAL* FastBisectionSet_with_comp<VAL, COMP>::data() const
{
	return _vals.data();
}

template<class VAL, class COMP>
void FastBisectionSet_with_comp<VAL, COMP>::findIdx(const VAL& id, size_t& idx, size_t& idx0, size_t& idx1) const
{
#if FAST_BISECTION_VALIDATION_ENABLED
	assert(isSorted());
#endif
	if (_vals.empty()) {
		idx = idx0 = idx1 = -1;
		return;
	}

	if (!_sorted) {
		std::sort(_vals.begin(), _vals.end(), _comp);
		_vals.shrink_to_fit();
		_sorted = true;
	}

#if FAST_BISECTION_VALIDATION_ENABLED
	assert(isSorted());
#endif

	idx0 = 0; 
	idx1 = _vals.size() - 1;
	auto pVals = _vals.data();

	if (isEqual(pVals[idx0], id)) {
		idx = idx0;
		return;
	} else if (isEqual(pVals[idx1], id)) {
		idx = idx1;
		return;
	}

	idx = (idx0 + idx1 + 1) / 2;

	while (idx0 != idx1) {
		if (_comp(id, pVals[idx]))
			idx1 = idx + 1;
		else if (_comp(pVals[idx], id))
			idx0 = idx;
		else {
			return;
		}

		size_t nextIdx = (idx0 + idx1) / 2;
		if (idx == nextIdx) {
			break; // not found
		}
		idx = nextIdx;
	}

	if (pVals[idx] != id)
		idx = -1;
}

template<class VAL, class COMP>
inline bool FastBisectionSet_with_comp<VAL, COMP>::isSorted() const
{
#if FAST_BISECTION_VALIDATION_ENABLED
	if (_vals.empty())
		return true;

	if (!_sorted) {
		for (size_t i = 0; i < _vals.size() - 1; i++) {
			for (size_t j = i + 1; j < _vals.size(); j++) {
				if (_vals[i] == _vals[j])
					return false;
			}
		}
		return true;
	}

	for (size_t i = 0; i < _vals.size() - 1; i++) {
		if (_vals[i + 1] < _vals[i])
			return false;
		for (size_t j = i + 1; j < _vals.size(); j++) {
			if (_vals[i] == _vals[j])
				return false;
		}
	}
#endif
	return true;
}
#if 1
template<class VAL, class COMP>
inline std::vector<VAL>::const_iterator FastBisectionSet_with_comp<VAL, COMP>::begin() const
{
	return _vals.begin();
}

template<class VAL, class COMP>
inline std::vector<VAL>::const_iterator FastBisectionSet_with_comp<VAL, COMP>::end() const
{
	return _vals.end();
}
#endif

template<class VAL>
using FastBisectionSet = FastBisectionSet_with_comp<VAL, FastBisectionSetDefComparator<VAL>>;

}