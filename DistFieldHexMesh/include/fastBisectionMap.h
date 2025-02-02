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

#include <defines.h>
#include <vector>
#include <algorithm>

namespace DFHM {

template<class IDX, class VAL>
class FastBisectionMap {
public:
	using PAIR = std::pair<IDX, VAL>;
	void insert(const PAIR& pair);
	VAL operator[](const IDX& id) const;

private:
	size_t findIdx(const IDX& id, size_t& idx0, size_t& idx1) const;

	bool isSorted() const;

	mutable bool _sorted = true;
	mutable std::vector<PAIR> _vals;
};

template<class IDX, class VAL>
void FastBisectionMap<IDX, VAL>::insert(const PAIR& newEntry)
{
	size_t idx0, idx1;
	size_t idx = findIdx(newEntry.first, idx0, idx1);
	if (idx < _vals.size()) {
		if (_vals[idx].first == newEntry.first)
			return;

		bool inserted = false;
		for (size_t i = idx0; i < idx1 && i + 1 < _vals.size(); i++) {
			if (i == 0 && newEntry.first < _vals[i].first) {
				_vals.insert(_vals.begin(), newEntry);
				inserted = true;
				break;
			} else if (_vals[i].first < newEntry.first && newEntry.first < _vals[i + 1].first) {
				_vals.insert(_vals.begin() + i + 1, newEntry);
				inserted = true;
				break;
			}
		}

		if (!inserted) {
			_sorted = _vals.back().first < newEntry.first;
			if (!_sorted) {
				int dbgBreak = 1;
			}
			_vals.push_back(newEntry);
		}
	} else {
		// idx past the back, so adding to the back won't break the sort
		_vals.push_back(newEntry);
	}

#ifdef _DEBUG
	assert(!_sorted || isSorted());
#endif
}

template<class IDX, class VAL>
VAL FastBisectionMap<IDX, VAL>::operator[](const IDX& id) const
{
	size_t idx0, idx1;
	size_t idx = findIdx(id, idx0, idx1);
	if (idx < _vals.size()) {
		if (_vals[idx].first == id)
			return _vals[idx].second;
	}
	return -1;
}

template<class IDX, class VAL>
size_t FastBisectionMap<IDX, VAL>::findIdx(const IDX& id, size_t& idx0, size_t& idx1) const
{
	if (_vals.empty())
		return -1;
	if (!_sorted) {
		auto comp = [](const PAIR& lhs, const PAIR& rhs)->bool {
			return lhs.first < rhs.first;
			};
		std::sort(_vals.begin(), _vals.end(), comp);
	}
	idx0 = 0; 
	idx1 = _vals.size() - 1;

	if (_vals[idx0].first == id)
		return _vals[idx0].second;
	else if (_vals[idx1].first == id)
		return _vals[idx1].second;

	size_t idx = (idx0 + idx1 + 1) / 2;

	while (idx0 != idx1) {
		if (id < _vals[idx].first)
			idx1 = idx + 1;
		else if (_vals[idx].first < id)
			idx0 = idx;
		else {
			return idx;
		}

		size_t nextIdx = (idx0 + idx1) / 2;
		if (idx == nextIdx) {
			break; // not found
		}
		idx = nextIdx;
	}

	return idx;
}

template<class IDX, class VAL>
bool FastBisectionMap<IDX, VAL>::isSorted() const
{
	for (size_t i = 0; i < _vals.size() - 1; i++) {
		if (_vals[i + 1].first < _vals[i].first)
			return false;
	}
	return true;
}

}