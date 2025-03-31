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
#include <fastBisectionSet.h>

namespace DFHM {

template<class IDX, class VAL>
struct FBMPair {
	inline FBMPair(const IDX& idx, const VAL& v = VAL())
		: _idx(idx), _val(v)
	{}

	inline bool operator<(const FBMPair& rhs) const
	{
		return _idx < rhs._idx;
	}

	inline bool operator==(const FBMPair& rhs) const
	{
		return _idx == rhs._idx;
	}

	IDX _idx;
	mutable VAL _val;
};

template<class IDX, class VAL>
struct FastBisectionMapDefComparator {
	inline bool operator()(const FBMPair<IDX, VAL>& lhs, const FBMPair<IDX, VAL>& rhs) const
	{
		return lhs < rhs;
	}
};

template<class IDX, class VAL, class COMP>
class FastBisectionMap_with_comp : public FastBisectionSet_with_comp<FBMPair<IDX, VAL>, COMP> {
public:
	using PAIR = FBMPair<IDX, VAL>;

	FastBisectionMap_with_comp();
	FastBisectionMap_with_comp(const COMP& comp);

	void insert(const IDX& i, const VAL& v);

	const VAL& operator[](const IDX& idx) const;
};

template<class IDX, class VAL, class COMP>
FastBisectionMap_with_comp<IDX, VAL, COMP>::FastBisectionMap_with_comp()
	: FastBisectionSet_with_comp<FBMPair<IDX, VAL>, COMP>(COMP())
{
}

template<class IDX, class VAL, class COMP>
FastBisectionMap_with_comp<IDX, VAL, COMP>::FastBisectionMap_with_comp(const COMP& comp)
	: FastBisectionSet_with_comp<FBMPair<IDX, VAL>, COMP>(comp)
{
}

template<class IDX, class VAL, class COMP>
inline void FastBisectionMap_with_comp<IDX, VAL, COMP>::insert(const IDX& i, const VAL& v)
{
	FastBisectionSet_with_comp<FBMPair<IDX, VAL>, COMP>::insert(FBMPair(i, v));
}

template<class IDX, class VAL, class COMP>
const VAL& FastBisectionMap_with_comp<IDX, VAL, COMP>::operator[](const IDX& idx) const
{
	FBMPair<IDX, VAL> key(idx);
	size_t idx2 = FastBisectionSet_with_comp<FBMPair<IDX, VAL>, COMP>::find(key);
	auto& val = FastBisectionSet_with_comp<FBMPair<IDX, VAL>, COMP>::operator[](idx2);
	return val._val;
}

template<class IDX, class VAL>
using FastBisectionMap = FastBisectionMap_with_comp<IDX, VAL, FastBisectionMapDefComparator<IDX, VAL>>;

}