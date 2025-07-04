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

#include <set>
#include <vector>
#include <defines.h>
#include <edge.h>

namespace DFHM {

namespace Utils {

void formEdgeLoops(const Block* pBlock, const MTC::set<EdgeKey> sharedSegments, MTC::set<EdgeKey> availEdges, MTC::vector<MTC::vector<EdgeKey>>& loops);

class Timer
{
public:
	enum TimerTag{
		TT_readVolume,
		TT_analyzeModelMesh,
		TT_splitAtPointInner,
		TT_needToSplitIntersection,
		TT_needToSplitConditional,
		TT_polyhedronIntersectsModel,
		TT_polyhedronTooManyFaces,
		TT_divideHexMesh,
		TT_UpdateTessellation,
		TT_lastTag,
	};

	static void dumpAll();

	Timer(size_t key);
	~Timer();

	void recordEntry();

private:
	size_t _key;
	bool _recorded = false;
	long long _startCount;
};

template <class T>
class ScopedRestore {
public:
	inline ScopedRestore(T& val)
		: _src(val)
		, _originalValue(val)
	{}

	inline ~ScopedRestore()
	{
		_src = _originalValue;
	}

private:
	T& _src;
	const T _originalValue;
};

}

}
