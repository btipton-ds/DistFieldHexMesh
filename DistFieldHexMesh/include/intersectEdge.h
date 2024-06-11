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

#include <algorithm>
#include <Index3D.h>

namespace DFHM
{

struct IntersectVertId : public Index3DId {
	IntersectVertId() = default;
	IntersectVertId(const IntersectVertId& src) = default;
	IntersectVertId(const Index3DId& id, const RayHitd& hit);
	IntersectVertId(const Index3DId& id, size_t triIdx);

	bool operator < (const IntersectVertId& rhs) const;

	size_t _triIndex = -1, _edgeIndex = -1;
};

struct IntersectEdge {
	IntersectEdge() = default;
	IntersectEdge(const IntersectEdge& src) = default;
	IntersectEdge(const IntersectVertId& vert0, const IntersectVertId& vert1);

	IntersectEdge& operator = (const IntersectEdge& rhs) = default;

	bool operator < (const IntersectEdge& rhs) const;

	IntersectVertId _vertIds[2];
};

inline IntersectVertId::IntersectVertId(const Index3DId& id, const RayHitd& hit)
	: Index3DId(id)
	, _triIndex(hit.triIdx)
	, _edgeIndex(hit.edgeIdx)
{
}

inline IntersectVertId::IntersectVertId(const Index3DId& id, size_t triIdx)
	: Index3DId(id)
	, _triIndex(triIdx)
	, _edgeIndex(-1)
{
}

inline bool IntersectVertId::operator < (const IntersectVertId& rhs) const
{
	return Index3DId::operator<(rhs);
}

inline IntersectEdge::IntersectEdge(const IntersectVertId& vert0, const IntersectVertId& vert1)
{
	_vertIds[0] = vert0;
	_vertIds[1] = vert1;
	if (_vertIds[1] < _vertIds[0]) {
		std::swap(_vertIds[0], _vertIds[1]);
	}
}

inline bool IntersectEdge::operator < (const IntersectEdge& rhs) const
{
	if (_vertIds[0] < rhs._vertIds[0])
		return true;
	else if (rhs._vertIds[0] < _vertIds[0])
		return false;

	return _vertIds[1] < rhs._vertIds[1];
}

}