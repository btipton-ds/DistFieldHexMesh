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

#include <edgeKey.h>

using namespace std;
using namespace DFHM;

EdgeKey::EdgeKey(const Index3DId& vert0, const Index3DId& vert1)
{
	_vertexIds[0] = vert0;
	_vertexIds[1] = vert1;
	_reversed = vert1 < vert0;
}

bool EdgeKey::operator < (const EdgeKey& rhs) const
{

	for (int i = 0; i < 2; i++) {
		auto& v = getSortedVertexId(i);
		auto& rhsV = rhs.getSortedVertexId(i);

		if (v < rhsV)
			return true;
		else if (rhsV < v)
			return false;
	}
	return false;
}

Index3DId EdgeKey::getOtherVert(const Index3DId& vert) const
{
	if (vert == _vertexIds[0])
		return _vertexIds[1];
	else if (vert == _vertexIds[1])
		return _vertexIds[0];

	assert(!"Invalid vert");
	return Index3DId();
}

bool EdgeKey::containsVertex(const Index3DId& vertexId) const
{
	return _vertexIds[0] == vertexId || _vertexIds[1] == vertexId;
}

