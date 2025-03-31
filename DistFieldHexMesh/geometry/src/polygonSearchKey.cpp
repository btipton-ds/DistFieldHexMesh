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
#include <polygonSearchKey.h>
#include <block.h>
#include <tolerances.h>

using namespace std;
using namespace DFHM;

namespace {
	inline bool isColinear(const Vector3d& pt0, const Vector3d& pt1, const Vector3d& pt2)
	{
		const auto tolSqr = Tolerance::paramTolSqr();
		Vector3d v0 = pt1 - pt0;
		Vector3d v1 = pt2 - pt1;
		v0.normalize();
		v1.normalize();
		double cpSqr = v1.cross(v0).squaredNorm();
		return cpSqr < tolSqr;
	}
}

MTC::vector<Index3DId> PolygonSearchKey::makeNonColinearVertexIds(const Block* pBlock, const MTC::vector<Index3DId>& vertexIds)
{
	MTC::vector<Index3DId> tmp;
	assert(pBlock);
	vector<bool> colin;
	vector<const Vector3d*> pts;
	colin.resize(vertexIds.size());
	pts.resize(vertexIds.size());

	for (size_t i = 0; i < vertexIds.size(); i++) {
		pts[i] = &pBlock->getVertexPoint(vertexIds[i]);
	}

	for (size_t j = 0; j < vertexIds.size(); j++) {
		size_t i = (j + vertexIds.size() - 1) % vertexIds.size();
		size_t k = (j + 1) % vertexIds.size();
		colin[j] = isColinear(*pts[i], *pts[j], *pts[k]);
	}
	for (size_t i = 0; i < colin.size(); i++) {
		if (!colin[i])
			tmp.push_back(vertexIds[i]);
	}

	if (tmp.size() != vertexIds.size()) {
		int dbgBreak = 1;
	}

	assert(tmp.size() == 4);
	return tmp;
}

void PolygonSearchKey::set(const Block* pBlock, const MTC::vector<Index3DId>& ids) const
{
	if (PolygonSearchKey::empty()) {
		_ids = makeNonColinearVertexIds(pBlock, ids);

		std::sort(_ids.begin(), _ids.end());
	}
}

bool PolygonSearchKey::operator < (const PolygonSearchKey& rhs) const
{
	if (_ids.size() < rhs._ids.size())
		return true;
	else if (_ids.size() > rhs._ids.size())
		return false;

	auto pLhs = _ids.data();
	auto pRhs = rhs._ids.data();
	for (size_t i = 0; i < _ids.size(); i++) {
		if (*pLhs < *pRhs)
			return true;
		else if (*pRhs < *pLhs)
			return false;
		pLhs++;
		pRhs++;
	}

	return false;
}
