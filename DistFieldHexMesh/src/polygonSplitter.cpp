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
#include <polygonSplitter.h>
#include <block.h>
#include <polygon.h>

using namespace std;
using namespace DFHM;

PolygonSplitter::PolygonSplitter(Block* pBlock, const Index3DId& polygonId)
	: _pBlock(pBlock)
	, _polygonId(polygonId)
{
}

bool PolygonSplitter::splitAtCentroid()
{
	Vector3d ctr;
	_pBlock->faceAvailFunc(_polygonId, TS_REFERENCE, [&ctr](const Polygon& face) {
		ctr = face.calCentroid();
		});

	return splitAtPoint(ctr);
}

bool PolygonSplitter::splitAtPoint(const Vector3d& pt)
{
	_pBlock->makeRefPolygonIfRequired(_polygonId);

	assert(_pBlock->polygonExists(TS_REFERENCE, _polygonId));
	Polygon& referenceFace = _pBlock->getPolygon(TS_REFERENCE, _polygonId);

	if (!referenceFace._splitFaceProductIds.empty()) {
		return false;
	}

	assert(_pBlock->polygonExists(TS_REAL, _polygonId));
	Polygon& realFace = _pBlock->getPolygon(TS_REAL, _polygonId);

	bool result = splitAtPointInner(realFace, referenceFace, pt);

	if (_pBlock->polygonExists(TS_REAL, _polygonId)) {
		_pBlock->freePolygon(_polygonId);
	}

	return result;
}

bool PolygonSplitter::splitAtPointInner(Polygon& realFace, Polygon& referanceFace, const Vector3d& pt) const
{
#if DEBUG_BREAKS && defined(_DEBUG)
	if (Index3DId(0, 8, 4, 4) == _polygonId) {
		int dbgBreak = 1;
	}
#endif

	assert(_pBlock->isPolygonReference(&referanceFace));
	assert(_pBlock->polygonExists(TS_REAL, _polygonId));
	const double sinAngleTol = sin(Tolerance::angleTol());

#if LOGGING_ENABLED
	auto pLogger = _pBlock->getLogger();
	auto& out = pLogger->getStream();

	LOG(out << Logger::Pad() << "Polygon::splitAtPoint. pre: " << realFace);
#endif

	assert(referanceFace.cellsOwnThis());

	// The code must be operating on the reference face
	const auto& vertexIds = referanceFace.getVertexIds();
	assert(vertexIds.size() == 4);
	vector<Index3DId> edgePtIds;
	edgePtIds.resize(vertexIds.size());
	for (size_t i = 0; i < vertexIds.size(); i++) {
		size_t j = (i + 1) % vertexIds.size();
		Edge edge(vertexIds[i], vertexIds[j]);

		// Be sure to project directly to the edge itself. 
		// DO NOT project to the face followed by the edge, because that can result in two points on the same edge.
		Vector3d edgePt = edge.projectPt(_pBlock, pt);
		bool inBounds;
		double t = edge.paramOfPt(_pBlock, edgePt, inBounds);
		if (inBounds) {
			Index3DId vertId = _pBlock->addVertex(edgePt);
			edgePtIds[i] = vertId;

			referanceFace.addSplitEdgeVert(edge, vertId);
		}
		else {
			assert(!"Edge point is not in bounds.");
			return false;
		}
	}

	Vector3d facePt = referanceFace.projectPoint(pt);
#if _DEBUG
	if (!referanceFace.containsPoint(facePt)) {
		assert(!"Face point is not in bounds.");
		return false;
	}
#endif

	Index3DId facePtId = _pBlock->addVertex(facePt);

#ifdef _DEBUG
	Vector3d srcNorm = referanceFace.calUnitNormal();
#endif // _DEBUG

	auto cellIds = realFace.getCellIds();
	for (size_t i = 0; i < vertexIds.size(); i++) {
		size_t j = (i + vertexIds.size() - 1) % vertexIds.size();
		auto priorEdgeId = edgePtIds[j];
		auto vertId = vertexIds[i];
		auto nextEdgeId = edgePtIds[i];
		Polygon newFace({ facePtId, priorEdgeId, vertId, nextEdgeId });

#ifdef _DEBUG
		Vector3d newNorm = Polygon::calUnitNormalStat(_pBlock, newFace.getVertexIds());
		double cp = newNorm.cross(srcNorm).norm();
		assert(cp < sinAngleTol);
#endif // _DEBUG

		auto newFaceId = _pBlock->addFace(newFace);
		referanceFace.addToSplitFaceProductIds(newFaceId);
	}

	for (const auto& cellId : cellIds) {
		if (_pBlock->polyhedronExists(TS_REAL, cellId)) {
			size_t splitLevel = realFace.getSplitLevel(cellId);
			const auto& splits = referanceFace._splitFaceProductIds;

			_pBlock->makeRefPolyhedronIfRequired(cellId);
			_pBlock->cellRealFunc(cellId, [this, &splits, splitLevel](Polyhedron& cell) {
				cell.replaceFaces(_polygonId, splits, splitLevel + 1);
				});
		}
	}

#if LOGGING_VERBOSE_ENABLED
	LOG(out << Logger::Pad() << "Polygon::splitAtPoint. pst: " << *this);
#endif

	return true;
}

bool PolygonSplitter::splitAtPlane(const Plane<double>& plane)
{
	return false;
}

bool PolygonSplitter::splitAtPlaneInner(Polygon& realFace, Polygon& referanceFace, Plane<double>& plane) const
{
	return false;
}
