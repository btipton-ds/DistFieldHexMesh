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
#include <assert.h>
#include <pool_vector.h>
#include <tm_lineSegment.h>
#include <polygonSplitter.h>
#include <block.h>
#include <polygon.h>
#include <tolerances.h>

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
	_pBlock->faceAvailFunc(TS_REFERENCE, _polygonId, [&ctr](const Polygon& face) {
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
		_pBlock->freePolygon(_polygonId, true);
	}

	return result;
}

bool PolygonSplitter::splitAtPointInner(Polygon& realFace, Polygon& referanceFace, const Vector3d& pt) const
{
	assert(_pBlock->isPolygonReference(&referanceFace));
	assert(_pBlock->polygonExists(TS_REAL, _polygonId));
	const double sinAngleTol = sin(Tolerance::angleTol());

	assert(referanceFace.cellsOwnThis());

	// The code must be operating on the reference face
	const auto& vertexIds = referanceFace.getVertexIds();
	assert(vertexIds.size() == 4);
	MTC::vector<Index3DId> edgePtIds;
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
			_pBlock->cellFunc(TS_REAL, cellId, [this, &splits, splitLevel](Polyhedron& cell) {
				cell.replaceFaces(_polygonId, splits, splitLevel + 1);
			});
		}
	}

	return true;
}

bool PolygonSplitter::splitWithFace(const Index3DId& imprintFaceId, Index3DId& lowerFaceId, Index3DId upperFaceId) const
{
	_pBlock->makeRefPolygonIfRequired(_polygonId);
	assert(_pBlock->polygonExists(TS_REAL, imprintFaceId));
	assert(_pBlock->polygonExists(TS_REAL, _polygonId));
	assert(_pBlock->polygonExists(TS_REFERENCE, _polygonId));

	Polygon& referenceFace = _pBlock->getPolygon(TS_REFERENCE, _polygonId);

	if (!referenceFace._splitFaceProductIds.empty()) {
		return false;
	}

	Polygon& realFace = _pBlock->getPolygon(TS_REAL, _polygonId);
	Polygon& imprintFace = _pBlock->getPolygon(TS_REAL, imprintFaceId);


	bool result = splitWithFaceInner(imprintFace, realFace, referenceFace);

	if (_pBlock->polygonExists(TS_REAL, _polygonId)) {
		_pBlock->freePolygon(_polygonId, true);
	}

	return result;
}

Index3DId PolygonSplitter::createTrimmedFace(const MTC::set<IntersectEdge>& cutFaceEdges)
{
	Index3DId result;

	auto pMesh = _pBlock->getModelMesh();
	MTC::set<Edge> newFaceEdges;
	MTC::vector<Index3DId> newVertIds;

	MTC::vector<Index3DId> vertIds;
	Planed facePlane;
	_pBlock->faceFunc(TS_REAL, _polygonId, [&vertIds, &facePlane](const Polygon& face) {
		vertIds = face.getVertexIds();
		facePlane = face.calPlane();
	});

	for (const auto& cuttingEdge : cutFaceEdges) {
		Vector3d imprintPt0 = _pBlock->getVertexPoint(cuttingEdge._vert0);
		Vector3d modelNorm0 = pMesh->triUnitNormal(cuttingEdge._vert0._triIndex);

		Vector3d imprintPt1 = _pBlock->getVertexPoint(cuttingEdge._vert1);
		Vector3d modelNorm1 = pMesh->triUnitNormal(cuttingEdge._vert1._triIndex);

		if (facePlane.distanceToPoint(imprintPt0) > Tolerance::sameDistTol() || facePlane.distanceToPoint(imprintPt1) > Tolerance::sameDistTol())
			continue;

		newFaceEdges.insert(Edge(cuttingEdge._vert0, cuttingEdge._vert1));

		for (size_t i = 0; i < vertIds.size(); i++) {
			size_t j = (i + 1) % vertIds.size();
			const auto& vertId0 = vertIds[i];
			const auto& vertId1 = vertIds[j];
			double dp[2][2];

			Vector3d pt0 = _pBlock->getVertexPoint(vertId0);
			Vector3d pt1 = _pBlock->getVertexPoint(vertId1);
			LineSegment seg(pt0, pt1);

			dp[0][0] = modelNorm0.dot(pt0 - imprintPt0);
			dp[0][1] = modelNorm1.dot(pt0 - imprintPt1);
			dp[1][0] = modelNorm0.dot(pt1 - imprintPt0);
			dp[1][1] = modelNorm1.dot(pt1 - imprintPt1);

			if (
				(dp[0][0] > Tolerance::sameDistTol() && dp[1][1] > Tolerance::sameDistTol()) ||
				(dp[0][1] > Tolerance::sameDistTol() && dp[1][0] > Tolerance::sameDistTol()) ) {
				newFaceEdges.insert(Edge(vertId0, vertId1));
			} else if ( /* Crossing case */
				(dp[0][0] * dp[1][1] <= 0) ||
				(dp[0][1] * dp[1][0] <= 0)) {
				double t = -1;
				if (seg.contains(imprintPt0, t)) {
					if (dp[0][0] > 0)
						newFaceEdges.insert(Edge(vertId0, cuttingEdge._vert0));
					else
						newFaceEdges.insert(Edge(vertId1, cuttingEdge._vert0));
				}
				else if (seg.contains(imprintPt1, t)) {
					if (dp[0][1] > 0)
						newFaceEdges.insert(Edge(vertId0, cuttingEdge._vert1));
					else
						newFaceEdges.insert(Edge(vertId1, cuttingEdge._vert1));
				}
			}
		}
	}

	MTC::vector<Index3DId> vertices;
	if (PolygonSplitter::connectEdges(_pBlock, newFaceEdges, vertices)) {
		result = _pBlock->addFace(vertices);
	}
	return result;
}

bool PolygonSplitter::connectEdges(const Block* pBlock, const MTC::set<Edge>& edges, MTC::vector<Index3DId>& vertices)
{
	if (edges.empty())
		return false;

	MTC::set<IntersectEdge> interEdges;
	for (const auto& edge : edges) {
		IntersectVertId iv0(edge.getVertex(0), -1);
		IntersectVertId iv1(edge.getVertex(1), -1);

		interEdges.insert(IntersectEdge(iv0, iv1));
	}

	return connectIntersectEdges(pBlock, interEdges, vertices, false);
}

bool PolygonSplitter::connectIntersectEdges(const Block* pBlock, const MTC::set<IntersectEdge>& edgesIn, MTC::vector<Index3DId>& vertices, bool isIntersection)
{
	if (edgesIn.empty())
		return false;

	MTC::set<IntersectEdge> edges(edgesIn);
	vertices.clear();
	MTC::vector<IntersectVertId> verts;
	verts.push_back(edges.begin()->_vert0);
	edges.erase(edges.begin());
	bool found = true;
	while (found && !edges.empty()) {
		found = false;
		const auto& v = verts.back();
		auto iter = edges.begin();
		while (iter != edges.end()) {
			auto e = *iter++;
			if (e._vert0 == v) {
				verts.push_back(e._vert1);
				edges.erase(e);
				found = true;
				break;
			} else if (e._vert1 == v) {
				verts.push_back(e._vert0);
				edges.erase(e);
				found = true;
				break;
			}
		}
	}

	if (!edges.empty())
		return false;

	if (isIntersection) {
		Vector3d triNorm(0, 0, 0), faceNorm(0, 0, 0);
		auto pMesh = pBlock->getModelMesh();
		for (const auto& iv : verts) {
			auto n = pMesh->triUnitNormal(iv._triIndex);
			triNorm += n;
		}
		triNorm.normalize();

		size_t i = 0;
		auto pt0 = pBlock->getVertexPoint(verts[i]);
		for (size_t j = 1; j < verts.size() - 1; j++) {
			size_t k = (j + 1) % verts.size();
			auto pt1 = pBlock->getVertexPoint(verts[j]);
			auto pt2 = pBlock->getVertexPoint(verts[k]);
			Vector3d v0 = pt2 - pt1;
			Vector3d v1 = pt0 - pt1;
			Vector3d n = v1.cross(v0).normalized();
			faceNorm += n;
		}

		faceNorm.normalize();
		if (triNorm.dot(faceNorm) < 0) {
			std::reverse(verts.begin(), verts.end());
		}
	}
	for (const auto& v : verts)
		vertices.insert(vertices.end(), v);

	return !vertices.empty();
}


bool PolygonSplitter::splitWithFaceInner(const Polygon& imprintFace, Polygon& realFace, Polygon& referanceFace) const
{
	cout << "real face ids\n";
	for (const auto& vertId : realFace.getVertexIds()) {
		Vector3d pt = _pBlock->getVertexPoint(vertId);
		cout << "  " << vertId << ": " << pt << "\n";
	}

	MTC::vector<size_t> vertIndices;
	MTC::vector<Index3DId> imprintVertIds;
	const auto& srcIds = imprintFace.getVertexIds();
	for (const auto& vertId : srcIds) {
		size_t vertIdx = realFace.getImprintIndex(vertId);
		if (vertIdx != -1) {
			vertIndices.push_back(vertIdx);
			imprintVertIds.push_back(vertId);
		}
	}
	if (imprintVertIds.empty())
		return false;

	assert(imprintVertIds.size() == 2);

	MTC::vector<Index3DId> verts0, verts1;

	size_t firstIdx = vertIndices[0];
	size_t lastIdx = vertIndices[1];
	for (size_t i = 0; i < srcIds.size(); i++) {
		size_t ii = (i + firstIdx) % srcIds.size();
		if (i == 0) {
			verts0.push_back(imprintVertIds[firstIdx]);
			verts0.push_back(srcIds[ii]);
		} else if (ii == lastIdx) {
			verts0.push_back(srcIds[ii]);
			verts0.push_back(imprintVertIds[lastIdx]);
			break;
		}
	}

	swap(firstIdx, lastIdx);
	for (size_t i = 0; i < srcIds.size(); i++) {
		size_t ii = (i + firstIdx) % srcIds.size();
		if (i == 0) {
			verts1.push_back(imprintVertIds[firstIdx]);
			verts1.push_back(srcIds[ii]);
		}
		else if (ii == lastIdx) {
			verts1.push_back(srcIds[ii]);
			verts1.push_back(imprintVertIds[lastIdx]);
			break;
		}
	}

	cout << "Verts0\n";
	Polygon::dumpPolygonPoints(_pBlock, cout, verts0);

	cout << "Verts1\n";
	Polygon::dumpPolygonPoints(_pBlock, cout, verts1);

	return false;
}
