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

bool PolygonSplitter::createTrimmedFace(const MTC::vector<MTC::set<Edge>>& patchFaces,
	const MTC::set<Index3DId>& skippedVerts, Index3DId& result)
{
	const double SDTol = Tolerance::sameDistTol();

	auto pMesh = _pBlock->getModelMesh();
	MTC::set<Edge> newFaceEdges;
	MTC::vector<Index3DId> newVertIds;

	MTC::vector<Index3DId> faceVertIds;
	Planed facePlane;
	_pBlock->faceFunc(TS_REAL, _polygonId, [&faceVertIds, &facePlane](const Polygon& face) {
		faceVertIds = face.getVertexIds();
		facePlane = face.calPlane();
	});

	for (const auto& patchFaceEdges : patchFaces) {
		for (const auto& cuttingEdge : patchFaceEdges) {
			Vector3d imprintPt0 = _pBlock->getVertexPoint(cuttingEdge.getVertex(0));
			Vector3d imprintPt1 = _pBlock->getVertexPoint(cuttingEdge.getVertex(1));

			if (!facePlane.isCoincident(imprintPt0, SDTol) || !facePlane.isCoincident(imprintPt1, SDTol))
				continue;

			newFaceEdges.insert(Edge(cuttingEdge.getVertex(0), cuttingEdge.getVertex(1)));

			for (size_t i = 0; i < faceVertIds.size(); i++) {
				size_t j = (i + 1) % faceVertIds.size();
				Edge faceEdge(faceVertIds[i], faceVertIds[j]), newEdge;
				if (createTrimmedEdge(faceEdge, cuttingEdge, newEdge))
					newFaceEdges.insert(newEdge);
			}
		}
	}

	// Add all edges which are outside the cutting surface and not included yet
	for (size_t i = 0; i < faceVertIds.size(); i++) {
		size_t j = (i + 1) % faceVertIds.size();
		const auto& vertId0 = faceVertIds[i];
		const auto& vertId1 = faceVertIds[j];
		if (!skippedVerts.contains(vertId0) && !skippedVerts.contains(vertId1)) {
			newFaceEdges.insert(Edge(vertId0, vertId1));
		}
	}

	MTC::vector<Index3DId> vertices;
	if (newFaceEdges.size() > 2 && PolygonSplitter::connectEdges(_pBlock, newFaceEdges, vertices)) {
		if (vertices.size() > 2)
			result = _pBlock->addFace(vertices);
		else
			assert(!"Bad vertex set to face");
	}

	return result.isValid();
}

Edge PolygonSplitter::createIntersectionEdge(const Planed& plane)
{
	MTC::vector<Vector3d> pts;
	faceFunc(TS_REAL, _polygonId, [this, &plane, &pts](const Polygon& face) {
		const auto& vertIds = face.getVertexIds();
		for (size_t i = 0; i < vertIds.size(); i++) {
			size_t j = (i + 1) % vertIds.size();
			Edge e(vertIds[i], vertIds[j]);
			auto seg = e.getSegment(getBlockPtr());
			RayHitd hit;
			if (plane.intersectLineSegment(seg, hit, Tolerance::sameDistTol())) {
				pts.push_back(hit.hitPt);
			}
		}
	});

	if (pts.size() == 2) {
		Index3DId vertId0 = getBlockPtr()->addVertex(pts[0]);
		Index3DId vertId1 = getBlockPtr()->addVertex(pts[1]);
		return Edge(vertId0, vertId1);
	}

	return Edge();
}

bool PolygonSplitter::createTrimmedEdge(const Edge& srcEdge, const Edge& cuttingEdge, Edge& newEdge)
{
	auto pMesh = _pBlock->getModelMesh();
	const auto& vertId0 = srcEdge.getVertex(0);
	const auto& vertId1 = srcEdge.getVertex(1);

	const Vector3d imprintPt0 = _pBlock->getVertexPoint(cuttingEdge.getVertex(0));
	const Vector3d imprintPt1 = _pBlock->getVertexPoint(cuttingEdge.getVertex(1));

	const Vector3d vertPt0 = _pBlock->getVertexPoint(vertId0);
	const Vector3d vertPt1 = _pBlock->getVertexPoint(vertId1);
	const LineSegment seg(vertPt0, vertPt1);
#if 0
	size_t triIndex0 = cuttingEdge._vertIds[0]._triIndex;
	size_t triIndex1 = cuttingEdge._vertIds[1]._triIndex;

	Vector3d v;
	double t, dp;
	if (triIndex0 != -1 && seg.contains(imprintPt0, t, Tolerance::sameDistTol())) {
		const Vector3d meshNorm0 = pMesh->triUnitNormal(triIndex0);
		if (t < Tolerance::paramTol()) {
			dp = meshNorm0.dot(vertPt1 - imprintPt0);
			if (dp >= 0) {
				newEdge = Edge(vertId1, cuttingEdge._vertIds[0]);
				return true;
			}
		} else if (t > 1 - Tolerance::paramTol()) {
			dp = meshNorm0.dot(vertPt0 - imprintPt0);
			if (dp >= 0) {
				newEdge = Edge(vertId0, cuttingEdge._vertIds[0]);
				return true;
			}
		} else {
			dp = meshNorm0.dot(vertPt0 - imprintPt0);
			if (dp >= 0) {
				newEdge = Edge(vertId0, cuttingEdge._vertIds[0]);
			} else {
				newEdge = Edge(vertId1, cuttingEdge._vertIds[0]);
			}
			return true;
		}
	} else if (triIndex1 != -1 && seg.contains(imprintPt1, t, Tolerance::sameDistTol())) {
		const Vector3d meshNorm1 = pMesh->triUnitNormal(triIndex1);
		if (t < Tolerance::paramTol()) {
			dp = meshNorm1.dot(vertPt1 - imprintPt1);
			if (dp >= 0) {
				newEdge = Edge(vertId1, cuttingEdge._vertIds[1]);
				return true;
			}
		} else if (t > 1 - Tolerance::paramTol()) {
			dp = meshNorm1.dot(vertPt0 - imprintPt0);
			if (dp >= 0) {
				newEdge = Edge(vertId0, cuttingEdge._vertIds[1]);
				return true;
			}
		} else {
			dp = meshNorm1.dot(vertPt0 - imprintPt1);
			if (dp >= 0) {
				newEdge = Edge(vertId0, cuttingEdge._vertIds[1]);
			}
			else {
				newEdge = Edge(vertId1, cuttingEdge._vertIds[1]);
			}
			return true;
		}
	}
#endif
	return false;
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

	MTC::vector<IntersectVertId> iVerts;
	if (connectIntersectEdges(pBlock, interEdges, iVerts, false)) {
		for (const auto& vert : iVerts)
			vertices.push_back(vert);
	}

	return !vertices.empty();
}

bool PolygonSplitter::connectIntersectEdges(const Block* pBlock, const MTC::set<IntersectEdge>& edgesIn, MTC::vector<IntersectVertId>& vertices, bool isIntersection)
{
	if (edgesIn.size() < 3)
		return false;

	MTC::set<IntersectEdge> edges(edgesIn);
	vertices.clear();
	MTC::vector<IntersectVertId> verts;
	verts.push_back(edges.begin()->_vertIds[0]);
	edges.erase(edges.begin());
	bool found = true;
	while (found && !edges.empty()) {
		found = false;
		const auto& v = verts.back();
		auto iter = edges.begin();
		while (iter != edges.end()) {
			auto e = *iter++;
			for (int i = 0; i < 2; i++) {
				if (e._vertIds[i] == v) {
					verts.push_back(e._vertIds[1 - i]);
					edges.erase(e);
					found = true;
					break;
				}
			}

			if (found)
				break;
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

const Block* PolygonSplitter::getBlockPtr() const
{
	return _pBlock;
}

Block* PolygonSplitter::getBlockPtr()
{
	return _pBlock;
}

//LAMBDA_CLIENT_IMPLS(PolygonSplitter)
void PolygonSplitter::vertexFunc(const Index3DId& id, const std::function<void(const Vertex& obj)>& func) const {
	auto p = getBlockPtr(); 
	p->vertexFunc(id, func);
} 

void PolygonSplitter::vertexFunc(const Index3DId& id, const std::function<void(Vertex& obj)>& func) {
	auto p = getBlockPtr(); 
	p->vertexFunc(id, func);
} 

void PolygonSplitter::faceFunc(TopolgyState state, const Index3DId& id, const std::function<void(const Polygon& obj)>& func) const {
	auto p = getBlockPtr(); 
	p->faceFunc(state, id, func);
} 

void PolygonSplitter::faceFunc(TopolgyState state, const Index3DId& id, const std::function<void(Polygon& obj)>& func) {
	auto p = getBlockPtr(); 
	p->faceFunc(state, id, func);
} 

void PolygonSplitter::cellFunc(TopolgyState state, const Index3DId& id, const std::function<void(const Polyhedron& obj)>& func) const {
	auto p = getBlockPtr(); 
	p->cellFunc(state, id, func);
} 

void PolygonSplitter::cellFunc(TopolgyState state, const Index3DId& id, const std::function<void(Polyhedron& obj)>& func) {
	auto p = getBlockPtr(); 
	p->cellFunc(state, id, func);
} 

void PolygonSplitter::faceAvailFunc(TopolgyState prefState, const Index3DId& id, const std::function<void(const Polygon& obj)>& func) const {
	auto p = getBlockPtr(); 
	p->faceAvailFunc(prefState, id, func);
} 

void PolygonSplitter::cellAvailFunc(TopolgyState prefState, const Index3DId& id, const std::function<void(const Polyhedron& obj)>& func) const {
	auto p = getBlockPtr(); 
	p->cellAvailFunc(prefState, id, func);
}
