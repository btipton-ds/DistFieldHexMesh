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
#include <splitParams.h>
#include <tolerances.h>

using namespace std;
using namespace DFHM;

namespace
{

std::set<Index3DId> breakIds = {
	Index3DId(3, 9, 0, 33),
	Index3DId(3, 9, 0, 52),
};

}

PolygonSplitter::PolygonSplitter(Block* pBlock, const Index3DId& polygonId)
	: _pBlock(pBlock)
	, _polygonId(polygonId)
{
}

Index3DId PolygonSplitter::addVertex(const Vector3d& pt) const
{
	return _pBlock->addVertex(pt);
}

Index3DId PolygonSplitter::addFace(const Polygon& face) const
{
	return _pBlock->addFace(face);
}


bool PolygonSplitter::splitAtCentroid()
{
	Vector3d ctr;
	getBlockPtr()->faceAvailFunc(TS_REFERENCE, _polygonId, [&ctr](const Polygon& face) {
		ctr = face.calCentroid();
		});

	return splitAtPoint(ctr);
}

bool PolygonSplitter::splitAtPoint(const Vector3d& pt)
{
	getBlockPtr()->makeRefPolygonIfRequired(_polygonId);

	assert(getBlockPtr()->polygonExists(TS_REFERENCE, _polygonId));
	Polygon& referenceFace = getBlockPtr()->getPolygon(TS_REFERENCE, _polygonId);

	if (!referenceFace._splitFaceProductIds.empty()) {
		return false;
	}

	assert(getBlockPtr()->polygonExists(TS_REAL, _polygonId));
	Polygon& realFace = getBlockPtr()->getPolygon(TS_REAL, _polygonId);

	bool result = splitAtPointInner(realFace, referenceFace, pt);

	if (getBlockPtr()->polygonExists(TS_REAL, _polygonId)) {
		getBlockPtr()->freePolygon(_polygonId, true);
	}

	return result;
}

bool PolygonSplitter::splitAtPointInner(Polygon& realFace, Polygon& referanceFace, const Vector3d& pt) const
{
	assert(getBlockPtr()->isPolygonReference(&referanceFace));
	assert(getBlockPtr()->polygonExists(TS_REAL, _polygonId));
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
		Vector3d edgePt = edge.projectPt(getBlockPtr(), pt);
		bool inBounds;
		double t = edge.paramOfPt(getBlockPtr(), edgePt, inBounds);
		if (inBounds) {
			Index3DId vertId = addVertex(edgePt);
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

	Index3DId facePtId = addVertex(facePt);

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
		Vector3d newNorm = Polygon::calUnitNormalStat(getBlockPtr(), newFace.getVertexIds());
		double cp = newNorm.cross(srcNorm).norm();
		assert(cp < sinAngleTol);
#endif // _DEBUG

		auto newFaceId = addFace(newFace);
		referanceFace.addToSplitFaceProductIds(newFaceId);
	}

	for (const auto& cellId : cellIds) {
		if (getBlockPtr()->polyhedronExists(TS_REAL, cellId)) {
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

	auto pMesh = getBlockPtr()->getModelMesh();
	MTC::set<Edge> newFaceEdges;
	MTC::vector<Index3DId> newVertIds;

	MTC::vector<Index3DId> faceVertIds;
	Planed facePlane;
	getBlockPtr()->faceFunc(TS_REAL, _polygonId, [&faceVertIds, &facePlane](const Polygon& face) {
		faceVertIds = face.getVertexIds();
		facePlane = face.calPlane();
	});

	for (const auto& patchFaceEdges : patchFaces) {
		for (const auto& cuttingEdge : patchFaceEdges) {
			Vector3d imprintPt0 = getBlockPtr()->getVertexPoint(cuttingEdge.getVertex(0));
			Vector3d imprintPt1 = getBlockPtr()->getVertexPoint(cuttingEdge.getVertex(1));

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
	if (newFaceEdges.size() > 2 && PolygonSplitter::connectEdges(getBlockPtr(), newFaceEdges, vertices)) {
		if (vertices.size() > 2)
			result = getBlockPtr()->addFace(vertices);
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
		Index3DId vertId0 = addVertex(pts[0]);
		Index3DId vertId1 = addVertex(pts[1]);
		return Edge(vertId0, vertId1);
	}

	return Edge();
}

void PolygonSplitter::createTrimmedFacesFromFaces(const MTC::set<Index3DId>& modelFaces, MTC::set<Index3DId>& newFaceIds)
{
	faceFunc(TS_REAL, _polygonId, [&](const Polygon& realFace) {
		if (breakIds.contains(_polygonId)) {
			int dbgBreak = 1;
		}

		Planed facePlane = realFace.calPlane();
		MTC::set<Edge> modelEdges, edges;
		for (const auto& faceId : modelFaces) {
			faceFunc(TS_REAL, faceId, [this, &realFace, &modelEdges](const Polygon& modelFace) {
				const auto& modelFaceEdges = modelFace.getEdges();
				for (const auto& modelEdge : modelFaceEdges) {
					if (realFace.isCoplanar(modelEdge)) {
						modelEdges.insert(modelEdge);
				}
					else {
						int dbgBreak = 1;
					}
				}
		});
		}

		createTrimmedFaceEdges(modelEdges, edges);

		MTC::map<Index3DId, MTC::set<Index3DId>> vertToVertMap;
		for (const auto& edge : edges) {
			for (int i = 0; i < 2; i++) {
				const auto& vertId = edge.getVertex(i);
				auto iter = vertToVertMap.insert(std::make_pair(vertId, MTC::set<Index3DId>())).first;
				iter->second.insert(edge.getOtherVert(vertId));
			}
		}

		while (!vertToVertMap.empty()) {
			MTC::vector<Index3DId> verts;
			MTC::set<Index3DId> usedVerts;
			auto iter = vertToVertMap.begin();

			verts.push_back(iter->first);
			usedVerts.insert(verts.back());
			const auto& others = iter->second;
			assert(!others.empty());

			verts.push_back(*others.begin());
			usedVerts.insert(verts.back());

			vertToVertMap.erase(vertToVertMap.find(verts.front()));

			iter = vertToVertMap.find(verts.back());
			while (iter != vertToVertMap.end()) {
				const auto& others = iter->second;
				bool found = false;
				for (const auto& other : others) {
					if (!usedVerts.contains(other)) {
						verts.push_back(other);
						usedVerts.insert(verts.back());
						vertToVertMap.erase(iter);
						found = true;
						break;
					}
				}
				if (!found) {
					vertToVertMap.erase(vertToVertMap.find(verts.back()));
				}

				iter = vertToVertMap.find(verts.back());
			}
			if (!verts.empty()) {
				// make a face
				Index3DId newFaceId = getBlockPtr()->addFace(verts);
				newFaceIds.insert(newFaceId);
			}
		}
	});
}

void PolygonSplitter::createTrimmedFaceEdges(const MTC::set<Edge>& modFaceEdges, MTC::set<Edge>& trimEdges)
{
	faceFunc(TS_REAL, _polygonId, [&](const Polygon& realFace) {
		MTC::map<Index3DId, MTC::set<Edge>> vertModEdgeMap;
		for (const auto& edge : modFaceEdges) {
			assert(realFace.isCoplanar(edge));

			for (int i = 0; i < 2; i++) {
				const auto& vert = edge.getVertex(i);
				auto iter = vertModEdgeMap.insert(std::make_pair(vert, set<Edge>())).first;
				iter->second.insert(edge);
			}
		}

		const auto& vertIds = realFace.getVertexIds();
		for (size_t i = 0; i < vertIds.size(); i++) {
			size_t j = (i + 1) % vertIds.size();

			const auto& vertId0 = vertIds[i];
			const auto& vertId1 = vertIds[j];

			Vector3d pt0 = getBlockPtr()->getVertexPoint(vertId0);
			Vector3d pt1 = getBlockPtr()->getVertexPoint(vertId1);
			LineSegmentd seg(pt0, pt1);

			MTC::map<double, Index3DId> edgeIntersectionMap;

			for (const auto& modEdge : modFaceEdges) {
				for (int i = 0; i < 2; i++) {
					const auto& vert = modEdge.getVertex(i);
					Vector3d pt = getBlockPtr()->getVertexPoint(vert);
					double t;
					if (seg.contains(pt, t, Tolerance::sameDistTol())) {
						edgeIntersectionMap.insert(make_pair(t, vert));
					}
				}
			}

			if (edgeIntersectionMap.empty()) {
				trimEdges.insert(Edge(vertId0, vertId1));
				continue;
			}

			edgeIntersectionMap.insert(make_pair(0, vertId0));
			edgeIntersectionMap.insert(make_pair(1, vertId1));

			auto iter = edgeIntersectionMap.begin();

			while ((iter + 1) != edgeIntersectionMap.end()) {
				double t0 = iter->first;
				const auto& vertId0 = iter->second;

				double t1 = (iter + 1)->first;
				const auto& vertId1 = (iter + 1)->second;
				assert(t1 > t0);

				Vector3d pt0 = getBlockPtr()->getVertexPoint(vertId0);
				Vector3d pt1 = getBlockPtr()->getVertexPoint(vertId1);
				Vector3d v0 = pt1 - pt0;
				Vector3d v1 = -v0;

				Vector3d modelNorm0, modelNorm1;
				bool hasModelNorm0 = calModelNorm(vertModEdgeMap, vertId0, modelNorm0);
				bool hasModelNorm1 = calModelNorm(vertModEdgeMap, vertId1, modelNorm1);
				if (hasModelNorm0 && hasModelNorm1) {

					double dp0 = modelNorm0.dot(v0);
					double dp1 = modelNorm1.dot(v1);
					if ((dp0 > 0) && (dp1 > 0)) {
						trimEdges.insert(Edge(vertId0, vertId1));
					}
				} else if (hasModelNorm0) {
					double dp = modelNorm0.dot(v0);
					if (dp > 0) {
						trimEdges.insert(Edge(vertId0, vertId1));
					}
				} else if (hasModelNorm1) {
					double dp = modelNorm1.dot(v1);
					if (dp > 0) {
						trimEdges.insert(Edge(vertId0, vertId1));
					}
				}

				iter++;
			}
		}

		trimEdges.insert(modFaceEdges.begin(), modFaceEdges.end());
	});
}

bool PolygonSplitter::calModelNorm(const MTC::map<Index3DId, MTC::set<Edge>>& vertModEdgeMap, const Index3DId& vertId, Vector3d& modNorm) const
{
	auto modIter = vertModEdgeMap.find(vertId);
	if (modIter != vertModEdgeMap.end()) {
		const auto& edges = modIter->second;
		if (edges.size() != 1) {
			assert(!"Unexpected condition");
			return false;
		}

		const auto& modFaces = edges.begin()->getFaceIds();
		if (modFaces.size() != 1) {
			assert(!"Unexpected condition");
			return false;
		}

		const auto& modFaceId = *modFaces.begin();

		// multiple intersection with and edge
		faceFunc(TS_REAL, modFaceId, [&modNorm](const Polygon& face) {
			modNorm = face.calUnitNormal();
		});

		return true;
	}

	return false;
}

bool PolygonSplitter::createTrimmedEdge(const Edge& srcEdge, const Edge& cuttingEdge, Edge& newEdge)
{
	auto pMesh = getBlockPtr()->getModelMesh();
	const auto& vertId0 = srcEdge.getVertex(0);
	const auto& vertId1 = srcEdge.getVertex(1);

	const Vector3d imprintPt0 = getBlockPtr()->getVertexPoint(cuttingEdge.getVertex(0));
	const Vector3d imprintPt1 = getBlockPtr()->getVertexPoint(cuttingEdge.getVertex(1));

	const Vector3d vertPt0 = getBlockPtr()->getVertexPoint(vertId0);
	const Vector3d vertPt1 = getBlockPtr()->getVertexPoint(vertId1);
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
