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
				MTC::vector<MTC::vector<Index3DId>> faceVerts;
				createConvexFaceVerts(verts, faceVerts);

				// make a face
				for (const auto& verts : faceVerts) {
					Index3DId newFaceId = getBlockPtr()->addFace(verts);
					newFaceIds.insert(newFaceId);
				}
			}
		}
	});
}

namespace
{
void addEdgeToMap(MTC::map<Index3DId, MTC::set<Edge>>& vertEdgeMap, const Edge& edge) {
	for (int i = 0; i < 2; i++) {
		const auto& v = edge.getVertex(i);
		auto iter = vertEdgeMap.insert(std::make_pair(v, MTC::set<Edge> ())).first;
		iter->second.insert(edge);
	}
}

void createVertEdgeMap(Block* pBlock, const MTC::vector<Index3DId>& verts, MTC::map<Edge, size_t>& edges, MTC::map<Index3DId, MTC::set<Edge>>& vertEdgeMap)
{
	size_t numVerts = verts.size();
	for (size_t i = 0; i < numVerts; i++) {
		size_t j = (i + 1) % numVerts;
		Edge e(verts[i], verts[j]);
		edges.insert(std::make_pair(e, 1));
		addEdgeToMap(vertEdgeMap, e);
	}
}

bool getInwardVectors(Block* pBlock, const Index3DId& vertId, const MTC::map<Index3DId, MTC::set<Edge>>& vertEdgeMap,
	Vector3d& pt, Vector3d& v0, Vector3d& v1)
{
	auto iter = vertEdgeMap.find(vertId);
	if (iter == vertEdgeMap.end()) {
		assert(!"Should not be possible");
		return false;
	}

	const auto& ccVertEdges = iter->second;
	auto vIter = ccVertEdges.begin();
	const auto& e0 = *vIter++;
	const auto& e1 = *vIter;

	Vector3d pt0 = pBlock->getVertexPoint(e0.getOtherVert(vertId));
	pt = pBlock->getVertexPoint(vertId);
	Vector3d pt1 = pBlock->getVertexPoint(e1.getOtherVert(vertId));

	v0 = (pt - pt0).normalized();
	v1 = (pt - pt1).normalized();

	return true;
}

void createFacesVerts(const Block* pBlock, const Vector3d& faceNormal, MTC::map<Edge, size_t>& edgeCountMap, MTC::vector<MTC::vector<Index3DId>>& convexFaceVerts)
{
#if 1
	MTC::set<Edge> edges;
	for (const auto& pair : edgeCountMap) {
		edges.insert(pair.first);
	}
	PolygonSplitter::connectEdges(pBlock, edges, convexFaceVerts);
#else
	while (!edgeCountMap.empty()) {
		Edge seedEdge;
		bool useNormal = false;
		for (auto& pair : edgeCountMap) {
			if (pair.second == 2) {
				seedEdge = pair.first;
				useNormal = true;
				break;
			}
		}

		if (!seedEdge.isValid()) {
			seedEdge = edgeCountMap.begin()->first;
			edgeCountMap.erase(edgeCountMap.begin());
		}

		MTC::vector<Index3DId> faceVerts;
		MTC::set<Index3DId> usedVerts;

		faceVerts.push_back(seedEdge.getVertex(0));
		faceVerts.push_back(seedEdge.getVertex(1));

		usedVerts.insert(faceVerts.front());
		usedVerts.insert(faceVerts.back());

		bool found = true;
		while (found) {
			found = false;
			const auto& priorVertId = faceVerts[faceVerts.size() - 2];
			const auto& lastVertId = faceVerts[faceVerts.size() - 1];
			Vector3d pt0 = pBlock->getVertexPoint(priorVertId);
			Vector3d pt1 = pBlock->getVertexPoint(lastVertId);
			Vector3d v0 = (pt0 - pt1);
			for (auto& pair : edgeCountMap) {
				if (pair.second < 2) {
					const auto& edge = pair.first;
					if (edge.containsVertex(lastVertId)) {
						const auto& nextId = edge.getOtherVert(lastVertId);
						if (nextId == faceVerts.front())
							continue;

						if (!usedVerts.contains(nextId)) {
							Vector3d pt2 = pBlock->getVertexPoint(nextId);
							Vector3d v1 = pt2 - pt1;
							if (!useNormal || v1.cross(v0).dot(faceNormal) > 0) {
								found = true;
								usedVerts.insert(nextId);
								faceVerts.push_back(nextId);
								break;
							}
						}
					}
				}
			}
		}

		if (!faceVerts.empty()) {
			for (size_t i = 0; i < faceVerts.size(); i++) {
				size_t j = (i + 1) % faceVerts.size();
				auto iter = edgeCountMap.find(Edge(faceVerts[i], faceVerts[j]));
				if (iter != edgeCountMap.end()) {
					iter->second--;
					if (iter->second == 0) {
						edgeCountMap.erase(iter);
					}
				}
			}
			convexFaceVerts.push_back(faceVerts);
		}
	}
#endif
}

}

bool PolygonSplitter::createConvexFaceVerts(const MTC::vector<Index3DId>& verts, MTC::vector<MTC::vector<Index3DId>>& convexFaceVerts)
{
	const bool primaryAxisCuts = true;
	MTC::set<Index3DId> concaveVerts;
	Polygon::findConcaveVertIdsStat(getBlockPtr(), verts, concaveVerts);
	if (concaveVerts.empty()) {
		convexFaceVerts.push_back(verts);
		return true;
	}

	Vector3d origin, xAxis, yAxis, zAxis;
	Polygon::calCoordSysStat(getBlockPtr(), verts, origin, xAxis, yAxis, zAxis);

	MTC::map<Edge, size_t> edgeCountMap;
	MTC::map<Index3DId, MTC::set<Edge>> vertEdgeMap;
	createVertEdgeMap(getBlockPtr(), verts, edgeCountMap, vertEdgeMap);

	for (const auto& ccVertId : concaveVerts) {
		Vector3d ccPt, ccV[2];
		getInwardVectors(getBlockPtr(), ccVertId, vertEdgeMap, ccPt, ccV[0], ccV[1]);

		double maxDp = 0;
		Vector3d norm0, norm1; // This is normal to the cutting plane to find the intersection
		for (int i = 0; i < 2; i++) {
			double dpX = ccV[i].dot(xAxis);
			double dpY = ccV[i].dot(yAxis);
			if (fabs(dpX) > fabs(dpY)) {
				if (fabs(dpX) > maxDp) {
					if (primaryAxisCuts) {
						norm0 = xAxis;
						norm1 = yAxis;
					} else {
						norm0 = ccV[i];
						norm1 = ccV[1 - i];
					}
					maxDp = fabs(dpX);
				}
			} else if (fabs(dpY) > fabs(dpX)) {
				if (fabs(dpY) > maxDp) {
					if (primaryAxisCuts) {
						norm0 = yAxis;
						norm1 = zAxis;
					} else {
						norm0 = ccV[i];
						norm1 = ccV[1 - i];
					}
					maxDp = fabs(dpY);
				}
			}
		}
		double cp = norm0.cross(norm1).norm();
		MTC::vector<Planed> cuttingPlanes;
		if (cp > cos(M_PI / 4.0)) {
			// Two plane cut
			cuttingPlanes.push_back(Planed(ccPt, norm0));
			cuttingPlanes.push_back(Planed(ccPt, norm1));
		} else {
			// single plane cut
			Vector3d bisector = norm0 + norm1;
			Vector3d perpBisector = zAxis.cross(bisector).normalized();
			cuttingPlanes.push_back(Planed(ccPt, perpBisector));
		}

		for (const auto& cutPlane : cuttingPlanes) {
			MTC::set<Edge> newEdges, newSplitEdges, deadEdges;
			for (const auto& edgePair : edgeCountMap) {
				const auto& edge = edgePair.first;
				auto seg = edge.getSegment(getBlockPtr());
				RayHitd hit;
				if (cutPlane.intersectLineSegment(seg, hit, Tolerance::sameDistTol())) {
					double l = seg.calLength();
					double t = (hit.hitPt - seg._pts[0]).norm() / l;
					double conjT = 1 - t;
					if (t < 1.0e-9 || conjT < 1.0e-9)
						continue;

					double r = t > conjT ? t / conjT : conjT / t;
					if (r < 2.5) {
						deadEdges.insert(edge);
						Index3DId midVertId = getBlockPtr()->addVertex(hit.hitPt);
						newEdges.insert(Edge(edge.getVertex(0), midVertId));
						newEdges.insert(Edge(edge.getVertex(1), midVertId));
						newSplitEdges.insert(Edge(ccVertId, midVertId));
					}
				}
			}

			for (const auto& deadEdge : deadEdges) {
				auto iter = edgeCountMap.find(deadEdge);
				if (iter != edgeCountMap.end())
					edgeCountMap.erase(iter);
			}

			for (const auto& newEdge : newEdges)
				edgeCountMap.insert(std::make_pair(newEdge, 1));
			for (const auto& newSplitEdge : newSplitEdges)
				edgeCountMap.insert(std::make_pair(newSplitEdge, 2));
			
		}
	}

	{
		MTC::set<Edge> dmpEdges;
		for (const auto& e : edgeCountMap) {
			dmpEdges.insert(e.first);
		}
		std::string filename = "dmpEdges_" + getBlockPtr()->getLoggerNumericCode();
		getBlockPtr()->dumpEdgeObj(filename, dmpEdges);
	}

	createFacesVerts(getBlockPtr(), zAxis, edgeCountMap, convexFaceVerts);

	return true;
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

	return false;
}

namespace
{

bool isLoopClosed(const MTC::map<Index3DId, MTC::set<Index3DId>>& vertToNextVertMap, const MTC::vector<Index3DId>& verts)
{
	if (verts.size() > 2) {
		auto iter = vertToNextVertMap.find(verts.back());
		if (iter != vertToNextVertMap.end()) {
			const auto& nextVerts = iter->second;
			return nextVerts.contains(verts.front());
		}
	}
	return false;
}

bool extendLoop(const MTC::map<Index3DId, MTC::set<Index3DId>>& vertToNextVertMap, MTC::set<Index3DId>& usedVerts, MTC::vector<Index3DId>& verts)
{
	auto iter = vertToNextVertMap.find(verts.back());
	if (iter == vertToNextVertMap.end())
		return false;

	const auto& nextVerts = iter->second;
	MTC::vector<Index3DId> bestBranch;
	for (const auto& nextVert : nextVerts) {
		if (!usedVerts.contains(nextVert)) {
			MTC::set<Index3DId> usedChainVerts(usedVerts);
			MTC::vector<Index3DId> chainVerts(verts);
			usedChainVerts.insert(nextVert);
			chainVerts.push_back(nextVert);
			if (isLoopClosed(vertToNextVertMap, chainVerts)) {
				if (bestBranch.empty() || chainVerts.size() < bestBranch.size())
					bestBranch = chainVerts;
			} else if (extendLoop(vertToNextVertMap, usedChainVerts, chainVerts)) {
				if (bestBranch.empty() || chainVerts.size() < bestBranch.size())
					bestBranch = chainVerts;
			}
		}
	}

	if (!bestBranch.empty()) {
		verts = bestBranch;
		return true;
	}

	return false;
}

}

bool PolygonSplitter::connectEdges(const Block* pBlock, const MTC::set<Edge>& edges, MTC::vector<MTC::vector<Index3DId>>& faceVertices, bool isIntersection)
{
	if (edges.size() < 3)
		return false;

	faceVertices.clear();

#if 1
	MTC::map<Index3DId, MTC::set<Index3DId>> vertToNextVertMap;
	for (const auto& edge : edges) {
		auto iter = vertToNextVertMap.insert(std::make_pair(edge.getVertex(0), MTC::set<Index3DId>())).first;
		iter->second.insert(edge.getVertex(1));

		iter = vertToNextVertMap.insert(std::make_pair(edge.getVertex(1), MTC::set<Index3DId>())).first;
		iter->second.insert(edge.getVertex(0));
	}

	MTC::set<Edge> sharedEdges;
	for (const auto& edge : edges) {
		auto iter = vertToNextVertMap.insert(std::make_pair(edge.getVertex(0), MTC::set<Index3DId>())).first;
		size_t count0 = iter->second.size();

		iter = vertToNextVertMap.insert(std::make_pair(edge.getVertex(1), MTC::set<Index3DId>())).first;
		size_t count1 = iter->second.size();
		if (count0 > 2 && count1 > 2)
			sharedEdges.insert(edge);
	}

	if (sharedEdges.empty()) {
		MTC::set<Index3DId> usedVerts;
		MTC::vector<Index3DId> verts;

		Index3DId firstVert = vertToNextVertMap.begin()->first;

		usedVerts.insert(firstVert);
		verts.push_back(firstVert);

		if (extendLoop(vertToNextVertMap, usedVerts, verts)) {
			faceVertices.push_back(verts);
		}
	} else {
		MTC::set<Index3DId> usedVerts;
		for (const auto& edge : sharedEdges) {
			usedVerts.insert(edge.getVertex(0));
			usedVerts.insert(edge.getVertex(1));

			for (int i = 0; i < 2; i++) {
				MTC::vector<Index3DId> verts;
				verts.push_back(edge.getVertex(0));
				verts.push_back(edge.getVertex(1));

				if (extendLoop(vertToNextVertMap, usedVerts, verts)) {
					usedVerts.insert(verts.begin(), verts.end());
					faceVertices.push_back(verts);
					if (usedVerts.size() == vertToNextVertMap.size())
						break;
				}
			}
		}
	}

#else
	assert(!"TODO, make this work with nonmanifold edges and multiple faces. Currently only works on a single face.");
	MTC::set<Edge> edges(edgesIn);
	MTC::vector<Index3DId> verts;
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

	MTC::vector<Index3DId> vertices;
	for (const auto& v : verts)
		vertices.insert(vertices.end(), v);
	if (!vertices.empty())
		faceVertices.push_back(vertices);
#endif
	return !faceVertices.empty();
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
