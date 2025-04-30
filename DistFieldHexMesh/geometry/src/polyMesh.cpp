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

#include <tm_ioUtil.h>
#include <tm_spatialSearch.hpp>
#include <tm_math.h>

#include <tolerances.h>
#include <index3D.h>
#include <splitParams.h>
#include <appData.h>
#include <vertex.h>
#include <edge.h>
#include <polygon.h>
#include <polyMesh.h>
#include <logger.h>
#include <appData.h>

using namespace std;
using namespace DFHM;

#define MEM_STORE_SCALE 10

PolyMesh::PolyMesh(const AppDataPtr& pAppData)
: _pAppData(pAppData)
, _vertices(this, true, MEM_STORE_SCALE * 8 * 8)
, _polygons(this, true, MEM_STORE_SCALE * 8 * 6)
{

}

PolyMesh::PolyMesh(const AppDataPtr& pAppData, const TriMesh::CMeshPtr& srcMesh)
	: PolyMesh(pAppData)
{
	size_t nTris = srcMesh->numTris();
	for (size_t i = 0; i < nTris; i++) {
		auto triIndices = srcMesh->getTri(i);
		Polygon newFace;
		for (int i = 0; i < 3; i++) {
			const auto& pt = srcMesh->getVert(triIndices[i])._pt;
			auto vertId = _vertices.findOrAdd(pt);
			newFace.addVertex(vertId);
		}
		_polygons.findOrAdd(newFace);
	}
}

PolyMesh::~PolyMesh()
{

}

const Index3D& PolyMesh::getBlockIdx() const
{
	static Index3D result(0, 0, 0);
	return result;
}

Volume* PolyMesh::getVolume()
{
	return nullptr;
}

const Volume* PolyMesh::getVolume() const
{
	return nullptr;
}

const Block* PolyMesh::getOwner(const Index3D& blockIdx) const
{
	return nullptr;
}

Block* PolyMesh::getOwner(const Index3D& blockIdx)
{
	return nullptr;
}

const PolyMesh* PolyMesh::getPolyMeshPtr() const
{
	return this;
}

PolyMesh* PolyMesh::getPolyMeshPtr()
{
	return this;
}

const Vector3d& PolyMesh::getVertexPoint(const Index3DId& id) const
{
	return _vertices[id];
}

void PolyMesh::makeQuads(const SplittingParams& params)
{
	auto sortLenFunc = [this](const EdgeKey& lhs, const EdgeKey& rhs) {
		double l0, l1;
		edgeFunc(lhs, [&l0](const Edge& edge) {
			l0 = edge.calLength();
		});
		edgeFunc(rhs, [&l1](const Edge& edge) {
			l1 = edge.calLength();
		});

		return l0 > l1;
	};

	vector<EdgeKey> edges;
	_polygons.iterateInOrder([&edges](const Index3DId& faceId, const Polygon& face)->bool {
		face.iterateEdges([&edges](const Edge& edge) {
			edges.push_back(edge);
			return true;
		});
		return true;
	});

	sort(edges.begin(), edges.end(), sortLenFunc);

	for (const auto& ek : edges) {
		edgeFunc(ek, [this, &params](const Edge& edge) {
			mergeToQuad(params, edge);
		});
	}
	int dbgBreak = 1;
}

void PolyMesh::reduceSlivers(const SplittingParams& params, double maxSliverAngleRadians)
{
	vector<Index3DId> orderedVertIds;
	_vertices.iterateInOrder([this, &orderedVertIds](const Index3DId& radiantId, const Vertex& vert)->bool {
		orderedVertIds.push_back(radiantId);
		return true;
	});

	sort(orderedVertIds.begin(), orderedVertIds.end(), [this](const Index3DId& lhsId, const Index3DId& rhsId)->bool {
		const auto& lhsVert = getVertex(lhsId);
		const auto& rhsVert = getVertex(rhsId);
		return lhsVert.getFaceIds().size() > rhsVert.getFaceIds().size();
	});

	for (const auto& radiantId : orderedVertIds) {
		const auto& vert = getVertex(radiantId);
		if (radiantId == Index3DId(0, 0, 0, 113)) {
			int dbgBreak = 1;
		}
		MTC::vector<MTC::vector<Index3DId>> planarFaceSets;
		makeCoplanarFaceSets(vert.getFaceIds(), planarFaceSets);
		for (const auto& faceIds : planarFaceSets) {
			processPlanarFaces(params, radiantId, maxSliverAngleRadians, faceIds);
		}
	}
}

void PolyMesh::processPlanarFaces(const SplittingParams& params, const Index3DId& radiantId, double minAngleRadians, const MTC::vector<Index3DId>& faceIds)
{
	const auto& radiantVert = getVertex(radiantId);
	vector<Index3DId> orderedFaceIds(faceIds.begin(), faceIds.end());
	set<Index3DId> faceIdSet(faceIds.begin(), faceIds.end()), sharedVerts;

	// ordered largest to smallest by area
	sort(orderedFaceIds.begin(), orderedFaceIds.end(), [this, &radiantId](const Index3DId& lhsId, const Index3DId& rhsId)->bool {
		const auto& lhsFace = getPolygon(lhsId);
		const auto& rhsFace = getPolygon(rhsId);

		double lhsVertAngle = lhsFace.calVertexAngle(radiantId);
		double rhsVertAngle = rhsFace.calVertexAngle(radiantId);

		return lhsVertAngle < rhsVertAngle;
	});

	const auto& largestFace = getPolygon(orderedFaceIds.back());
	double largestAngle = largestFace.calVertexAngle(radiantId);
	auto basePlane = largestFace.calPlane();

	set<Index3DId> connectedVertIds;
	for (size_t i = 0; i < faceIds.size(); i++) {
		const auto& face = getPolygon(faceIds[i]);
		face.iterateEdges([this, &basePlane, &radiantId, &connectedVertIds, &faceIdSet, &sharedVerts](const Edge& edge)->bool {
			auto id = edge.getOtherVert(radiantId);
			if (id.isValid()) {
				connectedVertIds.insert(id);

				const auto& adjFaceIds = edge.getFaceIds();
				int numAdj = 0;
				for (const auto& adjFaceId : adjFaceIds) {
					if (faceIdSet.contains(adjFaceId))
						numAdj++;
				}
				if (numAdj == 2)
					sharedVerts.insert(id);
			}
			return true;
		});
	}

	if (connectedVertIds.empty())
		return;

	vector<Index3DId> orderedVertIds(connectedVertIds.begin(), connectedVertIds.end());
	auto axisId = orderedVertIds.front();

	const auto& face = getPolygon(faceIds.front());
	auto zAxis = face.calUnitNormal();
	const auto& origin = radiantVert.getPoint();
	const auto& pt1 = getVertexPoint(axisId);
	Vector3d xAxis = (pt1 - origin).normalized();
	Vector3d yAxis = zAxis.cross(xAxis);

	vector<pair<double, Index3DId>> planarAngleEdgeMap;
	for (const auto& vertId : orderedVertIds) {
		double angle = calEdgeAngle(vertId, origin, xAxis, yAxis);
		planarAngleEdgeMap.push_back(make_pair(angle, vertId));
	}

	if (planarAngleEdgeMap.size() < 2)
		return;

	sort(planarAngleEdgeMap.begin(), planarAngleEdgeMap.end(), [](const pair<double, Index3DId>& lhs, const pair<double, Index3DId>& rhs)->bool {
		return lhs.first < rhs.first;
	});
	
	auto lastAngle = planarAngleEdgeMap.back().first;
	for (const auto& pair : planarAngleEdgeMap) {
		auto thisAngle = pair.first;
		const auto& thisId = pair.second;
		auto deltaAngle = thisAngle - lastAngle;
		if (deltaAngle < 0) {
			deltaAngle += 2 * M_PI;
		}

		if (deltaAngle > minAngleRadians || !sharedVerts.contains(thisId)) {
			lastAngle = thisAngle;
		}
		else {
			auto newFaceId = removeEdge(params, basePlane, radiantId, thisId);
			if (!newFaceId.isValid()) {
				lastAngle = thisAngle;
			}
		}
	}

}

Index3DId PolyMesh::removeEdge(const SplittingParams& params, const Planed& plane, const Index3DId& radiantVertId, const Index3DId& otherVertId)
{
	const double MAX_CP = 0.02;
	const double MAX_CP_SQR = MAX_CP * MAX_CP;
	const size_t maxVerts = 30;

	Index3DId result;
	EdgeKey key(radiantVertId, otherVertId);
	edgeFunc(key, [this, &plane, &params, &radiantVertId, &result](const Edge& edge) {
		const auto& faceIds = edge.getFaceIds();
		if (faceIds.size() != 2)
			return;

		auto iter = faceIds.begin();

		const auto& faceId0 = *iter++;
		const auto& faceId1 = *iter;

		auto& face0 = getPolygon(faceId0);
		auto& face1 = getPolygon(faceId1);

		if (isShortEdge(edge, face0, face1))
			return;

		auto norm0 = face0.calUnitNormal();
		auto norm1 = face1.calUnitNormal();
		if (norm0.dot(norm1) < 0)
			return;

		const auto& vertIds0 = face0.getVertexIds();
		const auto& vertIds1 = face1.getVertexIds();

		map<Index3DId, set<Index3DId>> vertToVertMap;
		for (size_t i = 0; i < vertIds0.size(); i++) {
			size_t j = (i + 1) % vertIds0.size();
			const auto& id0 = vertIds0[i];
			const auto& id1 = vertIds0[j];

			if (edge != EdgeKey(id0, id1)) {
				auto iter = vertToVertMap.find(id0);
				if (iter == vertToVertMap.end())
					iter = vertToVertMap.insert(make_pair(id0, set<Index3DId>())).first;
				iter->second.insert(id1);

				iter = vertToVertMap.find(id1);
				if (iter == vertToVertMap.end())
					iter = vertToVertMap.insert(make_pair(id1, set<Index3DId>())).first;
				iter->second.insert(id0);
			}
		}

		for (size_t i = 0; i < vertIds1.size(); i++) {
			size_t j = (i + 1) % vertIds1.size();
			const auto& id0 = vertIds1[i];
			const auto& id1 = vertIds1[j];

			if (edge != EdgeKey(id0, id1)) {
				auto iter = vertToVertMap.find(id0);
				if (iter == vertToVertMap.end())
					iter = vertToVertMap.insert(make_pair(id0, set<Index3DId>())).first;
				iter->second.insert(id1);

				iter = vertToVertMap.find(id1);
				if (iter == vertToVertMap.end())
					iter = vertToVertMap.insert(make_pair(id1, set<Index3DId>())).first;
				iter->second.insert(id0);
			}
		}


		MTC::vector<Index3DId> newVertIds;
		while (!vertToVertMap.empty()) {
			if (newVertIds.empty()) {
				auto iter = vertToVertMap.find(radiantVertId);
				auto iter2 = iter->second.begin();
				const auto& id0 = iter->first;
				const auto& id1 = *iter2;
				newVertIds.push_back(id0);
				newVertIds.push_back(id1);

				iter = vertToVertMap.find(newVertIds.front());
				iter->second.erase(newVertIds.back());

				iter = vertToVertMap.find(newVertIds.back());
				iter->second.erase(newVertIds.front());

				vertToVertMap.erase(newVertIds.front());
			}
			else {
				auto curBack = newVertIds.back();
				auto iter = vertToVertMap.find(curBack);
				if (iter->second.size() != 1)
					return; // This can happen when removing edges from a full circle. That's not allowed

				auto iter2 = iter->second.begin();
				newVertIds.push_back(*iter2);
				vertToVertMap.erase(curBack);
				iter = vertToVertMap.find(newVertIds.back());
				if (iter != vertToVertMap.end())
					iter->second.erase(curBack);
			}
		}

		if (newVertIds.front() == newVertIds.back())
			newVertIds.pop_back();

		if (!hasValidRotation(newVertIds, norm0))
			return;

		// It's 50/50 that the new vertices are reversed. Test the new normal with the prior normal(s) and reverse if needed
		Vector3d n2;
		if (!Polygon::calUnitNormalStat(this, newVertIds, n2))
			return;

		if (n2.dot(norm0) < 0) {
			// DO NOT USE - std::reverse(newVertIds.begin(), newVertIds.end());
			// Index 0 must remain at index 0

			MTC::vector<Index3DId> tmp(newVertIds);
			newVertIds.clear();
			for (size_t i = 0; i < tmp.size(); i++) {
				size_t j = (i == 0) ? i : (tmp.size() - i);
				newVertIds.push_back(tmp[j]);
			}

		}

		Vector3d n;
		if (!Polygon::calUnitNormalStat(this, newVertIds, n))
			assert("calUnitNormalStat failed");
		assert(norm0.dot(n) > 0);
		assert(norm1.dot(n) > 0);

		if (hasHighLocalConvexity(params, norm0, newVertIds))
			return;

		// Run through all possible rotations

		if (hasValidRotation(newVertIds, norm0)) {
			removeFace(faceId0);
			removeFace(faceId1);
			Polygon newFace(newVertIds);
			result = _polygons.findOrAdd(newFace);
			if (result == Index3DId(0, 0, 0, 113)) {
				int dbgBreak = 1;
			}
			auto& face = getPolygon(result);
#ifdef _DEBUG
			{
				auto n = face.calUnitNormal();
				assert(n.squaredNorm() > 0);
				assert(norm0.dot(n) > 0);
				assert(norm1.dot(n) > 0);
			}
#endif
			// For this new face to have the plane of the source faces
			face.setCentroid_risky(plane.getOrgin());
			face.setUnitNormal_risky(plane.getNormal());
			face.setIsConvex_risky(Polygon::IS_CONVEX_ENOUGH);
		}
	});

	return result;
}

bool PolyMesh::hasValidRotation(MTC::vector<Index3DId>& vertIds, const Vector3d& norm) const
{
	// Brute force rotation and test of all vertices to assure that all fan triangles have correctly oriented normals.
	// The prevents "drawing out of bounds" when retessellating for rendering and correct intersection for raycasting and curvature calculations.
	// If vertIds is constructed with the radiant vertex of the fan in position zero, no rotations will be required.
	// The loop is a safety net for cases that don't match that rule.
	for (size_t i = 0; i < vertIds.size(); i++) {
		bool isValid = formsValidPolygon(vertIds, norm);
		if (isValid)
			return true;
		rotate(vertIds.begin(), vertIds.begin() + 1, vertIds.end());
	}

	return false;
}

bool PolyMesh::formsValidPolygon(const MTC::vector<Index3DId>& vertIds, const Vector3d& norm) const
{
	vector<Vector3d> pts;
	for (const auto& id : vertIds) {
		pts.push_back(getVertexPoint(id));
	}

	Vector3d n;
	const auto& pt0 = pts[0];
	for (size_t i = 1; i < pts.size(); i++) {
		size_t j = (i + 1) % pts.size();
		size_t k = (i + 2) % pts.size();
		const auto& pt1 = pts[j];
		const auto& pt2 = pts[k];

		auto v0 = pt0 - pt1;
		auto v1 = pt2 - pt1;
		n = v1.cross(v0);
		if (norm.dot(n) < 0)
			return false;
	}

	return Polygon::calUnitNormalStat(this, vertIds, n);
}

EdgeKey PolyMesh::findCommonEdge(const Polygon& a, const Polygon& b)
{
	EdgeKey result;
	a.iterateEdges([&b, &result](const Edge& edgeA)->bool {
		b.iterateEdges([&edgeA, &result](const Edge& edgeB)->bool {
			if (edgeA == edgeB) {
				result = edgeA;
			}
			return !result.isValid();
		});
		return !result.isValid();
	});

	return result;
}

bool PolyMesh::isShortEdge(const Edge& edge, const Polygon& face0, const Polygon& face1) const
{
	auto edgeLength = edge.calLength();
	double area0, area1;
	Vector3d ctr0, ctr1;
	face0.calAreaAndCentroid(area0, ctr0);
	face1.calAreaAndCentroid(area1, ctr1);

	// Edge length of a square of equivalent area
	auto sqrEdgeLen = sqrt(area0 + area1);
	// Don't merge a short edge
	return edgeLength < sqrEdgeLen;
}

bool PolyMesh::hasHighLocalConvexity(const SplittingParams& params, const Vector3d& norm, const MTC::vector<Index3DId>& vertIds) const
{
	vector<const Vector3d*> pts;
	pts.resize(vertIds.size());
	for (size_t i = 0; i < vertIds.size(); i++)
		pts[i] = &getVertexPoint(vertIds[i]);

	for (size_t i = 0; i < vertIds.size(); i++) {
		size_t j = (i + 1) % vertIds.size();
		size_t k = (i + 2) % vertIds.size();

		const auto& pt0 = *pts[i];
		const auto& pt1 = *pts[j];
		const auto& pt2 = *pts[k];

		Vector3d v0 = (pt0 - pt1).normalized();
		Vector3d v1 = (pt2 - pt1).normalized();
		Vector3d n = v1.cross(v0);
		double cp = n.dot(norm);
		if (cp < params.maxLocalConcavityCrossProduct)
			return true;
	}
	return false;
}

double PolyMesh::calEdgeAngle(const Index3DId& vertId, const Vector3d& origin, const Vector3d& xAxis, const Vector3d& yAxis) const
{
	auto& pt = getVertexPoint(vertId);
	Vector3d v = pt - origin;
	double x = v.dot(xAxis);
	double y = v.dot(yAxis);
	double angle = atan2(y, x);
	return angle;
}

void PolyMesh::makeCoplanarFaceSets(const FastBisectionSet<Index3DId>& faceIds, MTC::vector<MTC::vector<Index3DId>>& planarFaceSets)
{
#if 1
	const double sameDistTol = Tolerance::sameDistTolFloat();

	// There's a potential hole in the algorithm if there is a coplanar face which is not edge adjacent with the other members of the set. 
	// Not sure that's possible, and it's computationally expensive to remove.
	// It should also have no effect on the outcome of edge removal.
	// If it shows up, then plug the hole.

	if (faceIds.size() > 50) {
		int dbgBreak = 1;
	}
	vector<Index3DId> faceIdsAscendingArea(faceIds.begin(), faceIds.end());
	sort(faceIdsAscendingArea.begin(), faceIdsAscendingArea.end(), [this](const Index3DId& lhsId, const Index3DId& rhsId) {
		const auto& lhsFace = getPolygon(lhsId);
		const auto& rhsFace = getPolygon(rhsId);

		double lhsArea, rhsArea;
		Vector3d discarded;
		lhsFace.calAreaAndCentroid(lhsArea, discarded);
		rhsFace.calAreaAndCentroid(rhsArea, discarded);
		return lhsArea < rhsArea;
	});

	MTC::vector<Index3DId> currentFaces;
	Vector3d weightedNormal, groupOrigin;
	double totalArea = 0;
	double currentArea;
	Vector3d discarded;
	while (!faceIdsAscendingArea.empty()) {
		if (currentFaces.empty()) {
			currentFaces.push_back(faceIdsAscendingArea.back());
			faceIdsAscendingArea.pop_back();
			const auto& face = getPolygon(currentFaces.back());
			face.calAreaAndCentroid(currentArea, groupOrigin);
			weightedNormal = currentArea * face.calUnitNormal();
			totalArea += currentArea;
		}

		double maxArea = 0;
		size_t bestFaceIdx = -1;
		Vector3d bestNormal;
		Planed groupPlane(groupOrigin, weightedNormal);
		for (size_t i = 0; i < faceIdsAscendingArea.size(); i++) {
			const auto& currentFace = getPolygon(faceIdsAscendingArea[i]);
			auto currentFaceNorm = currentFace.calUnitNormal();
			if (weightedNormal.dot(currentFaceNorm) > 0) {
				const auto& vertIds = currentFace.getVertexIds();
				bool allVertsCoplanar = true;
				for (const auto& vertId : vertIds) {
					const auto& pt = getVertexPoint(vertId);
					if (!groupPlane.isCoincident(pt, sameDistTol)) {
						allVertsCoplanar = false;
						break;
					}
				}

				if (allVertsCoplanar) {
					currentFace.calAreaAndCentroid(currentArea, discarded);
					if (currentArea > maxArea) {
						maxArea = currentArea;
						bestNormal = currentFaceNorm;
						bestFaceIdx = i;
					}
				}
			}
		}

		if (bestFaceIdx != -1) {
			currentFaces.push_back(faceIdsAscendingArea[bestFaceIdx]);
			faceIdsAscendingArea.erase(faceIdsAscendingArea.begin() + bestFaceIdx);
			assert(bestNormal.dot(weightedNormal) > 0);
			weightedNormal += maxArea * bestNormal;
			totalArea += maxArea;
		} else {
			if (currentFaces.size() > 1)
				planarFaceSets.push_back(currentFaces);
			currentFaces.clear();
		}
	}
#else
	set<size_t> usedIndices;
	for (size_t i = 0; i < faceIds.size(); i++) {
		if (usedIndices.contains(i))
			continue;
		usedIndices.insert(i);
		MTC::vector<Index3DId> thisSet;
		const auto& faceId = faceIds[i];
		thisSet.push_back(faceId);
		const auto& baseFace = getPolygon(faceId);
		auto basePlane = baseFace.calPlane();

		for (size_t j = i + 1; j < faceIds.size(); j++) {
			if (usedIndices.contains(j))
				continue;
			const auto& thisFace = getPolygon(faceIds[j]);
			auto thisPlane = thisFace.calPlane();
			if (basePlane.isCoincident(thisPlane, Tolerance::planeCoincidentDistTol(), Tolerance::planeCoincidentCrossProductTol())) {
				thisSet.push_back(faceIds[j]);
				usedIndices.insert(j);
			}
		}

		if (thisSet.size() > 2) {
			planarFaceSets.push_back(thisSet);
		}
	}
#endif
}

void PolyMesh::calCurvatures()
{
	_edgeCurvatures.clear();

	_polygons.iterateInOrder([this](const Index3DId& faceId, const Polygon& face)->bool {
		face.iterateEdges([this](const Edge& edge) {
			const auto& params = _pAppData->getParams();
			auto c = edge.calCurvature(params);
			_edgeCurvatures.insert(make_pair(edge, c));
			return true;
		});
		return true;
	});
}

void PolyMesh::mergeToQuad(const SplittingParams& params, const Edge& edge)
{
	const double MAX_CP = 0.02;
	const double MAX_CP_SQR = MAX_CP * MAX_CP;

	auto& faceIds = edge.getFaceIds();
	if (faceIds.size() != 2)
		return;

	auto iter = faceIds.begin();
	auto faceId0 = *iter++;
	auto faceId1 = *iter;

	if (!_polygons.exists(faceId0) || !_polygons.exists(faceId1))
		return;

	auto& face0 = getPolygon(faceId0);
	auto& face1 = getPolygon(faceId1);
	if (isShortEdge(edge, face0, face1))
		return;

	auto vertIds0 = face0.getVertexIds();
	auto vertIds1 = face1.getVertexIds();

	auto ncLin0 = PolygonSearchKey::makeNonColinearVertexIds(this, vertIds0);
	auto ncLin1 = PolygonSearchKey::makeNonColinearVertexIds(this, vertIds1);
	if (ncLin0.size() != 3 || ncLin1.size() != 3)
		return;

	// Check for close to coplanar
	Vector3d norm0, norm1;
	if (!Polygon::calUnitNormalStat(this, ncLin0, norm0))
		return;
	if (!Polygon::calUnitNormalStat(this, ncLin1, norm1))
		return;

	auto cp = norm0.cross(norm1).squaredNorm();
	if (cp > MAX_CP_SQR)
		return;

	Index3DId otherId0;
	for (const auto& id : vertIds0) {
		if (!edge.containsVertex(id)) {
			otherId0 = id;
			break;
		}
	}
	if (!otherId0.isValid())
		return;


	Index3DId otherId1;
	for (const auto& id : vertIds1) {
		if (!edge.containsVertex(id)) {
			otherId1 = id;
			break;
		}
	}
	if (!otherId1.isValid())
		return;

	const auto& ePt0 = getVertexPoint(edge[0]);
	const auto& ePt1 = getVertexPoint(edge[1]);
	const auto& pt0 = getVertexPoint(otherId0);
	const auto& pt1 = getVertexPoint(otherId1);
	LineSegmentd seg(ePt0, ePt1);
	double t;

	double deltaT = 0.25;
	if (seg.distanceToPoint(pt0, t) < Tolerance::sameDistTol() || t < -deltaT || t > 1 + deltaT)
		return;

	if (seg.distanceToPoint(pt1, t) < Tolerance::sameDistTol() || t < -deltaT || t > 1 + deltaT)
		return;

#if 1 && defined(_DEBUG)
	if (faceId0 == Index3DId(0, 0, 0, 8047)) {
		int dbgBreak = 1;
	}
#endif
	double area0, area1;
	Vector3d ctr0, ctr1;
	face0.calAreaAndCentroid(area0, ctr0);
	face1.calAreaAndCentroid(area1, ctr1);
	Vector3d ctr = (ctr0 * area0 + ctr1 * area1) / (area0 + area1);
	Vector3d norm = face0.calUnitNormal() + face1.calUnitNormal();
	norm.normalize();
#if 1
	Planed plane(ctr, norm);
	removeEdge(params, plane, edge[0], edge[1]);
#else
	MTC::vector<Index3DId> newVertIds;
	for (size_t i = 0; i < vertIds0.size(); i++) {
		size_t j = (i + 1) % vertIds0.size();
		newVertIds.push_back(vertIds0[i]);
		if (edge.containsVertex(vertIds0[i]) && edge.containsVertex(vertIds0[j])) {
			newVertIds.push_back(otherId1);
		}
	}

	if (hasValidRotation(newVertIds, norm)) {
		removeFace(faceId0);
		removeFace(faceId1);
		Polygon newFace(newVertIds);
		auto newFaceId = _polygons.findOrAdd(newFace);
		auto& face = getPolygon(newFaceId);
#ifdef _DEBUG
		{
			auto n = face.calUnitNormal();
			assert(n.dot(norm0) > 0);
			assert(n.dot(norm1) > 0);
		}
#endif // _DEBUG

		// For this new face to have the plane of the source faces
		face.setIsConvex_risky(Polygon::IS_CONVEX_ENOUGH);
	}
#endif
}

void PolyMesh::removeFace(const Index3DId& id)
{
	_polygons[id].disconnectVertEdgeTopology();
	_polygons.removeFromLookup(id);
	_polygons.free(id);
}

std::vector<float> PolyMesh::getGlTriPoints() const
{
	std::vector<float> result;
	_polygons.iterateInOrder([this, &result](const Index3DId& id, const Polygon& face)->bool {
		face.iterateTriangles([this, &result](const Index3DId& id0, const Index3DId& id1, const Index3DId& id2)->bool {
			const Vector3d* pts[] = {
				&getVertexPoint(id0),
				&getVertexPoint(id1),
				&getVertexPoint(id2),
			};

			for (int i = 0; i < 3; i++) {
				const auto& pt = *pts[i];
				for (int j = 0; j < 3; j++) {
					result.push_back((float)pt[j]);
				}
			}
			return true;
		});
		return true;
	});
	return result;
}

std::vector<float> PolyMesh::getGlTriNormals() const
{
	std::vector<float> result;
	_polygons.iterateInOrder([this, &result](const Index3DId& id, const Polygon& face)->bool {
		Vector3d norm = face.calUnitNormal();
		face.iterateTriangles([this, &result, &norm](const Index3DId& id0, const Index3DId& id1, const Index3DId& id2)->bool {
			for (int i = 0; i < 3; i++) {
				for (int j = 0; j < 3; j++) {
					result.push_back((float)norm[j]);
				}
			}
			return true;
		});
		return true;
	});
	return result;
}

std::vector<unsigned int> PolyMesh::getGlTriIndices() const
{
	std::vector<unsigned int> result;
	unsigned int idx = 0;
	_polygons.iterateInOrder([this, &result, &idx](const Index3DId& id, const Polygon& face)->bool {
		face.iterateTriangles([this, &result, &idx](const Index3DId& id0, const Index3DId& id1, const Index3DId& id2)->bool {
			for (int i = 0; i < 3; i++) {
				result.push_back(idx++);
			}
			return true;
		});
		return true;
	});
	return result;
}


void PolyMesh::dumpFaceSetAsObj(const std::string& path, size_t num, const Index3DId& radiantId, const MTC::vector<Index3DId>& faceIds, const std::set<Index3DId>& sharedVerts)
{
	vector<Index3DId> verts;
	map<Index3DId, size_t> ptToIndexMap;
	set<EdgeKey> edges;

	stringstream ss;
	ss << "vert_" << num << "_";
	string prefix = ss.str();
	verts.push_back(radiantId);
	ptToIndexMap.insert(make_pair(radiantId, 0));

	for (const auto& faceId : faceIds) {
		const auto& face = getPolygon(faceId);
		const auto& vertIds = face.getVertexIds();
		for (size_t i = 0; i < vertIds.size(); i++) {
			size_t j = (i + 1) % vertIds.size();
			auto iter = ptToIndexMap.find(vertIds[i]);
			if (iter == ptToIndexMap.end()) {
				size_t newIdx = verts.size();
				verts.push_back(vertIds[i]);
				ptToIndexMap.insert(make_pair(vertIds[i], newIdx));
			}
			edges.insert(EdgeKey(vertIds[i], vertIds[j]));
		}
	}

	{
		ofstream out(path + "/" + prefix + "planarFaceGroup.obj");
		for (const auto& id : verts) {
			const auto& pt = getVertexPoint(id);
			out << "v " << pt[0] << " " << pt[1] << " " << pt[2] << "\n";
		}

		for (const auto& id : faceIds) {
			const auto& face = getPolygon(id);
			const auto& vertIds = face.getVertexIds();
			out << "f";
			for (const auto& id : vertIds) {
				auto iter = ptToIndexMap.find(id);
				size_t idx = iter->second + 1;
				out << " " << idx;
			}
			out << "\n";
		}
	}

	{
		ofstream out(path + "/" + prefix + "removableEdges.obj");
		for (const auto& id : verts) {
			const auto& pt = getVertexPoint(id);
			out << "v " << pt[0] << " " << pt[1] << " " << pt[2] << "\n";
		}

		for (const auto& id : sharedVerts) {
			size_t idx0 = 1;
			auto iter = ptToIndexMap.find(id);
			size_t idx1 = iter->second + 1;
			out << "l " << idx0 << " " << idx1 << "\n";
		}
	}
}

#define FUNC_IMPL(NAME, KEY, MEMBER_NAME, CONST, CLASS) \
void PolyMesh::NAME##Func(const KEY& id, const function<void(CONST CLASS& obj)>& func) CONST \
{ \
	func(MEMBER_NAME[id]); \
}

#define POINTER_FUNC_IMPL(KEY, MEMBER_NAME, CONST, CLASS) \
CONST DFHM::CLASS& PolyMesh::get##CLASS(const KEY& id) CONST \
{ \
	return MEMBER_NAME[id]; \
}

#define IMPLS(NAME, KEY, MEMBER_NAME, CLASS) \
FUNC_IMPL(NAME, KEY, MEMBER_NAME, const, CLASS) \
FUNC_IMPL(NAME, KEY, MEMBER_NAME, , CLASS) \
POINTER_FUNC_IMPL(KEY, MEMBER_NAME, const, CLASS) \
POINTER_FUNC_IMPL(KEY, MEMBER_NAME, , CLASS) 

#define EDGE_IMPL(NAME, KEY, CONST) \
void PolyMesh::NAME##Func(const KEY& key, const function<void(CONST Edge& obj)>& func) CONST \
{ \
	if (_vertices.exists(key[0]) && _vertices.exists(key[1])) { \
		Edge edge(key, this); \
		func(edge); \
	} \
}

#define EDGE_IMPLS(NAME, KEY) \
EDGE_IMPL(NAME, KEY, const) \
EDGE_IMPL(NAME, KEY,)

#if 0
IMPLS(vertex, Index3DId, _vertices, Vertex)
IMPLS(face, Index3DId, _polygons, Polygon)
EDGE_IMPLS(edge, EdgeKey)
#endif

void PolyMesh::vertexFunc(const Index3DId& id, const function<void(const Vertex& obj)>& func) const {
	func(_vertices[id]);
} 

void PolyMesh::vertexFunc(const Index3DId& id, const function<void(Vertex& obj)>& func) {
	func(_vertices[id]);
} 

const DFHM::Vertex& PolyMesh::getVertex(const Index3DId& id) const {
	return _vertices[id];
}  

DFHM::Vertex& PolyMesh::getVertex(const Index3DId& id) {
	return _vertices[id];
}

void PolyMesh::faceFunc(const Index3DId& id, const function<void(const Polygon& obj)>& func) const {
	func(_polygons[id]);
} 

void PolyMesh::faceFunc(const Index3DId& id, const function<void(Polygon& obj)>& func) {
	func(_polygons[id]);
} 

const DFHM::Polygon& PolyMesh::getPolygon(const Index3DId& id) const {
	return _polygons[id];
}  

DFHM::Polygon& PolyMesh::getPolygon(const Index3DId& id) {
	return _polygons[id];
}

void PolyMesh::edgeFunc(const EdgeKey& key, const function<void(const Edge& obj)>& func) const {
	if (_vertices.exists(key[0]) && _vertices.exists(key[1])) {
		Edge edge(key, this); 
		func(edge);
	}
} 

void PolyMesh::edgeFunc(const EdgeKey& key, const function<void(Edge& obj)>& func) {
	if (_vertices.exists(key[0]) && _vertices.exists(key[1])) {
		Edge edge(key, this); 
		func(edge);
	}
}

void PolyMesh::cellFunc(const Index3DId& key, const std::function<void(const Polyhedron& obj)>& func) const
{
	throw runtime_error("Not implemented");
}

void PolyMesh::cellFunc(const Index3DId& key, const std::function<void(Polyhedron& obj)>& func)
{
	throw runtime_error("Not implemented");
}

const Polyhedron& PolyMesh::getPolyhedron(const Index3DId& id) const
{
	throw runtime_error("Not implemented");
}

Polyhedron& PolyMesh::getPolyhedron(const Index3DId& id)
{
	throw runtime_error("Not implemented");
}
