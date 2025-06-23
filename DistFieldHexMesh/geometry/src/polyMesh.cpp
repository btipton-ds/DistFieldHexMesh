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

#include <defines.h>

#include <vector>

#include <tm_ioUtil.h>
#include <tm_spatialSearch.hpp>
#include <tm_math.h>
#include <tm_bestFit.h>

#include <tolerances.h>
#include <index3D.h>
#include <splitParams.h>
#include <appData.h>
#include <vertex.h>
#include <edge.h>
#include <polygon.h>
#include <polyMesh.h>
#include <splitter2D.h>
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

void PolyMesh::initClosed()
{
	if (_isClosed == IS_UNKNOWN) {
		_isClosed = IS_TRUE;

		map<EdgeKey, int> edgeMap;
		_polygons.iterateInOrder([&edgeMap](const Index3DId& faceId, const Polygon& face) {
			face.iterateEdgeKeys([&edgeMap](const EdgeKey& ek)->bool {
				auto iter = edgeMap.find(ek);
				if (iter == edgeMap.end()) {
					iter = edgeMap.insert(make_pair(ek, 0)).first;
				}
				iter->second++;
				return true;
				});
			});

		size_t numOpen = 0;
		for (const auto& pair : edgeMap) {
			if (pair.second != 2) {
				numOpen++;
			}
		}

		_isClosed = numOpen < 2 ? IS_TRUE : IS_FALSE;
	}
}

const CBoundingBox3Dd& PolyMesh::getBBox() const
{
	if (_bbox.empty()) {
		_polygons.iterateInOrder([this](const Index3DId& faceId, const Polygon& face) {
			const auto& vertIds = face.getVertexIds();
			for (const auto& id : vertIds) {
				const auto& pt = face.getVertexPoint(id);
				_bbox.merge(pt);
			}
		});
	}

	return _bbox;
}


bool PolyMesh::isClosed() const
{
	return _isClosed == IS_TRUE;
}

const Vector3d& PolyMesh::getVertexPoint(const Index3DId& id) const
{
	return _vertices[id];
}

void PolyMesh::simplify(const SplittingParams& params, bool flattenQuads)
{
	flattenSharps(params);
	makeQuads(params, flattenQuads);

	double maxSliverAngleRadians = 15 / 180.0 * M_PI;
	reduceSlivers(params, maxSliverAngleRadians);

	flattenFaces(params);
}

void PolyMesh::flattenSharps(const SplittingParams& params)
{
	vector<shared_ptr<vector<Index3DId>>> sharpLoops;
	createSharpEdgeLoops(params, sharpLoops);

	// Turn off vertex searching
	_vertices.setSupportsReverseLookup(false);

	size_t count = 0;
	for (const auto& ptr : sharpLoops) {
		const auto& loop = *ptr;
		if (loop.size() > 5) {
#if 0
			stringstream ss;
			ss << "D:/DarkSky/Projects/output/objs/sharpLoop_" << count++ << ".obj";
			dumpVertsAsLineSegs(ss.str(), loop);
#endif
			flattenEdgeLoop(loop);
		}
	}

	// Turn vertex searching back on. This also recreates the search map.
	_vertices.setSupportsReverseLookup(true);
}

void PolyMesh::flattenEdgeLoop(const std::vector<Index3DId>& loop)
{
	const auto tol = Tolerance::sameDistTolFloat();
	const auto tol2 = 2 * tol;

	vector<Vector3d> pts;
	for (const auto& id : loop)
		pts.push_back(getVertexPoint(id));

	Planed plane;
	double err;
	if (bestFitPlane(pts, plane, err) && err < tol) {
		for (const auto& id : loop) {
			const auto& pt = getVertexPoint(id);
			double dist = plane.distanceToPoint(pt);
			if (dist < tol2) {
				auto projPt = plane.projectPoint(pt);
				auto& vert = getVertex(id);
				vert.replacePoint(projPt);
			} else {
				int dbgBreak = 1;
			}
		}
	}
}
void PolyMesh::flattenFaces(const SplittingParams& params)
{
	_polygons.setSupportsReverseLookup(false);

	for (int i = 0; i < 5; i++) {
		map<Index3DId, Planed> faceIdToPlaneMap;
		auto createFaceToPlaneMap = [&]() {
			faceIdToPlaneMap.clear();
			_polygons.iterateInOrder([this, &faceIdToPlaneMap](const Index3DId& id, const Polygon& face)->bool {
				const auto& vertIds = face.getVertexIds();
				vector<Vector3d> pts;
				for (const auto& id : vertIds) {
					pts.push_back(getVertexPoint(id));
				}

				Planed plane;
				double err;
				bool fitPass = bestFitPlane(pts, plane, err);
				assert(fitPass);
				faceIdToPlaneMap.insert(make_pair(face.getId(), plane));
				return true;
				});
			};
		createFaceToPlaneMap();

		_vertices.iterateInOrder([&faceIdToPlaneMap, &createFaceToPlaneMap](const Index3DId& id, Vertex& vert)->bool {
			const double diff = 1.0e-9;
			const auto& faceIdSet = vert.getFaceIds();
			vector<Index3DId> faceIds(faceIdSet.begin(), faceIdSet.end());

			vector<Planed> planes;
			vector<double> grad, nullGrad, delta;

			planes.resize(faceIds.size());
			grad.resize(faceIds.size());
			delta.resize(faceIds.size(), 0);

			for (size_t i = 0; i < faceIds.size(); i++) {
				const auto& pl = faceIdToPlaneMap[faceIds[i]];
				planes[i] = pl;
			}

			auto calErr = [&](Vector3d pt, size_t idx, double sign)->double {
				double rmsErr = 0;
				for (size_t i = 0; i < faceIds.size(); i++) {
					const auto& pl = planes[i];

					if (idx == i) {
						pt += diff * pl.getNormal();
					}
					else if (fabs(sign) != 0) {
						pt += sign * diff * grad[i] * pl.getNormal();
					}
					auto err = pl.distanceToPoint(pt);
					rmsErr += err * err;
				}
				rmsErr /= faceIds.size();
				rmsErr = sqrt(rmsErr);

				return rmsErr;
				};

			Vector3d pt = vert.getPoint();

			double rmsErr1 = calErr(pt, -1, 0);
			if (rmsErr1 < Tolerance::sameDistTol()) {
				return true;
			}

			auto calGradient = [&]() {
				double gradSqr = 0;
				for (size_t i = 0; i < faceIds.size(); i++) {
					grad[i] = (calErr(pt, i, 0) - rmsErr1) / diff;
					gradSqr += grad[i] * grad[i];
				}
				gradSqr = sqrt(gradSqr);
				for (size_t i = 0; i < faceIds.size(); i++) {
					grad[i] /= gradSqr;
				}
				};
			calGradient();

			const double scale = 1;
			auto rmsErr0 = calErr(pt, -1, -1);
			auto rmsErr2 = calErr(pt, -1, 1);
			double b = (rmsErr2 - rmsErr0) / (2 * diff);
			double a = (rmsErr0 - rmsErr2) / (2 * diff * diff);
			double dx = scale * -b / (2 * a);

			Vector3d pt2 = pt;
			for (size_t i = 0; i < faceIds.size(); i++) {
				pt2 -= planes[i].getNormal() * dx * grad[i];
			}

			auto rmsErr3 = calErr(pt2, -1, 0);
			for (int i = 0; i < 3; i++) {
				if (b < 0.001)
					break;
				rmsErr1 = calErr(pt, -1, 0);
				pt = pt2;
				calGradient();

				rmsErr0 = calErr(pt, -1, -1);
				rmsErr2 = calErr(pt, -1, 1);
				b = (rmsErr2 - rmsErr0) / (2 * diff);
				a = (rmsErr0 - rmsErr2) / (2 * diff * diff);
				dx = scale * -b / (2 * a);

				pt2 = pt;
				for (size_t i = 0; i < faceIds.size(); i++) {
					pt2 -= planes[i].getNormal() * dx * grad[i];
				}

				rmsErr3 = calErr(pt2, -1, 0);
			}
			return true;
		});
	}
	_polygons.setSupportsReverseLookup(true);
}

void PolyMesh::flattenFaces_deprecated(const SplittingParams& params)
{
	/*
	* This needs to move vertices to be coplanar with the existing faces instead of moving independently.
	* That should converge in one pass - or NOT.
	*/
	size_t num = 0;
	double avgError = 0;
	for (int i = 0; i < params.numFlattenPasses; i++) {
		num = 0;
		_polygons.iterateInOrder([&num, &avgError](const Index3DId& id, Polygon& face)->bool {
			auto err = face.flatten(true);
			avgError += err;
			num++;
			return true;
		});
		avgError /= num;

		if (avgError < Tolerance::sameDistTol())
			break;
	}
}

void PolyMesh::makeQuads(const SplittingParams& params, bool flatten)
{
#if 1
	set<EdgeKey> edgeSet;
	_polygons.iterateInOrder([&edgeSet](const Index3DId& faceId, const Polygon& face)->bool {
		face.iterateEdges([&edgeSet](const Edge& edge) {
			edgeSet.insert(edge);
			return true;
		});
		return true;
	});

	// The set, vector, map allow the sort lambda to access precalulcated curvatures. It avoids expensive duplicate curvature calculations.
	vector<EdgeKey> edges;
	map<EdgeKey, double> edgeToCurvMap;
	for (const auto& ek : edgeSet) {
		double curv;
		edgeFunc(ek, [&curv, &params](const Edge& edge) {
			curv = edge.calCurvature(params);
		});
		edgeToCurvMap.insert(make_pair(ek, curv));
		edges.push_back(ek);
	}

	auto sortFunc = [this, &edgeToCurvMap](const EdgeKey& lhs, const EdgeKey& rhs) {
		auto iter = edgeToCurvMap.find(lhs);
		assert(iter != edgeToCurvMap.end());
		auto c0 = iter->second;

		iter = edgeToCurvMap.find(rhs);
		assert(iter != edgeToCurvMap.end());
		auto c1 = iter->second;
		return c0 < c1;
	};

	sort(edges.begin(), edges.end(), sortFunc);

	vector<Index3DId> newFaceIds;
	for (const auto& ek : edges) {
		edgeFunc(ek, [this, &params, &newFaceIds](const Edge& edge) {
			auto id = mergeToQuad(params, edge);
			if (id.isValid())
				newFaceIds.push_back(id);
		});
	}
#else
	set<EdgeKey> edgeSet;
	_polygons.iterateInOrder([&edgeSet](const Index3DId& faceId, const Polygon& face)->bool {
		face.iterateEdges([&edgeSet](const Edge& edge) {
			edgeSet.insert(edge);
			return true;
		});
		return true;
	});

	vector<EdgeKey> edges(edgeSet.begin(), edgeSet.end());

	auto sortFunc = [this, &params](const EdgeKey& lhs, const EdgeKey& rhs) {
		double c0, c1;
		edgeFunc(lhs, [&c0, &params](const Edge& edge) {
			c0 = edge.calCurvature(params);
			});
		edgeFunc(rhs, [&c1, &params](const Edge& edge) {
			c1 = edge.calCurvature(params);
			});

		return c0 < c1;
	};

	sort(edges.begin(), edges.end(), sortFunc);

	vector<Index3DId> newFaceIds;
	for (const auto& ek : edges) {
		edgeFunc(ek, [this, &params, &newFaceIds](const Edge& edge) {
			auto id = mergeToQuad(params, edge);
			if (id.isValid())
				newFaceIds.push_back(id);
		});
	}
#endif

	if (flatten) {
		_polygons.setSupportsReverseLookup(false);

		for (const auto& faceId : newFaceIds) {
			auto& face = getPolygon(faceId);
			face.flatten(true);
		}
		_polygons.setSupportsReverseLookup(true);
	}
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

		const auto faceIds = vert.getFaceIds();
		MTC::vector<MTC::vector<Index3DId>> planarFaceSets;
		makeCoplanarFaceSets(radiantId, faceIds, planarFaceSets);
		for (const auto& faceIds : planarFaceSets) {
			if (faceIds.size() > 2) {
				processPlanarFaces(params, radiantId, maxSliverAngleRadians, faceIds);
			}
		}
	}
}

void PolyMesh::processPlanarFaces(const SplittingParams& params, const Index3DId& radiantId, double minAngleRadians, const MTC::vector<Index3DId>& faceIds)
{
	const auto& radiantVert = getVertex(radiantId);

	set<Index3DId> faceIdSet(faceIds.begin(), faceIds.end()), sharedVerts;

	Planed basePlane;
	double maxAngle = 0;
	for (const auto& id : faceIds) {
		const auto& lhsFace = getPolygon(id);

		double angle = lhsFace.calVertexAngle(radiantId);
		if (angle > maxAngle) {
			basePlane = lhsFace.calPlane();
			maxAngle = angle;
		}
	}

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
		if (angle < 0)
			angle += 2 * M_PI;
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
		if (deltaAngle < 0)
			deltaAngle += 2 * M_PI;
		assert(deltaAngle >= 0 && deltaAngle < 2 * M_PI);

		if (deltaAngle > minAngleRadians || !sharedVerts.contains(thisId)) {
			lastAngle = thisAngle;
		} else {
			EdgeKey ek(radiantId, thisId);
			if (adjacentEdgesHaveSimilarLength(ek)) {
				auto newFaceId = removeEdgeWithChecks(params, basePlane, radiantId, thisId);
				if (!newFaceId.isValid()) {
					lastAngle = thisAngle;
				}
			}
		}
	}
}

bool PolyMesh::isRemovable(const SplittingParams& params, const EdgeKey& key) const
{
	bool result = false;

	edgeFunc(key, [this, &params, &result](const Edge& edge) {
		const auto& faceIds = edge.getFaceIds();
		auto iter = faceIds.begin();
		const auto& id0 = *iter++;
		const auto& id1 = *iter;

		const auto& face0 = getPolygon(id0);
		const auto& face1 = getPolygon(id1);

		const auto& verts0 = face0.getVertexIds();
		const auto& verts1 = face1.getVertexIds();

		if (verts0.size() + verts1.size() < _maxRemovableVerts) {
			double area0, area1;
			Vector3d discarded;
			face0.calAreaAndCentroid(area0, discarded);
			face1.calAreaAndCentroid(area1, discarded);
			double ratio = fabs((area0 / (area0 + area1)) - 0.5);
			if (ratio < _maxRemovableAreaRatio) {
				double l = edge.calLength();
				double w = (area0 + area1) / l;
				double aspectRatio = l / w;
				if (aspectRatio > _minAspectRatio) {
					result = true;
				}
			}
		}
	});

	return result;
}

Index3DId PolyMesh::removeEdgeWithChecks(const SplittingParams& params, const Planed& plane, const Index3DId& radiantVertId, const Index3DId& otherVertId)
{
	const double MAX_CP = 0.02;
	const double MAX_CP_SQR = MAX_CP * MAX_CP;
	const size_t maxVerts = 30;

	Index3DId result;
	EdgeKey key(radiantVertId, otherVertId);
	edgeFunc(key, [this, &plane, &params, &radiantVertId, &result](const Edge& edge) {
		const auto& edgeFaceIds = edge.getFaceIds();
		if (edgeFaceIds.size() != 2) {
			return;
		}

		auto iter = edgeFaceIds.begin();

		const auto& faceId0 = *iter++;
		const auto& faceId1 = *iter;

		auto& face0 = getPolygon(faceId0);
		auto& face1 = getPolygon(faceId1);

		const auto& otherId = edge.getOtherVert(radiantVertId);
		if (!otherId.isValid()) {
			return;
		}

		auto norm0 = face0.calUnitNormal();
		auto norm1 = face1.calUnitNormal();
		if (norm0.dot(norm1) < 0) {
			return;
		}

		MTC::vector<Index3DId> newVertIds;
		if (!mergeVertices(EdgeKey(radiantVertId, otherId), face0, face1, newVertIds)) {
			return;
		}

		// It's 50/50 that the new vertices are reversed. Test the new normal with the prior normal(s) and reverse if needed
		Vector3d n2;
		auto ncolin = PolygonSearchKey::makeNonColinearVertexIds(this, newVertIds);
		if (!Polygon::calUnitNormalStat(this, ncolin, n2)) {
//			dumpVertsAsPolygon("D:/DarkSky/Projects/output/objs/badPolyNormal.obj", newVertIds);
			return;
		}

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

		if (hasHighLocalConvexity(params, norm0, newVertIds)) {
			return;
		}

		// Run through all possible rotations

		if (formsValidPolygon(params, newVertIds, norm0)) {
			result = removeEdge(edge, faceId0, faceId1, newVertIds, plane);
		}
	});

	return result;
}

bool PolyMesh::mergeVertices(const EdgeKey& key, const Polygon& face0, const Polygon& face1, MTC::vector<Index3DId>& newVertIds) const
{
	const auto& radiantVertId = key[0];
	const auto& otherId = key[1];

	const auto& vertIds0 = face0.getVertexIds();
	const auto& vertIds1 = face1.getVertexIds();

	map<Index3DId, set<Index3DId>> vertToVertMap;
	for (size_t i = 0; i < vertIds0.size(); i++) {
		size_t j = (i + 1) % vertIds0.size();
		const auto& id0 = vertIds0[i];
		const auto& id1 = vertIds0[j];

		if (key != EdgeKey(id0, id1)) {
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

		if (key != EdgeKey(id0, id1)) {
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
		} else {
			auto curBack = newVertIds.back();
			auto iter = vertToVertMap.find(curBack);
			if (iter->second.size() != 1)
				return false; // This can happen when removing edges from a full circle. That's not allowed

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
	assert(newVertIds[0] == radiantVertId);

	return !newVertIds.empty();
}

void PolyMesh::createSharpEdgeLoops(const SplittingParams& params, std::vector<std::shared_ptr<std::vector<Index3DId>>>& sharpLoops) const
{
	set<EdgeKey> sharpEdges;
	_polygons.iterateInOrder([this, &params, &sharpEdges](const Index3DId& id, const Polygon& face)->bool {
		face.iterateEdges([this, &params, &sharpEdges](const Edge& edge)->bool {
			auto& faceEdges = edge.getFaceIds();
			if (faceEdges.size() == 2) {
				auto angle = calEdgeAngle(edge);
				if (angle > params.getSharpAngleRadians())
					sharpEdges.insert(edge);
			}
			return true;
		});
		return true;
	});

	map<Index3DId, set<Index3DId>> vertToVertMap;
	for (const auto& ek : sharpEdges) {
		for (int i = 0; i < 2; i++) {
			auto iter = vertToVertMap.find(ek[i]);
			if (iter == vertToVertMap.end())
				iter = vertToVertMap.insert(make_pair(ek[i], set<Index3DId>())).first;
			iter->second.insert(ek[1 - i]);
		}
	}

	shared_ptr<vector<Index3DId>> pLoop = make_shared<vector<Index3DId>>();
	while (!vertToVertMap.empty()) {
		auto& loop = *pLoop;
		if (loop.empty()) {
			for (const auto& pair : vertToVertMap) {
				if (pair.second.size() > 2) {
					loop.push_back(pair.first);
					loop.push_back(*pair.second.begin());

					break;
				}
			}

			if (loop.empty()) {
				for (const auto& pair : vertToVertMap) {
					if (pair.second.size() == 1) {
						loop.push_back(pair.first);
						loop.push_back(*pair.second.begin());

						break;
					}
				}
			}

			if (loop.empty()) {
				auto iter = vertToVertMap.begin();
				loop.push_back(iter->first);
				const auto& conSet = iter->second;
				if (conSet.empty()) {
					assert(!"Not implemented yet");
				} else {
					auto iter2 = conSet.begin();
					loop.push_back(*iter2);

				}
			}

			vertToVertMap.erase(loop.front());
			auto iter = vertToVertMap.find(loop.back());
			if (iter != vertToVertMap.end()) {
				iter->second.erase(loop.front());
				if (iter->second.empty()) {
					vertToVertMap.erase(loop.back());
				}
			}
		}

		auto tailId = loop.back();
		auto tailIter = vertToVertMap.find(tailId);
		if (tailIter == vertToVertMap.end()) {
			sharpLoops.push_back(pLoop);
			pLoop = make_shared<vector<Index3DId>>();
		} else {
			auto& conSet = tailIter->second;
			Index3DId smoothestVert = chooseSmoothestVert(params, conSet, loop);

			if (!smoothestVert.isValid()) {
				sharpLoops.push_back(pLoop);
				pLoop = make_shared<vector<Index3DId>>();
			} else {
				loop.push_back(smoothestVert);
				conSet.erase(smoothestVert);
				if (conSet.empty())
					vertToVertMap.erase(tailId);

			}

			tailIter = vertToVertMap.find(loop.back());
			if (tailIter != vertToVertMap.end()) {
				tailIter->second.erase(tailId);
				if (tailIter->second.empty())
					vertToVertMap.erase(loop.back());
			}
		}
	}
}

Index3DId PolyMesh::chooseSmoothestVert(const SplittingParams& params, const std::set<Index3DId>& conVerts, const std::vector<Index3DId>& loop) const
{
	auto maxAngle = params.getSharpAngleRadians();

	double angle = 0;
	const auto& pt0 = getVertexPoint(loop[loop.size() - 2]);
	const auto& origin = getVertexPoint(loop[loop.size() - 1]);
	Vector3d xAxis = (origin - pt0).normalized();
	for (const auto& id : conVerts) {
		const auto& pt1 = getVertexPoint(id);
		Vector3d v = (pt1 - origin).normalized();
		Vector3d zAxis = v.cross(xAxis);
		if (zAxis.squaredNorm() < 1.0e-12)
			continue;
		zAxis.normalize();
		Vector3d yAxis = zAxis.cross(xAxis);
		double x = v.dot(xAxis);
		double y = v.dot(yAxis);
		angle = atan2(y, x);
		if (fabs(angle) < maxAngle) {
			return id;
		}
	}

	return Index3DId();
}

Index3DId PolyMesh::removeEdge(const EdgeKey& key, const Index3DId& faceId0, const Index3DId& faceId1, const MTC::vector<Index3DId>& newVertIds, const Planed& plane)
{
	Index3DId result;
	removeFace(faceId0);
	removeFace(faceId1);
	Polygon newFace(newVertIds);
	result = _polygons.findOrAdd(newFace);
	if (result == Index3DId(0, 0, 0, 113)) {
		int dbgBreak = 1;
	}
	auto& face = getPolygon(result);

	// For this new face to have the plane of the source faces
	face.setCentroid_risky(plane.getOrigin());
	face.setUnitNormal_risky(plane.getNormal());
	face.setIsConvex_risky(Polygon::IS_CONVEX_ENOUGH);

	return result;
}

bool PolyMesh::formsValidPolygon(const SplittingParams& params, const MTC::vector<Index3DId>& vertIds, const Vector3d& norm) const
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

		auto v0 = (pt0 - pt1).normalized();
		auto v1 = (pt2 - pt1).normalized();
		n = v1.cross(v0);
		auto dp = norm.dot(n);
		if (dp < params.maxLocalConcavityCrossProduct)
			return false;
	}

	if (!Polygon::calUnitNormalStat(this, vertIds, n))
		return false;
	return true;
}

bool PolyMesh::formsValidQuad(const SplittingParams& params, const MTC::vector<Index3DId>& vertIds) const
{
	const double maxAcuteAngle = 15 / 180.0 * M_PI;

	if (vertIds.size() != 4)
		return false;

	const vector<Vector3d> pts = {
		getVertexPoint(vertIds[0]),
		getVertexPoint(vertIds[1]),
		getVertexPoint(vertIds[2]),
		getVertexPoint(vertIds[3]),
	};

	Vector3d v0, v1, n0, n1;

	// Check normals are aligned
	v0 = pts[0] - pts[1];
	v1 = pts[2] - pts[1];

	n0 = v1.cross(v0);

	v0 = pts[2] - pts[3];
	v1 = pts[0] - pts[3];

	n1 = v1.cross(v0);
	auto dp = n0.dot(n1);
	if (dp < 0)
		return false;

	Planed plane;
	double err;
	if (bestFitPlane(pts, plane, err) && err < Tolerance::sameDistTolFloat()) {
		for (size_t i = 0; i < vertIds.size(); i++) {
			if (Polygon::calVertexAngleStat(this, vertIds, i) < maxAcuteAngle)
				return false;
		}
		return true;
	}

	return false;
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

bool PolyMesh::isCoplanar(const SplittingParams& params, const Edge& edge) const
{
	auto cur = edge.calCurvature(params); // calCurvature return 0 if there are not two faces
	return cur >= 0 && cur < Tolerance::paramTol();
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

bool PolyMesh::adjacentEdgesHaveSimilarLength(const EdgeKey& edgeKey) const
{
	const double MAX_LENGTH_VARIATION = 0.2;
	double ratio;
	edgeFunc(edgeKey, [this, &ratio](const Edge& edge) {
		const auto& radiantId = edge[0];

		auto& faceIds = edge.getFaceIds();
		auto iter = faceIds.begin();
		const auto& faceId0 = *iter++;
		const auto& faceId1 = *iter;

		const auto& face0 = getPolygon(faceId0);
		const auto& face1 = getPolygon(faceId1);

		double edgeLen = edge.calLength(), len0, len1;

		face0.iterateEdges([&len0, &edge, &radiantId](const Edge& testEdge)->bool {
			if (testEdge != edge && testEdge.containsVertex(radiantId)) {
				len0 = testEdge.calLength();
				return false;
			}
			return true;
			});

		face1.iterateEdges([&len1, &edge, &radiantId](const Edge& testEdge)->bool {
			if (testEdge != edge && testEdge.containsVertex(radiantId)) {
				len1 = testEdge.calLength();
				return false;
			}
			return true;
			});

		if (len0 > len1)
			ratio = len1 / len0;
		else
			ratio = len0 / len1;

		assert(ratio <= 1); // Should be guaranteed due to the preceding if test
	});

	return ratio > 1 - MAX_LENGTH_VARIATION;
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

double PolyMesh::calEdgeAngle(const Edge& edge) const
{
	const auto& origin = getVertexPoint(edge[0]);
	auto zAxis = edge.calUnitDir();

	const auto& faceIds = edge.getFaceIds();
	assert(faceIds.size() == 2);

	auto iter = faceIds.begin();
	const auto& face0 = getPolygon(*iter++);
	const auto& face1 = getPolygon(*iter);

	auto ctr0 = face0.calCentroid();
	auto ctr1 = face1.calCentroid();

	Vector3d xAxis = origin - ctr0;
	xAxis = xAxis - zAxis * zAxis.dot(xAxis);
	xAxis.normalize();
	Vector3d yAxis = zAxis.cross(xAxis);

	Vector3d v = ctr1 - origin;
	v = v - zAxis * zAxis.dot(xAxis);
	v.normalize();

	double x = v.dot(zAxis);
	double y = v.dot(yAxis);

	double angle = atan2(y, x);
	return angle;
}

void PolyMesh::makeCoplanarFaceSets(const Index3DId& radiantId, const FastBisectionSet<Index3DId>& faceIds, MTC::vector<MTC::vector<Index3DId>>& planarFaceSets)
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
	vector<Index3DId> faceIdsAscendingAngle(faceIds.begin(), faceIds.end());
	sort(faceIdsAscendingAngle.begin(), faceIdsAscendingAngle.end(), [this, &radiantId](const Index3DId& lhsId, const Index3DId& rhsId) {
		const auto& lhsFace = getPolygon(lhsId);
		const auto& rhsFace = getPolygon(rhsId);

		double lhsAngle = lhsFace.calVertexAngle(radiantId);
		double rhsAngle = rhsFace.calVertexAngle(radiantId);
		return lhsAngle < rhsAngle;
	});

	MTC::vector<Index3DId> currentFaces;
	Vector3d weightedNormal, groupOrigin;
	double totalArea = 0;
	double currentArea;
	Vector3d discarded;
	while (!faceIdsAscendingAngle.empty()) {
		if (currentFaces.empty()) {
			currentFaces.push_back(faceIdsAscendingAngle.back());
			faceIdsAscendingAngle.pop_back();
			const auto& face = getPolygon(currentFaces.back());
			face.calAreaAndCentroid(currentArea, groupOrigin);
			weightedNormal = currentArea * face.calUnitNormal();
			totalArea += currentArea;
		}

		double maxArea = 0;
		size_t bestFaceIdx = -1;
		Vector3d bestNormal;
		Planed groupPlane(groupOrigin, weightedNormal);
		for (size_t i = 0; i < faceIdsAscendingAngle.size(); i++) {
			const auto& currentFace = getPolygon(faceIdsAscendingAngle[i]);
			auto currentFaceNorm = currentFace.calUnitNormal();
			if (weightedNormal.dot(currentFaceNorm) > 0) {
				const auto& vertIds = currentFace.getVertexIds();
				bool allVertsCoplanar = true;
				for (const auto& vertId : vertIds) {
					const auto& pt = getVertexPoint(vertId);
					auto dist = groupPlane.distanceToPoint(pt);
					if (dist > sameDistTol) {
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
			currentFaces.push_back(faceIdsAscendingAngle[bestFaceIdx]);
			faceIdsAscendingAngle.erase(faceIdsAscendingAngle.begin() + bestFaceIdx);
			assert(bestNormal.dot(weightedNormal) > 0);
			weightedNormal += maxArea * bestNormal;
			totalArea += maxArea;
		} else {
			if (currentFaces.size() > 1)
				planarFaceSets.push_back(currentFaces);
			currentFaces.clear();
		}
	}

	if (currentFaces.size() > 1)
		planarFaceSets.push_back(currentFaces);

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

Index3DId PolyMesh::mergeToQuad(const SplittingParams& params, const Edge& edge)
{
	Index3DId result;

	auto& faceIds = edge.getFaceIds();
	if (faceIds.size() != 2)
		return result;

	auto iter = faceIds.begin();
	auto faceId0 = *iter++;
	auto faceId1 = *iter;

	if (!_polygons.exists(faceId0) || !_polygons.exists(faceId1))
		return result;

	auto& face0 = getPolygon(faceId0);
	auto& face1 = getPolygon(faceId1);
	if (isShortEdge(edge, face0, face1))
		return result;

	auto vertIds0 = face0.getVertexIds();
	auto vertIds1 = face1.getVertexIds();

	MTC::vector<Index3DId> newVertIds;
	if (!mergeVertices(edge, face0, face1, newVertIds))
		return result;
	
	if (!formsValidQuad(params, newVertIds))
		return result;

	Vector3d norm = face0.calUnitNormal() + face1.calUnitNormal();
	norm.normalize();
	const auto& ctr = getVertexPoint(vertIds0[0]);
	Planed plane(ctr, norm);
	result = removeEdge(edge, faceId0, faceId1, newVertIds, plane);

	return result;
}

void PolyMesh::removeFace(const Index3DId& id)
{
	_polygons[id].disconnectVertEdgeTopology();
	_polygons.removeFromLookup(id);
	_polygons.free(id);
}

void PolyMesh::getGlTriPoints(std::vector<float>& result) const
{
	result.clear();
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
}

void PolyMesh::getGlTriNormals(std::vector<float>& result) const
{
	result.clear();
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
}

void PolyMesh::getGlTriIndices(std::vector<unsigned int>& result) const
{
	result.clear();
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
}

void PolyMesh::dumpVertsAsPolygon(const std::string& path, const MTC::vector<Index3DId>& vertIds) const
{
	ofstream out(path);
	for (const auto& id : vertIds) {
		const auto& pt = getVertexPoint(id);
		out << "v " << pt[0] << " " << pt[1] << " " << pt[2] << "\n";
	}

	out << "f";
	for (size_t i = 0; i < vertIds.size(); i++) {
		size_t idx = i + 1;
		out << " " << idx;
	}
	out << "\n";
}

void PolyMesh::dumpVertsAsLineSegs(const std::string& path, const MTC::vector<Index3DId>& vertIds) const
{
	ofstream out(path);
	for (const auto& id : vertIds) {
		const auto& pt = getVertexPoint(id);
		out << "v " << pt[0] << " " << pt[1] << " " << pt[2] << "\n";
	}

	for (size_t i = 0; i < vertIds.size() - 1; i++) {
		size_t j = (i + 1) % vertIds.size();
		out << "l " << (i + 1) << " " << (j + 1) << "\n";
	}
}

void PolyMesh::dumpFaceSetAsObj(const std::string& path, size_t num, const Index3DId& radiantId, const MTC::vector<Index3DId>& faceIds, const std::set<Index3DId>& sharedVerts) const
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
		if (!_polygons.exists(faceId))
			continue;

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

		for (const auto& faceId : faceIds) {
			if (!_polygons.exists(faceId))
				continue;
			const auto& face = getPolygon(faceId);
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
			if (iter != ptToIndexMap.end()) {
				size_t idx1 = iter->second + 1;
				out << "l " << idx0 << " " << idx1 << "\n";
			}
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
