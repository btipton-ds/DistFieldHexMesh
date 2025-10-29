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
#include <assert.h>

#include <objectPool.hpp>
#include <meshData.h>
#include <model.h>
#include <tm_spatialSearch.hpp>
#include <tolerances.h>
#include <splitParams.h>
#include <polygon.h>
#include <polygonSampling.hpp>
#include <debugMeshData.h>

using namespace std;
using namespace DFHM;

void Model::clear()
{
	_modelMeshData.clear();
	if (_pPolyMeshSearchTree)
		_pPolyMeshSearchTree->clear();
}

size_t Model::numBytes() const
{
	size_t result = _modelMeshData.size();
	for (const auto& pData : _modelMeshData) {
		if (pData)
			result += pData->numBytes();
	}

	if (_pPolyMeshSearchTree)
		result += _pPolyMeshSearchTree->numBytes();
	return result;
}

void Model::setBounds(const BOX_TYPE& bbox)
{
	_pPolyMeshSearchTree = make_shared<PolyMeshSearchTree>(bbox);
}

size_t Model::add(const MeshDataPtr& pData)
{
	CBoundingBox3Dd bbox;
	for (const auto& pData : _modelMeshData) {
		bbox.merge(pData->getBBox());
	}
	bbox.merge(pData->getBBox());
	setBounds(bbox);

	size_t meshIdx = _modelMeshData.size();
	_modelMeshData.push_back(pData);

	return _modelMeshData.size();
}

void Model::rebuildSearchTree()
{
	CBoundingBox3Dd bbox;
	for (size_t meshIdx = 0; meshIdx < _modelMeshData.size(); meshIdx++) {
		auto& pData = _modelMeshData[meshIdx];
		bbox.merge(pData->getBBox());
	}

	bbox.grow(Polygon::bboxOffsetDist());
	_pPolyMeshSearchTree = make_shared<PolyMeshSearchTree>(bbox);

	for (size_t meshIdx = 0; meshIdx < _modelMeshData.size(); meshIdx++) {
		auto& pData = _modelMeshData[meshIdx];
		auto& pPolyMesh = pData->getPolyMesh();

		if (pPolyMesh) {
			pPolyMesh->iteratePolygons([this, meshIdx](const Index3DId& id, const Polygon& face) {
				auto bb = face.getBBox();
				_pPolyMeshSearchTree->add(bb, PolyMeshIndex(meshIdx, id));

				return true;
			});
		}
	}
}

bool Model::isPointInGap(const SplittingParams& params, const Vector3d& startPt, const Vector3d& dir) const
{
	if (!_pPolyMeshSearchTree)
		return false;

	const double minDotProduct = sin(params.gapOpposingFaceAngleDeg * M_PI / 180.0);

#if ENABLE_DEBUGGING_MUTEXES
	static std::mutex mut;
	std::lock_guard<std::mutex> lg(mut);
#endif
	CBoundingBox3Dd bbox(startPt);
	bbox.grow(1.5 * params.maxGapWidth);
	vector<PolyMeshIndex> nearFaceIds;
	if (_pPolyMeshSearchTree->find(bbox, nullptr, nearFaceIds) != 0) {
		for (size_t i = 0; i < nearFaceIds.size(); i++) {
			const auto& nearFaceId = nearFaceIds[i];
			auto pMesh = _modelMeshData[nearFaceId.getMeshIdx()];

			bool nearModelIsClosed;
			const auto& pNearFace = getPolygon(nearFaceId, nearModelIsClosed);
			if (!pNearFace) {
				continue;
			}

			const auto& nearFaceNorm = pNearFace->calUnitNormal();

			Vector3d hitPt;
			double minDistToPoint = pNearFace->minDistToPoint(startPt, hitPt);
			if (minDistToPoint > 0 && minDistToPoint < params.maxGapWidth) {
				if (!nearModelIsClosed)
					return true;
				Vector3d v = hitPt - startPt;
				v.normalize();
				double dpHitFace = v.dot(nearFaceNorm);
				if (dpHitFace < -minDotProduct) {
					return true;
				}
			}
		}

	}

	return false;
}

struct Model::FitGapCircle
{
	FitGapCircle(const Model& model, const DFHM::Polygon* pFace0, bool startIsClosed, const Vector3d& startPt, const DFHM::Polygon* pFace1, bool endIsClosed)
		: _model(model)
		, _startModelIsClosed(startIsClosed)
		, _endModelIsClosed(endIsClosed)
	{
		_pts[0] = startPt;
		_contactFaces[0] = pFace0;
		_contactFaces[1] = pFace1;
		_normals[0] = pFace0->calUnitNormal();
		_normals[1] = pFace1->calUnitNormal();
	}

	bool calClosestPoint(Vector3d& result) {
		const auto distTol = Tolerance::sameDistTolFloat();
		const auto cpTol = Tolerance::planeCoincidentCrossProductTol();

		bool found = false;
		const auto pFace0 = _contactFaces[0];
		const auto pFace1 = _contactFaces[1];

		const auto& plane0 = pFace0->calPlane();
		const auto& plane1 = pFace1->calPlane();
		if (plane0.isCoincident(plane1, distTol, cpTol))
			return false;

		double minDist = DBL_MAX;

		Rayd intersectionRay;
		if (plane0.intersectPlane(plane1, intersectionRay, Tolerance::sameDistTol())) {
			Vector3d sectionOrigin = intersectionRay.project(_pts[0]);
			Planed sectionPlane(sectionOrigin, intersectionRay._dir);
			Vector3d projPt0 = sectionPlane.projectPoint(_pts[0], Tolerance::sameDistTol());
			Vector3d projPt1 = sectionPlane.projectPoint(plane1.getOrigin(), Tolerance::sameDistTol());
			_xAxis = (projPt0 - sectionOrigin).normalized();
			_zAxis = intersectionRay._dir;
			_yAxis = _zAxis.cross(_xAxis).normalized();

			auto v = projPt1 - sectionOrigin;
			auto x = _xAxis.dot(v);
			auto y = _yAxis.dot(v);
			auto theta = atan2(y, x) / 2;
			Vector3d pt1 = sectionOrigin + cos(theta) * _xAxis + sin(theta) * _yAxis;
			Vector3d v0 = pt1 - sectionOrigin;
			Vector3d v1 = _zAxis;
			Vector3d midNorm = v0.cross(v1);
			Planed midPlane(sectionOrigin, midNorm);

			Rayd normRay(_pts[0], _normals[0]);
			RayHitd hit;
			if (midPlane.intersectRay(normRay, hit, Tolerance::sameDistTol())) {
				_pts[2] = hit.hitPt;
				_pts[1] = plane1.projectPoint(_pts[2], Tolerance::sameDistTol());
				Vector3d v = _pts[1] - _pts[0];
				if (_startModelIsClosed && plane0.getNormal().dot(v) > 0 &&
					(!_endModelIsClosed || plane1.getNormal().dot(v) < 0)) {
					v0 = (_pts[0] - _pts[2]).normalized();
					v1 = (_pts[1] - _pts[2]).normalized();
					auto dpCtr = v0.dot(v1);
					if (dpCtr < 0) {
						if (pFace1->isPointInside(_pts[1], plane1)) {
#if 0
							auto dist0 = (_pts[0] - _pts[2]).norm();
							auto dist1 = (_pts[1] - _pts[2]).norm();
							auto err = dist1 - dist0;
							assert(fabs(err) < 1.0e-5);
#endif
							result = _pts[1];
							found = true;
						}
					}
				}
			} // If the ray doesn't hit the midplane, it's a pathological case and we should be able to ignore it.
		} else {
			// Parallel planes case
			_pts[1] = plane1.projectPoint(_pts[0], Tolerance::sameDistTol());
			if (pFace1->isPointInside(_pts[1], plane1)) {
				Vector3d v = _pts[1] - _pts[0];
				if (_startModelIsClosed && plane0.getNormal().dot(v) > 0 &&
					(!_endModelIsClosed || plane1.getNormal().dot(v) < 0)) {
					_pts[2] = _pts[0] + 0.5 * v;
					result = _pts[1];
					found = true;
				}
			}
		}

		return found;
	}

	const Model& _model;
	const bool _startModelIsClosed, _endModelIsClosed;
	Vector3d _pts[3]; // 0 and 1 are the contact points. Point 2 is the center point when the distance between 0 and 1 are equal
	Vector3d _normals[2];
	Vector3d _xAxis, _yAxis, _zAxis;
	const DFHM::Polygon* _contactFaces[2]; // These are the polygons the points fall on
};

void Model::calculateGaps(const SplittingParams& params)
{
	if (!_pPolyMeshSearchTree)
		return;

	size_t numIndices = 0;
	for (size_t i = 0; i < _modelMeshData.size(); i++) {
		numIndices += _modelMeshData[i]->getPolyMesh()->numPolygons();
	}

	vector<PolyMeshIndex> allFaceIndices;
	allFaceIndices.reserve(numIndices);

	// Connected face determination is expensive, so make a lookup map. May want to do a binary search vector at some time.
	map<PolyMeshIndex, set<PolyMeshIndex>> connectedFaceMap;

	for (size_t meshIdx = 0; meshIdx < _modelMeshData.size(); meshIdx++) {
		auto& pMeshData = _modelMeshData[meshIdx];
		auto& pStartPolyMesh = pMeshData->getPolyMesh();

		pStartPolyMesh->iteratePolygons([this, &allFaceIndices, &connectedFaceMap, meshIdx](const Index3DId& id, Polygon& face)->bool {
			PolyMeshIndex polyFaceId(meshIdx, id);
			allFaceIndices.push_back(polyFaceId);
			set<Index3DId> ids;
			face.getConnectedFaceIds(ids);
			auto iter = connectedFaceMap.insert(make_pair(polyFaceId, set<PolyMeshIndex>())).first;
			auto& polyIds = iter->second;
			for (const auto& id : ids)
				polyIds.insert(PolyMeshIndex(meshIdx, id));
			return true;
		});
	}

	sort(allFaceIndices.begin(), allFaceIndices.end(), [this](const PolyMeshIndex& lhs, const PolyMeshIndex& rhs)->bool {
		double lhsArea, rhsArea;
		Vector3d lhsCtr, rhsCtr;

		getPolygon(lhs)->calAreaAndCentroid(lhsArea, lhsCtr);
		getPolygon(rhs)->calAreaAndCentroid(rhsArea, rhsCtr);
		return lhsArea < rhsArea;
	});

	MultiCore::runLambda([this, &params, &allFaceIndices, &connectedFaceMap](size_t index)->bool {

		const auto startFaceId = allFaceIndices[index];
		auto pStartModelData = _modelMeshData[startFaceId.getMeshIdx()];
		auto pStartMesh = pStartModelData->getPolyMesh();
		bool startModelIsClosed = pStartMesh->isClosed();
		auto pStartFace = getPolygon(startFaceId);
		if (!pStartFace)
			return true; // skip this one

		auto connectedFaceIter = connectedFaceMap.find(startFaceId);
		const auto& connectedFaceIds = connectedFaceIter->second;
		pStartFace->setNeedsGapTest(IS_FALSE);

		pStartFace->sampleSpacedPoints(params.gapGridSpacing, [this, &params, &connectedFaceIds, startModelIsClosed, pStartFace](const Vector3d& pt) {
			Vector3d endVec;
			PolyMeshIndex endFaceId;
			if (calculateFaceGaps(params, connectedFaceIds, pt, startModelIsClosed, pStartFace, endVec, endFaceId)) {
				if (endVec.squaredNorm() > Tolerance::sameDistTolSqr()) {
					pStartFace->addGap(pt, endVec, endFaceId);
				}
			}
		});


		return true;
	}, allFaceIndices.size(), RUN_MULTI_THREAD);

}

bool Model::calculateFaceGaps(const SplittingParams& params, const set<PolyMeshIndex>& connectedFaceIds, const Vector3d& startPt,
	bool startModelIsClosed, Polygon* pStartFace, Vector3d& endPtVector, PolyMeshIndex& endPtFaceId) const
{
	const double minDotProduct = sin(params.gapOpposingFaceAngleDeg * M_PI / 180.0);
	bool result = false;

	double startArea;
	Vector3d startCtr;
	pStartFace->calAreaAndCentroid(startArea, startCtr);

	const auto& pl = pStartFace->calPlane();
	const auto& startFaceNorm = pl.getNormal();
	Vector3d xAxis = pl.getXRef();
	Vector3d yAxis = startFaceNorm.cross(xAxis).normalized();
	CBoundingBox3Dd bbox;

	auto dist = params.maxGapWidth;
	bbox.merge(startPt - xAxis * dist - yAxis * dist);
	bbox.merge(startPt + xAxis * dist - yAxis * dist);
	bbox.merge(startPt + xAxis * dist + yAxis * dist);
	bbox.merge(startPt - xAxis * dist + yAxis * dist);

	bbox.merge(startPt - xAxis * dist - yAxis * dist + startFaceNorm * dist);
	bbox.merge(startPt + xAxis * dist - yAxis * dist + startFaceNorm * dist);
	bbox.merge(startPt + xAxis * dist + yAxis * dist + startFaceNorm * dist);
	bbox.merge(startPt - xAxis * dist + yAxis * dist + startFaceNorm * dist);

	std::vector<PolyMeshIndex> nearFaceIds;
	if (_pPolyMeshSearchTree->find(bbox, nullptr, nearFaceIds) != 0) {
#if 0 || ENABLE_DEBUGGING_MUTEXES
		static mutex mut;
		lock_guard<mutex> lg(mut);
#endif
		double minDist = DBL_MAX;
		for (size_t faceIdx = 0; faceIdx < nearFaceIds.size(); faceIdx++) {
			const auto& nearFaceId = nearFaceIds[faceIdx];
			const auto& pNearFace = getPolygon(nearFaceId);

			if (!pNearFace || connectedFaceIds.count(nearFaceId) != 0) {
				continue;
			}

			// Faces are sorted by ascending area. If the test face has smaller area than this face
			// We've already tested it in the other direction and can skip it.
			double nearArea;
			Vector3d nearCtr;
			pNearFace->calAreaAndCentroid(nearArea, nearCtr);
			if (nearArea < startArea)
				continue;


			const auto& nearFaceNorm = pNearFace->calUnitNormal();
			auto dp = startFaceNorm.dot(nearFaceNorm);
			if (fabs(dp) < 0.5)
				continue;

			const auto& nearVertIds = pNearFace->getVertexIds();
			bool hasVertexInFront = false;
			for (const auto& vertId : nearVertIds) {
				const auto& pt = pNearFace->getVertexPoint(vertId);
				Vector3d v = pt - startPt;
				if (v.dot(startFaceNorm) > 0) {
					hasVertexInFront = true;
					break;
				}
			}
			if (!hasVertexInFront)
				continue;

			bool nearFaceModelIsClosed = isClosed(nearFaceId);

			FitGapCircle prism(*this, pStartFace, startModelIsClosed, startPt, pNearFace, nearFaceModelIsClosed);
			Vector3d hitPt;
			if (prism.calClosestPoint(hitPt)) {
				//						writeGapObj(pStartFace, pNearFace, prism);
				Vector3d v = hitPt - startPt;
				double minDistToPoint = v.norm();
				if (minDistToPoint > 0 && minDistToPoint < params.maxGapWidth && minDistToPoint < minDist) {
					minDist = minDistToPoint;
					v.normalize();
					double dpHitFace = v.dot(nearFaceNorm);
					double dpStartFace = v.dot(startFaceNorm);
					auto pMesh = _modelMeshData[nearFaceId.getMeshIdx()];
					bool nearModelisClosed = pMesh->isClosed();
					//						if ((!nearModelisClosed || dpHitFace < -minDotProduct) && dpStartFace > minDotProduct) {
												// if (!obscured(startPt, hitPt))
					pStartFace->setNeedsGapTest(IS_TRUE);
					endPtVector = prism._pts[1] - prism._pts[0];
					endPtFaceId = nearFaceId;
					if (endPtVector.squaredNorm() > 0)
						result = true;
					//						}
				}
			}
		}
	}
	return result;
}

void Model::addGapDebugGraphicsData(DebugMeshDataPtr& pDbgData) const
{
	for (const auto& pMeshData : _modelMeshData) {
		auto pMesh = pMeshData->getPolyMesh();
		pMesh->iteratePolygons([pDbgData](const Index3DId& faceId, const Polygon& face)->bool {
			const auto& gapData = face.getGapData();
			for (const auto& rec : gapData) {
				LineSegmentd seg(rec._ourPoint, rec._ourPoint + rec._endVec);
				pDbgData->add(seg);
			}

			return true;
		});
	}
}

void Model::writeGapObj(const Polygon* pStartFace, const Polygon* pNearFace, const FitGapCircle& prism) const
{
	vector<Vector3d> pts;
	vector<int> indices;
	vector<vector<int>> faces, edges;
	map<const Vector3d, int> ptToIndexMap;

	vector<int> face;

	for (const auto& id : pStartFace->getVertexIds()) {
		const auto& pt = pStartFace->getVertexPoint(id);
		auto iter = ptToIndexMap.find(pt);

		if (iter == ptToIndexMap.end()) {
			int idx = pts.size();
			pts.push_back(pt);
			iter = ptToIndexMap.insert(make_pair(pt, idx)).first;
		}
		face.push_back(iter->second);
	}
	faces.push_back(face);
	face.clear();

	for (const auto& id : pNearFace->getVertexIds()) {
		const auto& pt = pNearFace->getVertexPoint(id);
		auto iter = ptToIndexMap.find(pt);

		if (iter == ptToIndexMap.end()) {
			int idx = pts.size();
			pts.push_back(pt);
			iter = ptToIndexMap.insert(make_pair(pt, idx)).first;
		}
		face.push_back(iter->second);
	}
	faces.push_back(face);

	auto iter = ptToIndexMap.find(prism._pts[0]);

	if (iter == ptToIndexMap.end()) {
		int idx = pts.size();
		pts.push_back(prism._pts[0]);
		iter = ptToIndexMap.insert(make_pair(prism._pts[0], idx)).first;
	}
	int startPtIdx = iter->second;

	iter = ptToIndexMap.find(prism._pts[1]);

	if (iter == ptToIndexMap.end()) {
		int idx = pts.size();
		pts.push_back(prism._pts[1]);
		iter = ptToIndexMap.insert(make_pair(prism._pts[1], idx)).first;
	}
	int endPtIdx = iter->second;

	iter = ptToIndexMap.find(prism._pts[2]);

	if (iter == ptToIndexMap.end()) {
		int idx = pts.size();
		pts.push_back(prism._pts[2]);
		iter = ptToIndexMap.insert(make_pair(prism._pts[2], idx)).first;
	}
	int ctrPtIdx = iter->second;

	{
		ofstream out("D:/Projects/output/objs/gapFaces.obj");
		for (const auto& pt : pts) {
			out << "v " << pt[0] << " " << pt[1] << " " << pt[2] << "\n";
		}
		for (const auto& face : faces) {
			out << "f";
			for (int idx : face)
				out << " " << (idx + 1);
			out << "\n";
		}
	}
	{
		ofstream out("D:/Projects/output/objs/gapEdges.obj");
		for (const auto& pt : pts) {
			out << "v " << pt[0] << " " << pt[1] << " " << pt[2] << "\n";
		}

		out << "f " << (startPtIdx + 1) << " " << (ctrPtIdx + 1) << " " << (endPtIdx + 1) << "\n";
	}
}

void Model::clampVerticesToSymPlanes(const std::vector<Planed>& symPlanes)
{
	for (auto& pData : _modelMeshData) {
		if (pData && pData->getPolyMesh()) {
			pData->getPolyMesh()->initSymmetry(symPlanes);
		}
	}
}

bool Model::isClosed(const PolyMeshIndex& id) const
{
	return _modelMeshData[id.getMeshIdx()]->isClosed();
}

bool Model::isPointInside(const Vector3d& pt) const
{
	if (_pPolyMeshSearchTree) {
		int numInside = 0;
		for (int i = 0; i < 3; i++) {
			Vector3d v(0, 0, 0);
			v[i] = 1;
			Rayd ray(pt, v);
			size_t count = 0;
			_pPolyMeshSearchTree->biDirRayCastTraverse(ray, [this, &count](const Rayd& ray, const PolyMeshIndex& polyIdx)->bool {
				bool isClosed;
				const auto pFace = getPolygon(polyIdx, isClosed);
				if (isClosed) {

					RayHitd hit;
					if (pFace->intersect(ray, hit)) {
						count++;
					}
				}

				return true;
			});

			if ((count % 2) == 1)
				numInside++;
		}
		// If 2 out of 3 are inside, count it as inside
		return numInside >= 2;
	}
	return false;
}

size_t Model::findPolys(const BOX_TYPE& bbox, const PolyMeshSearchTree::Refiner* pRefiner, std::vector<PolyMeshSearchTree::Entry>& result) const
{
	std::vector<PolyMeshSearchTree::Entry> entries;
	if (_pPolyMeshSearchTree->find(bbox, pRefiner, entries)) {
		for (const auto& entry : entries) {
			const auto& triBox = entry.getBBox();
			if (bbox.intersectsOrContains(triBox, Tolerance::sameDistTol())) {
				result.push_back(entry);
			}
		}
	}

#if ENABLE_MODEL_SEARCH_TREE_VERIFICATION 
	set<PolyMeshSearchTree::Entry> result1;
	std::vector<PolyMeshSearchTree::Entry> entries1;
	auto pSub = getPolySubTree(bbox, nullptr);
	if (pSub) {
		if (pSub->find(bbox, nullptr, entries1)) {
			for (const auto& entry : entries) {
				const auto& triBox = entry.getBBox();
				if (bbox.intersectsOrContains(triBox, Tolerance::sameDistTol())) {
					result1.insert(entry);
				}
			}
		}

	}

#ifdef _DEBUG
	if (result.size() != result1.size())
		assert(!"subTree wrong size");
	for (const auto& e : result)
		assert(result1.contains(e));
#endif // _DEBUG
	result.clear();
	result.insert(result.end(), result1.begin(), result1.end());
#endif

	return result.size();
}

size_t Model::findPolys(const BOX_TYPE& bbox, const PolyMeshSearchTree::Refiner* pRefiner, std::vector<PolyMeshIndex>& result, PolyMeshSearchTree::BoxTestType contains) const
{
	std::vector<PolyMeshSearchTree::Entry> entries;
	if (_pPolyMeshSearchTree->find(bbox, pRefiner, entries)) {
		for (const auto& entry : entries) {
			const auto& triBox = entry.getBBox();
			if (bbox.intersectsOrContains(triBox, Tolerance::sameDistTol())) {
				result.push_back(entry.getIndex());
			}
		}
	}
	return result.size();
}

bool DFHM::Model::rayCast(const Rayd& ray, bool frontFacesOnly, MultiPolyMeshRayHit& hit, bool biDir) const
{
	double minDist = DBL_MAX; // Distance increases deeper in the view. Min distance is closest to the viewer - aka the first hit
	_pPolyMeshSearchTree->biDirRayCastTraverse(ray, [this, biDir, frontFacesOnly, &hit, &minDist](const Rayd& ray, const PolyMeshIndex& polyIdx)->bool {
		bool isClosed;
		const auto pFace = getPolygon(polyIdx, isClosed);
		if (!pFace)
			return true;
		auto pMesh = _modelMeshData[polyIdx.getMeshIdx()]->getPolyMesh();
		pFace->iterateTriangles([&ray, &pMesh, isClosed, &polyIdx, frontFacesOnly, &hit, &minDist, biDir](const Index3DId& idx0, const Index3DId& idx1, const Index3DId& idx2)->bool {
			const Vector3d* pts[] = {
				&pMesh->getVertexPoint(idx0),
				&pMesh->getVertexPoint(idx1),
				&pMesh->getVertexPoint(idx2),
			};

			RayHitd rayHit;
			if (intersectRayTri(ray, pts, rayHit)) {

				if (isClosed && frontFacesOnly) {
					Vector3d v0 = *pts[1] - *pts[0];
					Vector3d v1 = *pts[2] - *pts[0];
					auto norm = v0.cross(v1);
					norm.normalize();
					double dp = ray._dir.dot(norm);
					if (dp > 0)
						return true;
				}

				if (biDir) {
					if (rayHit.dist < minDist) {
						minDist = rayHit.dist;
						hit = MultiPolyMeshRayHit(polyIdx.getMeshIdx(), polyIdx.getPolyId(), rayHit.hitPt, rayHit.dist);
					}
				} else if (rayHit.dist > 0 && rayHit.dist < minDist) {
					minDist = rayHit.dist;
					hit = MultiPolyMeshRayHit(polyIdx.getMeshIdx(), polyIdx.getPolyId(), rayHit.hitPt, rayHit.dist);
				}
			}
			return true;
		});
		return true;
	});

	return minDist != DBL_MAX;
}

const DFHM::Polygon* Model::getPolygon(const PolyMeshIndex& idx) const
{
	bool isClosed;
	return getPolygon(idx, isClosed);
}

const DFHM::Polygon* Model::getPolygon(const PolyMeshIndex& idx, bool& isClosed) const
{
	size_t meshIdx = idx.getMeshIdx();
	assert(meshIdx < _modelMeshData.size());
	auto pMeshData = _modelMeshData.data();
	auto pData = pMeshData[meshIdx];
	if (pData->isActive()) {
		auto pMesh = pData->getPolyMesh();
		isClosed = pMesh->isClosed();
		return &pMesh->getPolygon(idx.getPolyId());
	}
	return nullptr;
}

DFHM::Polygon* Model::getPolygon(const PolyMeshIndex& idx)
{
	bool isClosed;
	return getPolygon(idx, isClosed);
}

DFHM::Polygon* Model::getPolygon(const PolyMeshIndex& idx, bool& isClosed)
{
	size_t meshIdx = idx.getMeshIdx();
	if (meshIdx < _modelMeshData.size()) {
		auto pData = _modelMeshData[meshIdx];
		if (pData->isActive()) {
			auto pMesh = pData->getPolyMesh();
			isClosed = pMesh->isClosed();
			return &pMesh->getPolygon(idx.getPolyId());
		}
	}
	return nullptr;
}

const Vertex* Model::getVertex(const PolyMeshIndex& idx) const
{
	size_t meshIdx = idx.getMeshIdx();
	if (meshIdx < _modelMeshData.size()) {
		auto pData = _modelMeshData[meshIdx];
		auto pMesh = pData->getPolyMesh();
		return &pMesh->getVertex(idx.getPolyId());
	}
	return nullptr;
}
