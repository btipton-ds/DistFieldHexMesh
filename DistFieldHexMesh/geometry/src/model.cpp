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
#include <meshData.h>
#include <model.h>
#include <tm_spatialSearch.hpp>
#include <tolerances.h>
#include <splitParams.h>

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
			pPolyMesh->iterateFaces([this, meshIdx](const Index3DId& id, const Polygon& face) {
				auto bb = face.getBBox();
				_pPolyMeshSearchTree->add(bb, PolyMeshIndex(meshIdx, id));

				return true;
			});
		}
	}
}

void Model::calculateGaps(const SplittingParams& params)
{
	if (!_pPolyMeshSearchTree)
		return;

	for (size_t i = 0; i < _modelMeshData.size(); i++) {
		auto& pMeshData = _modelMeshData[i];
		auto& pPolyMesh = pMeshData->getPolyMesh();
		pPolyMesh->iterateVertices([this, &params, i](const Index3DId& id, Vertex& vert)->bool {
			double minDist = DBL_MAX;
			Vector3d closestPt;
			const auto& pt = vert.getPoint();
			const auto& norm = vert.calSurfaceNormal();
			CBoundingBox3Dd bbox(pt);
			bbox.grow(params.gapBoundingBoxSemiSpan);
			std::vector<PolyMeshIndex> hits;
			if (_pPolyMeshSearchTree->find(bbox, nullptr, hits) != 0) {
				for (size_t i = 0; i < hits.size(); i++) {
					const auto& modelFace = getPolygon(hits[i]);
					const auto& faceNorm = modelFace->calUnitNormal();
					double dp = norm.dot(faceNorm);
					if (dp < -0.7071) {
						Vector3d hitPt;
						double minDistToPoint = modelFace->minDistToPoint(pt, hitPt);
						if (minDistToPoint > 0 && minDistToPoint < minDist) {
							minDist = minDistToPoint;
							closestPt = hitPt;
						}
					}
				}
			}

			if (minDist < params.gapBoundingBoxSemiSpan)
				_polyMeshIdxToGapEndPtMap.insert(make_pair(PolyMeshIndex(i, id), closestPt));

			return true;
		});
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
				if (polyIdx.getMeshIdx() >= _modelMeshData.size())
					return true;
				auto pData = _modelMeshData[polyIdx.getMeshIdx()];
				if (!pData || !pData->isActive())
					return true;

				auto& pMesh = pData->getPolyMesh();
				if (!pMesh || !pMesh->isClosed())
					return true;

				const auto& face = pMesh->getPolygon(polyIdx.getPolyId());

				RayHitd hit;
				if (face.intersect(ray, hit)) {
					count++;
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

bool DFHM::Model::rayCast(const Rayd& ray, MultiPolyMeshRayHit& hit, bool biDir) const
{
	double minDist = DBL_MAX; // Distance increases deeper in the view. Min distance is closest to the viewer - aka the first hit
	_pPolyMeshSearchTree->biDirRayCastTraverse(ray, [this, biDir, &hit, &minDist](const Rayd& ray, const PolyMeshIndex& polyIdx)->bool {
		if (polyIdx.getMeshIdx() >= _modelMeshData.size())
			return true;
		auto pData = _modelMeshData[polyIdx.getMeshIdx()];
		if (!pData || !pData->isActive())
			return true;

		auto& pMesh = pData->getPolyMesh();
		if (!pMesh)
			return true;

		const auto& face = pMesh->getPolygon(polyIdx.getPolyId());
		face.iterateTriangles([&ray, &pMesh, &polyIdx, &hit, &minDist, biDir](const Index3DId& idx0, const Index3DId& idx1, const Index3DId& idx2)->bool {
			const Vector3d* pts[] = {
				&pMesh->getVertexPoint(idx0),
				&pMesh->getVertexPoint(idx1),
				&pMesh->getVertexPoint(idx2),
			};

			RayHitd rayHit;
			if (intersectRayTri(ray, pts, rayHit)) {
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
	size_t meshIdx = idx.getMeshIdx();
	if (meshIdx < _modelMeshData.size()) {
		auto pData = _modelMeshData[meshIdx];
		auto pMesh = pData->getPolyMesh();
		return &pMesh->getPolygon(idx.getPolyId());
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
