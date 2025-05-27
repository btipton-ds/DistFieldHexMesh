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
#include <meshData.h>
#include <model.h>
#include <tm_spatialSearch.hpp>
#include <tolerances.h>

using namespace std;
using namespace DFHM;

void Model::clear()
{
	_modelMeshData.clear();
	if (_pTriSearchTree)
		_pTriSearchTree->clear();
}

size_t Model::numBytes() const
{
	size_t result = _modelMeshData.size();
	for (const auto& pData : _modelMeshData) {
		if (pData)
			result += pData->numBytes();
	}
	if (_pTriSearchTree)
		result += _pTriSearchTree->numBytes();
	return result;
}

void Model::setBounds(const BOX_TYPE& bbox)
{
	_pTriSearchTree = make_shared<TriSearchTree>(bbox);
}

size_t Model::add(const MeshDataPtr& pData)
{
	CBoundingBox3Dd bbox;
	for (const auto& model : _modelMeshData) {
		bbox.merge(model->getMesh()->getBBox());
	}
	bbox.merge(pData->getMesh()->getBBox());
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
		auto& pMesh = pData->getMesh();
		if (pMesh) {
			bbox.merge(pMesh->getBBox());
		}
	}

	_pTriSearchTree = make_shared<TriSearchTree>(bbox);

	bbox.grow(Polygon::bboxOffsetDist());
	_pPolyMeshSearchTree = make_shared<PolyMeshSearchTree>(bbox);

	for (size_t meshIdx = 0; meshIdx < _modelMeshData.size(); meshIdx++) {
		auto& pData = _modelMeshData[meshIdx];
		auto& pMesh = pData->getMesh();
		if (pMesh) {
			for (size_t idx = 0; idx < pMesh->numTris(); idx++) {
				auto bb = pMesh->getTriBBox(idx);
				_pTriSearchTree->add(bb, TriMeshIndex(meshIdx, idx));
			}
		}

		auto& pPolyMesh = pData->getPolyMesh();

		if (pMesh) {
			pPolyMesh->iterateFaces([this, meshIdx](const Index3DId& id, const Polygon& face) {
				auto bb = face.getBBox();
				_pPolyMeshSearchTree->add(bb, PolyMeshIndex(meshIdx, id));

				return true;
			});
		}
	}
}

size_t Model::addMesh(const AppDataPtr& pAppData, const TriMesh::CMeshPtr& pMesh, const std::wstring& name)
{
	size_t meshIdx = _modelMeshData.size();
	auto pData = make_shared<MeshData>(pAppData, pMesh, name);
	_modelMeshData.push_back(pData);

	for (size_t idx = 0; idx < pMesh->numTris(); idx++) {
		auto bb = pMesh->getTriBBox(idx);
		_pTriSearchTree->add(bb, TriMeshIndex(meshIdx, idx));
	}
	pMesh->clearSearchTrees();

	return _modelMeshData.size();
}

bool Model::doesTriIntersect(const Model::TriSearchTree::Entry& entry, const Model::BOX_TYPE& bbox) const
{
	const auto tol = Tolerance::sameDistTol();
	const Vector3d* pts[3];
	if (getTri(entry.getIndex(), pts)) {
		return bbox.intersectsOrContains(*pts[0], *pts[1], *pts[2], tol);
	}
	return  false;
}

#if !USE_POLYMESH
size_t Model::findTris(const BOX_TYPE& bbox, std::vector<TriSearchTree::Entry>& result) const
{
	std::vector<TriSearchTree::Entry> entries;
	if (_pTriSearchTree->find(bbox, entries)) {
		for (const auto& entry : entries) {
			const auto& triBox = entry.getBBox();
			if (bbox.intersectsOrContains(triBox, Tolerance::sameDistTol())) {
				result.push_back(entry);
			}
		}
	}

#if DO_MODEL_SEARCH_TREE_VERIFICATION 
	set<TriSearchTree::Entry> result1;
	std::vector<TriSearchTree::Entry> entries1;
	auto pSub = getTriSubTree(bbox);
	if (pSub) {
		if (pSub->find(bbox, entries1)) {
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

size_t Model::findTris(const BOX_TYPE& bbox, std::vector<TriMeshIndex>& result, TriSearchTree::BoxTestType contains) const
{
	std::vector<TriSearchTree::Entry> entries;
	if (_pTriSearchTree->find(bbox, entries)) {
		for (const auto& entry : entries) {
			const auto& triBox = entry.getBBox();
			if (bbox.intersectsOrContains(triBox, Tolerance::sameDistTol())) {
				result.push_back(entry.getIndex());
			}
		}
	}
	return result.size();
}

size_t Model::rayCast(const Ray<double>& ray, std::vector<MultiTriMeshRayHit>& hits, bool biDir) const
{
	vector<TriMeshIndex> hitIndices;
	if (_pTriSearchTree->biDirRayCast(ray, hitIndices)) {
		for (const auto& triIdx2 : hitIndices) {
			auto pData = _modelMeshData[triIdx2.getMeshIdx()];
			if (!pData->isActive())
				continue;

			auto& pMesh = pData->getMesh();
			const auto& tri = pMesh->getTri(triIdx2.getTriIdx());
			const Vector3d* pts[] = {
				&pMesh->getVert(tri[0])._pt,
				&pMesh->getVert(tri[1])._pt,
				&pMesh->getVert(tri[2])._pt,
			};

			RayHitd hit;
			if (intersectRayTri(ray, pts, hit)) {
				if (biDir || hit.dist > 0) {
					MultiTriMeshRayHit MTHit(triIdx2.getMeshIdx(), hit);
					hits.push_back(MTHit);
				}
			}
		}
	}

	sort(hits.begin(), hits.end());
	return hits.size();
}

std::shared_ptr<const Model::TriSearchTree> Model::getTriSubTree(const BOX_TYPE& bbox) const
{
	auto refineFunc = [this](const Model::TriSearchTree::Entry& entry, const Model::BOX_TYPE& bbox)->bool {
		return doesTriIntersect(entry, bbox);
		};

	auto p = _pTriSearchTree->getSubTree(bbox, refineFunc);

#if DO_MODEL_SEARCH_TREE_VERIFICATION
	vector<Model::TriSearchTree::Entry> testFull, testClipped;
	_pTriSearchTree->find(bbox, refineFunc, testFull);
	if (!testFull.empty()) {
		if (!p) {
			stringstream ss;
			ss << "SubTree is null " << __FILE__ << ":" << __LINE__;
			throw runtime_error(ss.str());
		}

		p->find(bbox, refineFunc, testClipped);
		if (testFull.size() != testClipped.size()) {
			stringstream ss;
			ss << "SubTree error " << __FILE__ << ":" << __LINE__;
			throw runtime_error(ss.str());
		}
		else {
			set<Model::TriSearchTree::Entry> test;
			test.insert(testClipped.begin(), testClipped.end());
			for (const auto& entry : testFull) {
				if (!test.contains(entry)) {
					stringstream ss;
					ss << "Entry missing in subTree " << __FILE__ << ":" << __LINE__;
					throw runtime_error(ss.str());
				}
			}
		}
	}
	else if (!testFull.empty()) {
		stringstream ss;
		ss << "SubTree should have contents " << __FILE__ << ":" << __LINE__;
		throw runtime_error(ss.str());
	}
#endif
	return p;
}

const Model::MultMeshTriangle Model::getTriIndices(const TriMeshIndex& idx) const
{
	const auto& triIdx = _modelMeshData[idx.getMeshIdx()]->getMesh()->getTri(idx.getTriIdx());
	MultMeshTriangle result;
	for (int i = 0; i < 3; i++)
		result._vertIds[i] = TriMeshIndex(idx.getMeshIdx(), triIdx[i]);
	return result;
}

#endif

bool Model::getTri(const TriMeshIndex& idx, const Vector3d* pts[3]) const
{
	auto pMesh = _modelMeshData[idx.getMeshIdx()]->getMesh().get();

	if (pMesh) {
		auto tri = pMesh->getTri(idx.getTriIdx());
		for (int i = 0; i < 3; i++) {
			pts[i] = &pMesh->getVert(tri[i])._pt;
		}
		return true;
	}
	return false;
}

bool Model::doesPolyIntersect(const Model::PolyMeshSearchTree::Entry& entry, const Model::BOX_TYPE& bbox) const
{
	const auto tol = Tolerance::sameDistTol();

	bool result = false;
	if (!bbox.intersects(entry.getBBox(), tol))
		return result;

	const auto polyIdx = entry.getIndex();
	auto pData = _modelMeshData[polyIdx.getMeshIdx()];
	if (!pData->isActive())
		return result;

	auto& pMesh = pData->getPolyMesh();
	const auto& face = pMesh->getPolygon(polyIdx.getPolyId());
	face.iterateTriangles([&pMesh, &polyIdx, &bbox, &result, tol](const Index3DId& idx0, const Index3DId& idx1, const Index3DId& idx2)->bool {
		const Vector3d* pts[] = {
			&pMesh->getVertexPoint(idx0),
			&pMesh->getVertexPoint(idx1),
			&pMesh->getVertexPoint(idx2),
		};
		if (bbox.intersectsOrContains(*pts[0], *pts[1], *pts[2], tol)) {
			result = true;
		}
		return !result;
	});

	return result;
}

size_t Model::findPolys(const BOX_TYPE& bbox, std::vector<PolyMeshSearchTree::Entry>& result) const
{
	std::vector<PolyMeshSearchTree::Entry> entries;
	if (_pPolyMeshSearchTree->find(bbox, entries)) {
		for (const auto& entry : entries) {
			const auto& triBox = entry.getBBox();
			if (bbox.intersectsOrContains(triBox, Tolerance::sameDistTol())) {
				result.push_back(entry);
			}
		}
	}

#if DO_MODEL_SEARCH_TREE_VERIFICATION 
	set<TriSearchTree::Entry> result1;
	std::vector<TriSearchTree::Entry> entries1;
	auto pSub = getTriSubTree(bbox);
	if (pSub) {
		if (pSub->find(bbox, entries1)) {
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

size_t Model::findPolys(const BOX_TYPE& bbox, std::vector<PolyMeshIndex>& result, PolyMeshSearchTree::BoxTestType contains) const
{
	std::vector<PolyMeshSearchTree::Entry> entries;
	if (_pPolyMeshSearchTree->find(bbox, entries)) {
		for (const auto& entry : entries) {
			const auto& triBox = entry.getBBox();
			if (bbox.intersectsOrContains(triBox, Tolerance::sameDistTol())) {
				result.push_back(entry.getIndex());
			}
		}
	}
	return result.size();
}

size_t Model::rayCast(const Ray<double>& ray, std::vector<MultiPolyMeshRayHit>& hits, bool biDir) const
{
	vector<PolyMeshIndex> hitIndices;
	if (_pPolyMeshSearchTree->biDirRayCast(ray, hitIndices)) {
		for (const auto& polyIdx : hitIndices) {
			auto pData = _modelMeshData[polyIdx.getMeshIdx()];
			if (!pData->isActive())
				continue;

			auto& pMesh = pData->getPolyMesh();
			const auto& face = pMesh->getPolygon(polyIdx.getPolyId());
			face.iterateTriangles([&ray, &pMesh, &polyIdx, biDir, &hits](const Index3DId& idx0, const Index3DId& idx1, const Index3DId& idx2)->bool {
				const Vector3d* pts[] = {
					&pMesh->getVertexPoint(idx0),
					&pMesh->getVertexPoint(idx1),
					&pMesh->getVertexPoint(idx2),
				};

				RayHitd hit;
				if (intersectRayTri(ray, pts, hit)) {
					if (biDir || hit.dist > 0) {
						MultiPolyMeshRayHit MTHit(polyIdx.getMeshIdx(), polyIdx.getPolyId(), hit.hitPt, hit.dist);
						hits.push_back(MTHit);
					}
				}
				return true;
			});
		}
	}

	sort(hits.begin(), hits.end());
	return hits.size();
}

std::shared_ptr<const Model::PolyMeshSearchTree> Model::getPolySearchTree() const
{
	return _pPolyMeshSearchTree;
}
std::shared_ptr<const Model::PolyMeshSearchTree> Model::getPolySubTree(const BOX_TYPE& bbox) const
{
	return _pPolyMeshSearchTree->getSubTree(bbox);
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
