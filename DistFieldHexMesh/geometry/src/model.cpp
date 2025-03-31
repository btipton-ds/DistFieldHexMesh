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

using namespace std;
using namespace DFHM;

void Model::clear()
{
	_modelMeshData.clear();
	if (_pSearchTree)
		_pSearchTree->clear();
}

size_t Model::numBytes() const
{
	size_t result = _modelMeshData.size();
	for (const auto& pData : _modelMeshData) {
		if (pData)
			result += pData->numBytes();
	}
	if (_pSearchTree)
		result += _pSearchTree->numBytes();
	return result;
}

void Model::setBounds(const BOX_TYPE& bbox)
{
	_pSearchTree = make_shared<SearchTree>(bbox);
}

size_t Model::add(const MeshDataPtr& pData)
{
	size_t meshIdx = _modelMeshData.size();
	_modelMeshData.push_back(pData);

	auto pMesh = pData->getMesh();
	for (size_t idx = 0; idx < pMesh->numTris(); idx++) {
		auto bb = pMesh->getTriBBox(idx);
		_pSearchTree->add(bb, TriMeshIndex(meshIdx, idx));
	}
	pMesh->clearSearchTrees();

	return _modelMeshData.size();
}

size_t Model::addMesh(const TriMesh::CMeshPtr& pMesh, const std::wstring& name)
{
	size_t meshIdx = _modelMeshData.size();
	auto pData = make_shared<MeshData>(pMesh, name);
	_modelMeshData.push_back(pData);

	for (size_t idx = 0; idx < pMesh->numTris(); idx++) {
		auto bb = pMesh->getTriBBox(idx);
		_pSearchTree->add(bb, TriMeshIndex(meshIdx, idx));
	}
	pMesh->clearSearchTrees();

	return _modelMeshData.size();
}

size_t Model::findTris(const BOX_TYPE& bbox, std::vector<SearchTree::Entry>& indices) const
{
	return _pSearchTree->find(bbox, indices);
}

size_t Model::findTris(const BOX_TYPE& bbox, std::vector<TriMeshIndex>& result, SearchTree::BoxTestType contains) const
{
	return _pSearchTree->find(bbox, result, contains);
}

size_t Model::rayCast(const Ray<double>& ray, std::vector<MultiMeshRayHit>& hits, bool biDir) const
{
	vector<TriMeshIndex> hitIndices;
	if (_pSearchTree->biDirRayCast(ray, hitIndices)) {
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
					MultiMeshRayHit MTHit(triIdx2.getMeshIdx(), hit);
					hits.push_back(MTHit);
				}
			}
		}
	}

	sort(hits.begin(), hits.end());
	return hits.size();
}

const TriMesh::CVertex& Model::getVert(const TriMeshIndex& idx) const
{
	return _modelMeshData[idx.getMeshIdx()]->getMesh()->getVert(idx.getTriIdx());
}

const Model::MultMeshTriangle Model::getTri(const TriMeshIndex& idx) const
{
	const auto& triIdx = _modelMeshData[idx.getMeshIdx()]->getMesh()->getTri(idx.getTriIdx());
	MultMeshTriangle result;
	for (int i = 0; i < 3; i++)
		result._vertIds[i] = TriMeshIndex(idx.getMeshIdx(), triIdx[i]);
	return result;
}

const TriMesh::CEdge& Model::getEdge(const TriMeshIndex& idx) const
{
	return _modelMeshData[idx.getMeshIdx()]->getMesh()->getEdge(idx.getTriIdx());
}

double Model::triCurvature(const TriMeshIndex& idx) const
{
	return _modelMeshData[idx.getMeshIdx()]->getMesh()->triCurvature(idx.getTriIdx());
}
