#pragma once

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
#include <memory>

#include <tm_vector3.h>
#include <tm_spatialSearch.h>
#include <triMesh.h>
#include <triMeshIndex.h>

namespace DFHM {

class DrawModelMesh;
struct SplittingParams;

class AppData;

class MeshData;
using MeshDataPtr = std::shared_ptr<MeshData>;
using MeshDataConstPtr = std::shared_ptr<const MeshData>;

class Model;
using ModelPtr = std::shared_ptr<Model>;

class Model {
public:
	struct MultMeshTriangle {
		TriMeshIndex& operator[](size_t i);
		const TriMeshIndex& operator[](size_t i) const;

		TriMeshIndex _vertIds[3];
	};

	using SearchTree = CSpatialSearchBase<double, TriMeshIndex, 25>;
	using BOX_TYPE = SearchTree::BOX_TYPE;

	void setBounds(const BOX_TYPE& bbox);

	size_t add(const MeshDataPtr& pData);
	size_t addMesh(const TriMesh::CMeshPtr& pMesh, const std::wstring& name);
	void clear();
	bool empty() const;
	size_t size() const;
	size_t numBytes() const;

	std::vector<MeshDataPtr>::iterator begin();
	std::vector<MeshDataPtr>::iterator end();
	std::vector<MeshDataPtr>::const_iterator begin() const;
	std::vector<MeshDataPtr>::const_iterator end() const;

	size_t findTris(const BOX_TYPE& bbox, std::vector<SearchTree::Entry>& indices) const;
	size_t findTris(const BOX_TYPE& bbox, std::vector<TriMeshIndex>& result, SearchTree::BoxTestType contains = SearchTree::BoxTestType::IntersectsOrContains) const;
	size_t rayCast(const Ray<double>& ray, std::vector<MultiMeshRayHit>& hits, bool biDir = true) const;
	std::shared_ptr<const SearchTree> getSearchTree() const;
	std::shared_ptr<const SearchTree> getSubTree(const BOX_TYPE& bbox) const;

	const TriMesh::CVertex& getVert(const TriMeshIndex& idx) const;
	const MultMeshTriangle getTriIndices(const TriMeshIndex& idx) const;
	bool getTri(const TriMeshIndex& idx, const Vector3d* pts[3]) const;
	const TriMesh::CEdge& getEdge(const TriMeshIndex& idx) const;
	double triCurvature(const TriMeshIndex& idx) const;

private:
	std::vector<MeshDataPtr> _modelMeshData;
	std::shared_ptr<SearchTree> _pSearchTree;

};

inline bool Model::empty() const
{
	return _modelMeshData.empty();
}

inline size_t Model::size() const
{
	return _modelMeshData.size();
}

inline std::vector<MeshDataPtr>::iterator Model::begin()
{
	return _modelMeshData.begin();
}

inline std::vector<MeshDataPtr>::iterator Model::end()
{
	return _modelMeshData.end();
}

inline std::vector<MeshDataPtr>::const_iterator Model::begin() const
{
	return _modelMeshData.begin();
}

inline std::vector<MeshDataPtr>::const_iterator Model::end() const
{
	return _modelMeshData.end();
}

inline TriMeshIndex& Model::MultMeshTriangle::operator[](size_t i)
{
	return _vertIds[i];
}

inline const TriMeshIndex& Model::MultMeshTriangle::operator[](size_t i) const
{
	return _vertIds[i];
}

inline std::shared_ptr<const Model::SearchTree> Model::getSearchTree() const
{
	return _pSearchTree;
}


}
