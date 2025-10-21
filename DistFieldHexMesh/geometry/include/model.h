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

	Copyright Robert R Tipton, 2022, all rights reserved except those granted in prior license statement.

	Dark Sky Innovative Solutions http://darkskyinnovation.com/
*/

#include <defines.h>
#include <memory>

#include <tm_vector3.h>
#include <tm_spatialSearch.h>
#include <triMesh.h>

#include <tolerances.h>
#include <triMeshIndex.h>
#include <polyMeshIndex.h>
#include <triMeshTypeDefs.h>
#include <glMeshData.h>

namespace DFHM {

class DrawModelMesh;
struct SplittingParams;

class Polygon;
class Vertex;

class AppDataIntf;
using AppDataPtr = std::shared_ptr<AppDataIntf>;

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

	using BOX_TYPE = PolyMeshSearchTree::BOX_TYPE;

	void setBounds(const BOX_TYPE& bbox);

	size_t add(const MeshDataPtr& pData);

	void clear();
	bool empty() const;
	size_t size() const;
	size_t numBytes() const;

	const MeshDataPtr& getMeshData(size_t index) const;
	bool isClosed(const PolyMeshIndex& id) const;

	std::vector<MeshDataPtr>::iterator begin();
	std::vector<MeshDataPtr>::iterator end();
	std::vector<MeshDataPtr>::const_iterator begin() const;
	std::vector<MeshDataPtr>::const_iterator end() const;

	bool isPointInside(const Vector3d&  pt) const;
	bool isPointInGap(const SplittingParams& params, const Vector3d& pt, const Vector3d& dir) const;

	size_t findPolys(const BOX_TYPE& bbox, const PolyMeshSearchTree::Refiner* pRefiner, std::vector<PolyMeshSearchTree::Entry>& indices) const;
	size_t findPolys(const BOX_TYPE& bbox, const PolyMeshSearchTree::Refiner* pRefiner, std::vector<PolyMeshIndex>& result, PolyMeshSearchTree::BoxTestType contains = PolyMeshSearchTree::BoxTestType::IntersectsOrContains) const;

	bool rayCast(const Rayd& ray, bool frontFacesOnly, MultiPolyMeshRayHit& hit, bool biDir = true) const;

	std::shared_ptr<const PolyMeshSearchTree> getPolySearchTree() const;
	std::shared_ptr<const PolyMeshSearchTree> getPolySubTree(const BOX_TYPE& bbox, const PolyMeshSearchTree::Refiner* pRefiner) const;

	const Polygon* getPolygon(const PolyMeshIndex& idx) const;
	const Polygon* getPolygon(const PolyMeshIndex& idx, bool& isClosed) const;
	Polygon* getPolygon(const PolyMeshIndex& idx);
	Polygon* getPolygon(const PolyMeshIndex& idx, bool& isClosed);
	const Vertex* getVertex(const PolyMeshIndex& idx) const;

	void rebuildSearchTree();
	void calculateGaps(const SplittingParams& params);
	void clampVerticesToSymPlanes(const std::vector<Planed>& symPlanes);

	const std::map<PolyMeshIndex, Vector3d>& getPolyMeshIdxToGapEndPtMap() const;

private:
	struct SamplePt;
	struct GapTriangularPrism;

	void calculateFaceGaps(const SplittingParams& params, const std::vector<SamplePt>& samplePts, bool startModelIsClosed, Polygon* pStartFace) const;
	void writeGapObj(const Polygon* pStartFace, const Polygon* pNearFace, const GapTriangularPrism& prism) const;

	std::vector<MeshDataPtr> _modelMeshData;
	std::shared_ptr<PolyMeshSearchTree> _pPolyMeshSearchTree;
	std::map<PolyMeshIndex, Vector3d> _polyMeshIdxToGapEndPtMap;
};

inline bool Model::empty() const
{
	return _modelMeshData.empty();
}

inline size_t Model::size() const
{
	return _modelMeshData.size();
}

inline const MeshDataPtr& Model::getMeshData(size_t index) const
{
	return _modelMeshData[index];
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

inline std::shared_ptr<const PolyMeshSearchTree> Model::getPolySearchTree() const
{
	return _pPolyMeshSearchTree;
}

inline std::shared_ptr<const PolyMeshSearchTree> Model::getPolySubTree(const BOX_TYPE& bbox, const PolyMeshSearchTree::Refiner* pRefiner) const
{
	return _pPolyMeshSearchTree->getSubTree(bbox, pRefiner);
}

inline const std::map<PolyMeshIndex, Vector3d>& Model::getPolyMeshIdxToGapEndPtMap() const
{
	return _polyMeshIdxToGapEndPtMap;
}


}
