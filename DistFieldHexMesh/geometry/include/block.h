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

#include <stdexcept>
#include <vertex.h>
#include <chrono>
#include <mutex>
#include <atomic>

#include <defines.h>
#include <tm_vector3.h>
#include <tm_plane.h>
#include <triMesh.h>
#include <tm_spatialSearch.h>
#include <enums.h>
#include <index3D.h>
#include <objectPool.h>
#include <pool_set.h>
#include <logger.h>
#include <lambdaMacros.h>
#include <vertex.h>
#include <unalignedBBox.h>
#include <polygon.h>
#include <polyhedron.h>
#include <vertexSpatialTree.h>
#include <fastBisectionSet.h>

#if USE_MULTI_THREAD_CONTAINERS
#include <local_heap.h>
#endif

namespace DFHM {

using CMesh = TriMesh::CMesh;
using CMeshPtr = TriMesh::CMeshPtr;

class Model;

class MeshData;
using MeshDataPtr = std::shared_ptr<MeshData>;

struct SplittingParams;
class Volume;
using VolumePtr = std::shared_ptr<Volume>;
class Edge;
class Polygon;
class Polyhedron;
class Splitter3D;

class Block : public ObjectPoolOwner {
public:
	class GlPoints : public std::vector<float>
	{
	public:
		GlPoints();
		GlPoints(const GlPoints& src);
		size_t getId() const;
		size_t changeNumber() const;
		void changed();
	private:
		static std::atomic<size_t> _statId;
		size_t _id, _changeNumber = 0;
	};

	using glPointsPtr = std::shared_ptr<GlPoints>;
	using glPointsVector = std::vector<glPointsPtr>;
	using glPointsGroup = std::vector<glPointsVector>;

	struct GlHexFaces {
		// all faces for a block
		void addFace(const Block& blk, const Polygon& face);
		void addTriangle(const Vector3d& pt0, const Vector3d& pt1, const Vector3d& pt2);
		size_t numTriVertices() const;
		size_t numEdgeVertices() const;

		std::vector<float> _glTriPoints, _glTriNormals, _glEdgePoints;
	};

	using GlHexFacesPtr = std::shared_ptr<GlHexFaces>;
	using GlHexFacesVector = std::vector<GlHexFacesPtr>;
	using GlHexMeshGroup = std::vector<GlHexFacesVector>;

	Block(Volume* pVol, const Index3D& blockIdx, const std::vector<Vector3d>& pts, bool forReading = false);
	Block(Volume* pVol, const Index3D& blockIdx, const Vector3d pts[8], bool forReading = false);
	~Block();

	Volume* getVolume() override;
	const Volume* getVolume() const override;
	const SplittingParams& getSplitParams() const;

	const Index3D& getBlockIdx() const override;
	const Block* getOwner(const Index3D& blockIdx) const override;
	Block* getOwner(const Index3D& blockIdx) override;
	const Block* getOwner(const Edge& edge) const;
	Block* getOwner(const Edge& edge);
	const UnalignedBBoxd& getUnalignedBBox() const;

	void clear();

	size_t blockDim() const;

	// These method determine with block owns an entity based on it's location
	Index3D determineOwnerBlockIdx(const Vector3d& point) const;
	Index3D determineOwnerBlockIdx(const Vertex& vert) const;
	Index3D determineOwnerBlockIdx(const MTC::vector<Vector3d>& points) const;
	Index3D determineOwnerBlockIdx(const MTC::vector<Index3DId>& verts) const;
	Index3D determineOwnerBlockIdx(const Polygon& face) const;

	Index3DId getVertexIdOfPoint(const Vector3d& point);
	Index3DId findVertexIdOfPoint(const Vector3d& point) const;

	size_t numFaces(bool includeInner) const;
	size_t numPolyhedra() const;
	size_t numBytes() const;

	const std::vector<Vector3d>& getCornerPts() const;

	bool intersectsModel() const;
	VolumeType getVolumeType() const;
	bool doQualitySplits() const;
	bool verifyTopology() const;
	bool verifyDeterminOwnerBlockIndex() const;
	bool verifyIndices(const Index3D& idx) const;
	void createBlockCells();

	size_t calLinearSubBlockIndex(const Index3D& subBlockIdx) const;
	Index3D calSubBlockIndexFromLinear(size_t linearIdx) const;
	void addSubBlockFaces();
	void createBlockFaces();

	const Model& getModel() const;
	const std::shared_ptr<const PolyMeshSearchTree>& getPolySearchTree() const;
	void deleteModelSearchTree();
	void createHexTriMesh(MeshDrawType meshType, const std::vector<Planed>& planes, GlHexFacesPtr& polys);

	Index3DId addVertex(const Vector3d& pt, const Index3DId& currentId = Index3DId());
	const Vector3d& getVertexPoint(const Index3DId& vertId) const;
	void setVertexLockType(const Index3DId& vertId, VertexLockType val);
	VertexLockType getVertexLockType(const Index3DId& vertId) const;

	CBoundingBox3Dd getBBox() const;

	Index3DId addCell(const Polyhedron& cell, const Index3DId& parentCellId);
	Index3DId addHexCell(const MTC::vector<Index3DId>& cornerVertIds);
	Index3DId createGradedHexCell(const std::vector<Vector3d>& blockPts, size_t divs, const Index3D& subBlockIdx);

	bool vertexExists(const Index3DId& id) const;
	bool polygonExists(const Index3DId& id) const;

	bool polyhedronExists(const Index3DId& id) const;
	bool allCellsClosed() const;

	Index3DId addPolygon(const Polygon& face);
	void addPolygonToLookup(const DFHM::Polygon& face);
	void removePolygonFromLookup(const DFHM::Polygon& face);
	Index3DId findPolygon(const Polygon& face) const;

#if USE_MULTI_THREAD_CONTAINERS			
	MultiCore::local_heap* getHeapPtr();
	const MultiCore::local_heap* getHeapPtr() const;
#endif

	void addToNeedToSplit(const Index3DId& id);
	void addToNeedToSplit(const FastBisectionSet<Index3DId>& ids);

	void addToTouchedCellList(const Index3DId& cellId);
	void removeFromTouchedCellList(const Index3DId& cellId);

	void addToSplitStack(const Index3DId& cellIds);
	bool updateSplitStack(size_t splitNum);
	bool hasPendingSplits() const;

	void resetLayerNums();
	void markIncrementLayerNums(int i);
	void setIncrementLayerNums(int i);

	void freeVertex(const Index3DId& id);
	void freePolygon(const Index3DId& id);
	void freePolyhedron(const Index3DId& id);

	// pack removes the subBlock array if there's nothing interesting in it. It's a full search of the array and can be time consuming.
	void pack();
	bool write(std::ostream& outStream) const;
	void setSupportsReverseLookup(bool val);
	bool read(std::istream& inStream);

	bool isUnloaded() const;
	bool unload(std::string& filename);
	bool load();

	std::string getObjFilePath() const;
	void dumpPolyhedraObj(const MTC::vector<Index3DId>& cellIds, bool includeModel, bool useEdges, bool sharpOnly, const MTC::vector<Vector3d>& pts = MTC::vector<Vector3d>()) const;
	void dumpPolygonObj(const std::string& fileName, const MTC::set<Index3DId>& faceIds, const Index3DId& cellId = Index3DId()) const;
	void dumpPolygonObj(const std::string& fileName, const MTC::vector<Index3DId>& faceIds, const Index3DId& cellId = Index3DId()) const;
	void dumpEdgeObj(std::string& fileName, const MTC::set<EdgeKey>& edges) const;

	template<class F>
	void iterateVerticesInOrder(F fLambda) const;

	template<class F>
	void iterateVerticesInOrder(F fLambda);

	template<class F>
	void iteratePolygonsInOrder(F fLambda) const;

	template<class F>
	void iteratePolygonsInOrder(F fLambda);

	template<class F>
	void iteratePolyhedraInOrder(F fLambda) const;

	template<class F>
	void iteratePolyhedraInOrder(F fLambda);

	LAMBDA_BLOCK_DECLS(vertex, Index3DId, Vertex)
	LAMBDA_BLOCK_DECLS(face, Index3DId, Polygon)
	LAMBDA_BLOCK_DECLS(cell, Index3DId, Polyhedron)
	LAMBDA_BLOCK_DECLS(edge, EdgeKey, Edge)

private:
	friend class PolymeshTables;
	friend class PolyMesh;
	friend class Volume;
	friend class TestBlock;
	friend class MultiLockGuard;
	friend class Splitter3D;

	enum class AxisIndex {
		X, Y, Z
	};

	using SearchTree = VertSearchTreeIndex3DId_4;
	using SearchTreePtr = std::shared_ptr<SearchTree>;
	using SearchTreeConstPtr = std::shared_ptr<const SearchTree>;

	void remapBlockIndices(const std::vector<size_t>& idRemap, const Index3D& srcDims);
	void setIsOutput(bool val);
	void getAdjacentBlockIndices(MTC::set<Index3D>& indices) const;
	Index3D determineOwnerBlockIdxFromRatios(const Vector3d& ratios) const;

	MTC::vector<Index3DId> getSubBlockCornerVertIds(const std::vector<Vector3d>& blockPts, size_t divs, const Index3D& subBlockIdx);

	Vector3d triLinInterp(const Vector3d* blockPts, size_t divs, const Index3D& pt) const;
	void createSubBlocksForHexSubBlock(const Vector3d* blockPts, const Index3D& subBlockIdx);

	void calBlockOriginSpan(Vector3d& origin, Vector3d& span) const;
	bool includeFaceInDrawKey(MeshDrawType meshType, const std::vector<Planed>& planes, const Polygon& face) const;

	void dumpOpenCells() const;

	bool splitComplexPolyhedra(const SplittingParams& params, size_t splitNum);
	bool splitRequiredPolyhedra(const SplittingParams& params, size_t splitNum);

	VolumePtr getScratchVolume();

#ifdef _DEBUG
	bool isPolygonInUse(const Index3DId& faceId) const;
	bool isPolyhedronInUse(const Index3DId& cellId) const;
#endif // _DEBUG

	mutable VolumeType _volType = VOLTYPE_UNKNOWN;

	Index3D _blockIdx;

	Volume* _pVol;

	CBoundingBox3Dd
		_boundBox,      // The precise bounding box for this box
		_innerBoundBox; // An inner bounding box with a span of (_blockDim - 0.125) / _blockDim. Any vertex or face which is not completely within the inner box
						// must be tested to see if it belongs to this box or a Adjacent box.
						// This required for mutex management for objects which may be modified by more than one box/thread. Items belonging to this box do not require 
						// locking the mutex.Objects which lie on the boundary do require locking.
	
	size_t _blockDim;   // This is the dimension of the block = the number of cells across the block

	UnalignedBBoxd _corners;

	std::string _filename;

	std::set<Index3DId> _needToSplit, _touchedCellIds;

	ObjectPool<Vertex> _vertices;
	ObjectPool<Polygon> _polygons;
	ObjectPool<Polyhedron> _polyhedra;

	mutable bool _searchTreeSet = false;
	mutable std::shared_ptr<const PolyMeshSearchTree> _pPolySearchTree;
#if USE_MULTI_THREAD_CONTAINERS
	MultiCore::local_heap _heap;
#endif
};

inline size_t Block::GlPoints::getId() const
{
	return _id;
}

inline const UnalignedBBoxd& Block::getUnalignedBBox() const
{
	return _corners;
}

inline size_t Block::GlPoints::changeNumber() const
{
	return _changeNumber;
}

inline void Block::GlPoints::changed()
{
	_changeNumber++;
}

inline size_t Block::blockDim() const
{
	return _blockDim;
}

#if USE_MULTI_THREAD_CONTAINERS			
inline MultiCore::local_heap* Block::getHeapPtr()
{
	return &_heap;
}

inline const MultiCore::local_heap* Block::getHeapPtr() const
{
	return &_heap;
}
#endif

inline Volume* Block::getVolume()
{
	return _pVol;
}

inline const Volume* Block::getVolume() const
{
	return _pVol;
}

inline size_t Block::calLinearSubBlockIndex(const Index3D& subBlockIdx) const
{
	if (subBlockIdx.isInBounds(_blockDim))
		return subBlockIdx[0] + _blockDim * (subBlockIdx[1] + _blockDim * subBlockIdx[2]);
	return -1;
}

inline bool Block::isUnloaded() const
{
	return !_filename.empty();
}

template<class F>
inline void Block::iterateVerticesInOrder(F fLambda) const
{
	_vertices.iterateInOrder(fLambda);
}

template<class F>
inline void Block::iterateVerticesInOrder(F fLambda)
{
	_vertices.iterateInOrder(fLambda);
}

template<class F>
inline void Block::iteratePolygonsInOrder(F fLambda) const
{
	_polygons.iterateInOrder(fLambda);
}

template<class F>
inline void Block::iteratePolygonsInOrder(F fLambda)
{
	_polygons.iterateInOrder(fLambda);
}

template<class F>
inline void Block::iteratePolyhedraInOrder(F fLambda) const
{
	_polyhedra.iterateInOrder(fLambda);
}

template<class F>
inline void Block::iteratePolyhedraInOrder(F fLambda)
{
	_polyhedra.iterateInOrder(fLambda);
}

inline void Block::addToNeedToSplit(const Index3DId& id)
{
	auto pOwner = getOwner(id);
	if (pOwner)
		pOwner->_needToSplit.insert(id);
}

inline void Block::addToNeedToSplit(const FastBisectionSet<Index3DId>& ids)
{
	for (const auto& id : ids) {
		auto pOwner = getOwner(id);
		if (pOwner)
			pOwner->_needToSplit.insert(id);
	}
}

inline void Block::addToTouchedCellList(const Index3DId& cellId)
{
	auto pOwner = getOwner(cellId);
	if (pOwner) {
		pOwner->_touchedCellIds.insert(cellId);
	}
}

inline void Block::removeFromTouchedCellList(const Index3DId& cellId)
{
	auto pOwner = getOwner(cellId);
	if (pOwner) {
		pOwner->_touchedCellIds.erase(cellId);
	}

}

}
