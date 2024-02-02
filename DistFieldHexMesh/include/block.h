#pragma once

#include <stdexcept>
#include <vertex.h>
#include <chrono>
#include <mutex>

#include <tm_vector3.h>
#include <triMesh.h>
#include <index3D.h>
#include <objectPool.h>
#include <blockData.h>

namespace DFHM {

using CMesh = TriMesh::CMesh;
using CMeshPtr = TriMesh::CMeshPtr;

class Volume;
class Edge;
class Polygon;
class Polyhedron;

/*
	The multicore decomposition works on a single block at a time.
	The race conditions occur when two adjacent blocks are working on faces or vertices which lie in the common side of the block.
	Each block has a recursive_mutex.
	Lock the mutex whenever a block is being accessed or manipulated. Most hits will be within the block and the lock/unlock will have
	near zero overhead. When an operation on a block requires another block both blocks need to be locked.

	On this scheme, there is a single mutex for each paired face. There are extra, unused mutexes on the other positive faces.
*/

class Block  : public BlockData {
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

	using TriMeshVector = std::vector<CMeshPtr>;
	using TriMeshGroup = std::vector<TriMeshVector>;

	enum MeshType {
		MT_OUTER,
		MT_INNER,
		MT_BLOCK_BOUNDARY,
		MT_BOUNDARY,
		MT_ALL,
	};
	const Index3D& getBlockIdx() const override;

	Vector3d invTriLinIterp(const Vector3d& pt) const;
	Vector3d invTriLinIterp(const Vector3d* blockPts, const Vector3d& pt) const;

	Block(Volume* pVol, const Index3D& blockIdx, const std::vector<Vector3d>& pts);
	Block(const Block& src) = delete;

	size_t blockDim() const;
	Volume* getVolume();
	const Volume* getVolume() const;
	Block* getOwner(const Index3D& blockIdx);
	const Block* getOwner(const Index3D& blockIdx) const;

	// These method determine with block owns an entity based on it's location
	Index3D determineOwnerBlockIdx(const Vector3d& point) const;
	Index3D determineOwnerBlockIdx(const Vertex& vert) const;
	Index3D determineOwnerBlockIdx(const std::vector<Vector3d>& points) const;
	Index3D determineOwnerBlockIdx(const std::vector<Index3DId>& verts) const;
	Index3D determineOwnerBlockIdx(const Polygon& face) const;

	bool verifyTopology() const;
	bool verifyPolyhedronTopology(const Index3DId& cellId) const;
	std::vector<size_t> createSubBlocks();

	size_t calLinearSubBlockIndex(const Index3D& subBlockIdx) const;
	Index3D calSubBlockIndexFromLinear(size_t linearIdx) const;
	void addSubBlockFaces();
	void createBlockFaces();

	size_t processTris();
	void addTris(const CMeshPtr& pSrcMesh);
	const CMeshPtr& getModelMesh() const;
	CMeshPtr getBlockTriMesh(MeshType meshType, size_t minSplitNum);
	glPointsPtr makeFaceEdges(MeshType meshType, size_t minSplitNum);
	size_t splitAllCellsWithPrinicpalPlanesAtPoint(const Vector3d& splitPt);

	Index3DId idOfPoint(const Vector3d& pt);
	Index3DId addVertex(const Vector3d& pt, size_t currentId = -1);
	std::set<Edge> getVertexEdges(const Index3DId& vertexId) const;

	Vector3d getVertexPoint(const Index3DId& vertIdx) const;
	Index3DId addFace(const std::vector<Index3DId>& vertIndices);
	void addFaceToLookup(const Index3DId& faceId);
	bool removeFaceFromLookUp(const Index3DId& faceId);

	size_t addCell(const std::vector<Index3DId>& faceIds);
	size_t addHexCell(const Vector3d* blockPts, size_t divs, const Index3D& subBlockIdx, bool intersectingOnly);

	void addFaceToPolyhedron(const Index3DId& faceId, const Index3DId& cellId);

	bool vertexExists(const Index3DId& id) const;
	bool polygonExists(const Index3DId& id) const;
	bool polyhedronExists(const Index3DId& id) const;

	// pack removes the subBlock array if there's nothing interesting in it. It's a full search of the array and can be time consuming.
	void pack();
	bool isUnloaded() const;
	bool unload(std::string& filename);
	bool load();

	template<class LAMBDA>
	void vertexFunc(const Index3DId& id, LAMBDA func) const;

	template<class LAMBDA>
	void vertexFunc(const Index3DId& id, LAMBDA func);

	template<class LAMBDA>
	void faceFunc(const Index3DId& id, LAMBDA func) const;

	template<class LAMBDA>
	void faceFunc(const Index3DId& id, LAMBDA func);

	template<class LAMBDA>
	void cellFunc(const Index3DId& id, LAMBDA func) const;

	template<class LAMBDA>
	void cellFunc(const Index3DId& id, LAMBDA func);

private:
	friend class Volume;
	friend class TestBlock;

	enum class AxisIndex {
		X, Y, Z
	};

	Index3D determineOwnerBlockIdxFromRatios(const Vector3d& ratios) const;

	bool dividePolyhedraByCurvature(const std::vector<size_t>& cellIndices, std::vector<size_t>& newCells);

	const std::vector<Vector3d>& getCornerPts() const; // Change to returning fractions so we can assign boundary values.
	std::vector<Vector3d> getSubBlockCornerPts(const Vector3d* blockPts, size_t divs, const Index3D& subBlockIdx) const;
	void getBlockEdgeSegs(const Vector3d* blockPts, std::vector<LineSegment>& segs) const;

	Vector3d triLinInterp(const Vector3d* blockPts, size_t divs, const Index3D& pt) const;
	void createSubBlocksForHexSubBlock(const Vector3d* blockPts, const Index3D& subBlockIdx);

	static void addIndexToMap(const Index3D& subBlockIdx, std::set<Index3D>& subBlockIndices);
	Index3DId addFace(const std::vector<Vector3d>& pts);
	Index3DId addFace(int axis, const Index3D& subBlockIdx, const std::vector<Vector3d>& pts);

	void calBlockOriginSpan(Vector3d& origin, Vector3d& span) const;
	bool includeFace(MeshType meshType, size_t minSplitNum, const Polygon& face) const;

	Volume* _pVol;
	CBoundingBox3Dd 
		_boundBox, // The precise bounding box for this box
		_innerBoundBox; // An inner bounding box with a span of (_blockDim - 0.125) / _blockDim. Any vertex or face which is not completely within the inner box
						// must be tested to see if it belongs to this box or a neighbor box.
						// This required for mutex management for objects which may be modified by more than one box/thread. Items belonging to this box do not require 
						// locking the mutex.Objects which lie on the boundary do require locking.
	size_t _blockDim; // This the dimension of the block = the number of celss across the block

	CMeshPtr _pModelTriMesh;
	glPointsVector _blockEdges;
	TriMeshVector _blockMeshes;
	std::vector<Vector3d> _corners;

	std::string _filename;

};

inline size_t Block::GlPoints::getId() const
{
	return _id;
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

inline const CMeshPtr& Block::getModelMesh() const
{
	return _pModelTriMesh;
}

template<class LAMBDA>
inline void Block::vertexFunc(const Index3DId& id, LAMBDA func) const
{
	auto pOwner = getOwner(id);
	return pOwner->BlockData::vertexFunc(id.elementId(), func);
}

template<class LAMBDA>
inline void Block::vertexFunc(const Index3DId& id, LAMBDA func)
{
	auto pOwner = getOwner(id);
	return pOwner->BlockData::vertexFunc(id.elementId(), func);
}

template<class LAMBDA>
inline void Block::faceFunc(const Index3DId& id, LAMBDA func) const
{
	auto pOwner = getOwner(id);
	return pOwner->BlockData::faceFunc(id.elementId(), func);
}

template<class LAMBDA>
inline void Block::faceFunc(const Index3DId& id, LAMBDA func)
{
	auto pOwner = getOwner(id);
	return pOwner->BlockData::faceFunc(id.elementId(), func);
}

template<class LAMBDA>
inline void Block::cellFunc(const Index3DId& id, LAMBDA func) const
{
	auto pOwner = getOwner(id);
	return pOwner->BlockData::cellFunc(id.elementId(), func);
}

template<class LAMBDA>
inline void Block::cellFunc(const Index3DId& id, LAMBDA func)
{
	auto pOwner = getOwner(id);
	return pOwner->BlockData::cellFunc(id.elementId(), func);
}

}
