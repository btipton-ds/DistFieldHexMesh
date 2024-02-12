#pragma once

#include <stdexcept>
#include <vertex.h>
#include <chrono>
#include <mutex>

#include <tm_vector3.h>
#include <triMesh.h>
#include <enums.h>
#include <index3D.h>
#include <objectPool.h>
#include <lambdaMacros.h>
#include <vertex.h>
#include <polygon.h>
#include <polyhedron.h>

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

	using TriMeshVector = std::vector<CMeshPtr>;
	using TriMeshGroup = std::vector<TriMeshVector>;

	Volume* getVolume() override;
	const Volume* getVolume() const override;
	const Index3D& getBlockIdx() const override;
	const Block* getOwner(const Index3D& blockIdx) const override;
	Block* getOutBlockPtr(const Index3D& blockIdx) const override;

	Vector3d invTriLinIterp(const Vector3d& pt) const;
	Vector3d invTriLinIterp(const Vector3d* blockPts, const Vector3d& pt) const;

	Block(Volume* pVol, bool _isOutput, const Index3D& blockIdx, const std::vector<Vector3d>& pts);
	Block(const Block& src);

	size_t blockDim() const;

	// These method determine with block owns an entity based on it's location
	Index3D determineOwnerBlockIdx(const Vector3d& point) const;
	Index3D determineOwnerBlockIdx(const Vertex& vert) const;
	Index3D determineOwnerBlockIdx(const std::vector<Vector3d>& points) const;
	Index3D determineOwnerBlockIdx(const std::vector<Index3DId>& verts) const;
	Index3D determineOwnerBlockIdx(const Polygon& face) const;

	size_t numFaces(bool includeInner) const;
	size_t numPolyhedra() const;

	bool isOutput() const;
	bool verifyTopology() const;
	bool verifyPolyhedronTopology(const Index3DId& cellId) const;
	std::vector<Index3DId> createSubBlocks();

	size_t calLinearSubBlockIndex(const Index3D& subBlockIdx) const;
	Index3D calSubBlockIndexFromLinear(size_t linearIdx) const;
	void addSubBlockFaces();
	void createBlockFaces();

	size_t processTris() const;
	const CMeshPtr& getModelMesh() const;
	CMeshPtr getBlockTriMesh(FaceType meshType, size_t minSplitNum) const;
	glPointsPtr makeFaceEdges(FaceType meshType, size_t minSplitNum) const;

	Index3DId idOfPoint(const Vector3d& pt) const;
	Index3DId addVertex(const Vector3d& pt, const Index3DId& currentId = Index3DId());
	Vector3d getVertexPoint(const Index3DId& vertIdx) const;

	const std::vector<Index3DId>& getFaceVertexIds(const Index3DId& faceId) const;
	Index3DId findFace(const std::vector<Index3DId>& vertIndices) const;
	Index3DId addFace(const std::vector<Index3DId>& vertIndices);
	Index3DId addFace(const Polygon& face);
	void addFaceToLookup(const Index3DId& faceId);
	bool removeFaceFromLookUp(const Index3DId& faceId);

	Index3DId addCell(const Polyhedron& cell);
	Index3DId addCell(const std::set<Index3DId>& faceIds);
	Index3DId addCell(const std::vector<Index3DId>& faceIds);
	Index3DId addHexCell(const Vector3d* blockPts, size_t divs, const Index3D& subBlockIdx, bool intersectingOnly);

	bool vertexExists(const Index3DId& id) const;
	bool polygonExists(const Index3DId& id) const;
	bool polyhedronExists(const Index3DId& id) const;

	// Exists to support mutex locking and does it's own mutex locking
	const Vertex& getVertex_UNSFAFE(const Index3DId& id) const;
	// Exists to support mutex locking and does it's own mutex locking
	const Polygon& getFace_UNSFAFE(const Index3DId& id) const; 

	// pack removes the subBlock array if there's nothing interesting in it. It's a full search of the array and can be time consuming.
	void pack();
	bool isUnloaded() const;
	bool unload(std::string& filename);
	bool load();

	// All of these MUST be thread safe in the sense that the data structure never moves. It's up to the structure to assure 
	// its own thread safety. They are passed by reference because if the object is not in storage
	// that's fatal error for all agorithms and there is no recovery from that.

	LAMBDA_FUNC_PAIR_DECL(vertex);
	LAMBDA_FUNC_PAIR_DECL(face);
	LAMBDA_FUNC_PAIR_DECL(cell);

	template<class LAMBDA>
	void faceFunc2(const Index3DId& id0, const Index3DId& id1, LAMBDA func);

private:
	friend class Volume;
	friend class TestBlock;
	friend class MultiLockGuard;

	enum class AxisIndex {
		X, Y, Z
	};

	void setIsOutput(bool val);
	void getAdjacentBlockIndices(std::set<Index3D>& indices) const;
	Index3D determineOwnerBlockIdxFromRatios(const Vector3d& ratios) const;

	const std::vector<Vector3d>& getCornerPts() const; // Change to returning fractions so we can assign boundary values.
	std::vector<Vector3d> getSubBlockCornerPts(const Vector3d* blockPts, size_t divs, const Index3D& subBlockIdx) const;
	void getBlockEdgeSegs(const Vector3d* blockPts, std::vector<LineSegment>& segs) const;

	Vector3d triLinInterp(const Vector3d* blockPts, size_t divs, const Index3D& pt) const;
	void createSubBlocksForHexSubBlock(const Vector3d* blockPts, const Index3D& subBlockIdx);

	static void addIndexToMap(const Index3D& subBlockIdx, std::set<Index3D>& subBlockIndices);
	Index3DId addFace(const std::vector<Vector3d>& pts);
	Index3DId addFace(int axis, const Index3D& subBlockIdx, const std::vector<Vector3d>& pts);

	void calBlockOriginSpan(Vector3d& origin, Vector3d& span) const;
	bool includeFace(FaceType meshType, size_t minSplitNum, const Polygon& face) const;
	size_t splitAllCellsAtCentroid() const;
	size_t splitAllCellsAtPoint(const Vector3d& pt) const;
	size_t splitByCurvature(double arcAngleDegrees) const;
	size_t finishCellSplits() const;

	Index3D _blockIdx;

	bool _isOutput;
	Volume* _pVol;
	CBoundingBox3Dd 
		_boundBox, // The precise bounding box for this box
		_innerBoundBox; // An inner bounding box with a span of (_blockDim - 0.125) / _blockDim. Any vertex or face which is not completely within the inner box
						// must be tested to see if it belongs to this box or a neighbor box.
						// This required for mutex management for objects which may be modified by more than one box/thread. Items belonging to this box do not require 
						// locking the mutex.Objects which lie on the boundary do require locking.
	size_t _blockDim; // This the dimension of the block = the number of celss across the block

	mutable glPointsVector _blockEdges;
	mutable TriMeshVector _blockMeshes;
	std::vector<Vector3d> _corners;

	std::string _filename;

	ObjectPool<Vertex> _vertices;
	ObjectPool<Polygon> _polygons;
	ObjectPool<Polyhedron> _polyhedra;
};

inline void Block::setIsOutput(bool val)
{
	_isOutput = val;
}

inline bool Block::isOutput() const
{
	return _isOutput;
}

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

inline const Vertex& Block::getVertex_UNSFAFE(const Index3DId& id) const
{
	return _vertices[id];
}

inline const Polygon& Block::getFace_UNSFAFE(const Index3DId& id) const
{
	return _polygons[id];
}

LAMBDA_FUNC_PAIR_IMPL(vertex, _vertices);
LAMBDA_FUNC_PAIR_IMPL(face, _polygons);
LAMBDA_FUNC_PAIR_IMPL(cell, _polyhedra);

template<class LAMBDA>
void Block::faceFunc2(const Index3DId& id0, const Index3DId& id1, LAMBDA func)
{
	if (id1 < id0) {
		faceFunc(id1, [this, &id0, &func](Polygon& face1) {
			faceFunc(id0, [&face1, &func](Polygon& face0) {
				func(face0, face1);
			});
		});
	} else {
		faceFunc(id0, [this, &id1, &func](Polygon& face0) {
			faceFunc(id1, [&face0, &func](Polygon& face1) {
				func(face0, face1);
			});
		});
	}
}


}
