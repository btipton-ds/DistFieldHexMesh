#pragma once

#include <stdexcept>
#include <triMesh.h>
#include <objectPoolWMutex.h>
#include <index3D.h>
#include <vertex.h>
#include <mutex>
#include <tm_vector3.h>

namespace DFHM {

class Volume;
class Edge;
class Polygon;
class Polyhedron;

/*
	The multicore decomposition works on a single block at a time.
	The race conditions occur when two adjacent blocks are working on faces or vertices which like in the common side face of the block.
	Each block has a mutex for each positive face of the block - 3 mutexes
	When working on any face, the common face must be locked. For the negative faces, that requires fetching the adjacent block and locking the matching
	positive face.

	On this scheme, there is a single mutex for each paired face. There are extra, unused mutexes on the other positive faces.
*/

class Block : public ObjectPoolOwner {
public:
	const Index3D& getBlockIdx() const override;

	Vector3d invTriLinIterp(const Vector3d& pt) const;
	Vector3d invTriLinIterp(const Vector3d* blockPts, const Vector3d& pt) const;

	Block(Volume* pVol, const Index3D& blockIdx, const std::vector<Vector3d>& pts);
	Block(const Block& src) = delete;
	~Block(); // NOT virtual - do not inherit

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

	std::vector<size_t> createSubBlocks();

	size_t calLinearSubBlockIndex(const Index3D& subBlockIdx) const;
	Index3D calSubBlockIndexFromLinear(size_t linearIdx) const;
	void addSubBlockFaces();
	void createBlockFaces();

	size_t numFaces(bool includeInner) const;
	size_t numPolyhedra() const;

	size_t processTris();
	void addTris(const TriMesh::CMeshPtr& pSrcMesh);
	const TriMesh::CMeshPtr& getModelMesh() const;
	TriMesh::CMeshPtr getBlockTriMesh(bool outerOnly, size_t minSplitNum) const;
	std::shared_ptr<std::vector<float>> makeFaceEdges(bool outerOnly, size_t minSplitNum) const;
	void splitCellsWithPlane(const Plane& splitPlane);

	Index3DId idOfPoint(const Vector3d& pt);
	Index3DId addVertex(const Vector3d& pt, size_t currentId = -1);
	std::set<Edge> getVertexEdges(const Index3DId& vertexId) const;

	Vector3d getVertexPoint(const Index3DId& vertIdx) const;
	Index3DId addFace(const std::vector<Index3DId>& vertIndices);
	void addToLookup(const Polygon& face);
	bool removeFromLookUp(const Polygon& face);

	size_t addCell(const std::vector<Index3DId>& faceIds);
	size_t addHexCell(const Vector3d* blockPts, size_t divs, const Index3D& subBlockIdx, bool intersectingOnly);
	const Polyhedron& getPolyhedron(const Index3DId& cellId) const;
	Polyhedron& getPolyhedron(const Index3DId& cellId);

	bool vertexExists(const Index3DId& id) const;
	bool polygonExists(const Index3DId& id) const;
	bool polyhedronExists(const Index3DId& id) const;

	// pack removes the subBlock array if there's nothing interesting in it. It's a full search of the array and can be time consuming.
	void pack();
	bool isUnloaded() const;
	bool unload(std::string& filename);
	bool load();

	std::recursive_mutex& getEdgeMutex() const;
	std::recursive_mutex& getFaceMutex() const;
	std::recursive_mutex& getVertexMutex() const;

	template<class LAMBDA>
	void vertexFunc(const Index3DId& vertId, LAMBDA func) const;

	template<class LAMBDA>
	void vertexFunc(const Index3DId& vertId, LAMBDA func);

	template<class LAMBDA>
	void faceFunc(const Index3DId& faceId, LAMBDA func) const;

	template<class LAMBDA>
	void faceFunc(const Index3DId& faceId, LAMBDA func);

	template<class LAMBDA>
	void cellFunc(const size_t& cellId, LAMBDA func) const;

	template<class LAMBDA>
	void cellFunc(const size_t& cellId, LAMBDA func);

private:
	friend class Volume;
	friend class TestBlock;

	enum class AxisIndex {
		X, Y, Z
	};

	Index3D determineOwnerBlockIdxFromRatios(const Vector3d& ratios) const;

	std::vector<size_t> dividePolyhedraByCurvature(const std::vector<size_t>& cellIndices);

	const std::vector<Vector3d>& getCornerPts() const; // Change to returning fractions so we can assign boundary values.
	std::vector<Vector3d> getSubBlockCornerPts(const Vector3d* blockPts, size_t divs, const Index3D& subBlockIdx) const;
	void getBlockEdgeSegs(const Vector3d* blockPts, std::vector<LineSegment>& segs) const;

	Vector3d triLinInterp(const Vector3d* blockPts, size_t divs, const Index3D& pt) const;
	void createSubBlocksForHexSubBlock(const Vector3d* blockPts, const Index3D& subBlockIdx);

	static void addIndexToMap(const Index3D& subBlockIdx, std::set<Index3D>& subBlockIndices);
	Index3DId addFace(const std::vector<Vector3d>& pts);
	Index3DId addFace(int axis, const Index3D& subBlockIdx, const std::vector<Vector3d>& pts);

	void divideSubBlock(const Index3D& subBlockIdx, size_t divs);
	void calBlockOriginSpan(Vector3d& origin, Vector3d& span) const;

	std::string _filename;

	Volume* _pVol;
	CBoundingBox3Dd 
		_boundBox, // The precise bounding box for this box
		_innerBoundBox; // An inner bounding box with a span of (_blockDim - 0.125) / _blockDim. Any vertex or face which is not completely within the inner box
						// must be tested to see if it belongs to this box or a neighbor box.
						// This required for mutex management for objects which may be modified by more than one box/thread. Items belonging to this box do not require 
						// locking the mutex.Objects which lie on the boundary do require locking.
	Index3D _blockIdx;
	size_t _blockDim; // This the dimension of the block = the number of celss across the block

	TriMesh::CMeshPtr _pModelTriMesh;
	std::vector<Vector3d> _corners;

	ObjectPoolWMutex<Vertex> _vertices;
	ObjectPoolWMutex<Polygon> _polygons;
	ObjectPool<Polyhedron> _polyhedra;
};

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

inline const TriMesh::CMeshPtr& Block::getModelMesh() const
{
	return _pModelTriMesh;
}

inline std::recursive_mutex& Block::getVertexMutex() const
{
	return _vertices.getMutex();
}

inline std::recursive_mutex& Block::getFaceMutex() const
{
	return _polygons.getMutex();
}

template<class LAMBDA>
void Block::vertexFunc(const Index3DId& vertId, LAMBDA func) const
{
	auto pOwner = getOwner(vertId);
	std::lock_guard g(pOwner->_vertices);
	func(pOwner, pOwner->_vertices[vertId.elementId()]);
}

template<class LAMBDA>
void Block::vertexFunc(const Index3DId& vertId, LAMBDA func)
{
	auto pOwner = getOwner(vertId);
	std::lock_guard g(pOwner->_vertices);
	func(pOwner, pOwner->_vertices[vertId.elementId()]);
}

template<class LAMBDA>
void Block::faceFunc(const Index3DId& faceId, LAMBDA func) const
{
	auto pOwner = getOwner(faceId);
	std::lock_guard g(pOwner->_polygons);
	func(pOwner, pOwner->_polygons[faceId.elementId()]);
}

template<class LAMBDA>
void Block::faceFunc(const Index3DId& faceId, LAMBDA func)
{
	auto pOwner = getOwner(faceId);
	std::lock_guard g(pOwner->_polygons);
	func(pOwner, pOwner->_polygons[faceId.elementId()]);
}

template<class LAMBDA>
void Block::cellFunc(const size_t& cellId, LAMBDA func) const
{
	func(this, _polyhedra[cellId]);
}

template<class LAMBDA>
void Block::cellFunc(const size_t& cellId, LAMBDA func)
{
	func(this, _polyhedra[cellId]);
}

}
