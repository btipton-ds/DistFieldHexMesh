#pragma once

#include <stdexcept>
#include <triMesh.h>
#include <objectPoolWMutex.h>
#include <index3D.h>
#include <subBlock.h>
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

class Block {
public:
	friend class SubBlock;

	static Vector3d invTriLinIterp(const Vector3d* blockPts, const Vector3d& pt);

	Block(Volume* pVol, const Index3D& blockIdx, const std::vector<Vector3d>& pts);
	Block(const Block& src) = delete;
	~Block(); // NOT virtual - do not inherit

	size_t blockDim() const;
	Volume* getVolume();
	const Volume* getVolume() const;
	Block* getOwner(const Index3D& blockIdx);
	const Block* getOwner(const Index3D& blockIdx) const;

	size_t createSubBlocks();

	size_t calLinearSubBlockIndex(const Index3D& subBlockIdx) const;
	Index3D calSubBlockIndexFromLinear(size_t linearIdx) const;
	void addSubBlockFaces();
	void createBlockFaces();

	size_t numSubBlocks() const;
	bool subBlockExists(const Index3D& subBlockIdx) const;
	SubBlock& getSubBlock(const Index3D& subBlockIdx);
	const SubBlock& getSubBlock(const Index3D& subBlockIdx) const;
	void freeSubBlock(const Index3D& subBlockIdx);

	size_t numFaces(bool includeInner) const;
	size_t numPolyhedra() const;

	inline const Index3D& getBlockIdx()
	{
		return _blockIdx;
	}

	void processTris(const TriMesh::CMeshPtr& pSrcMesh);
	size_t processTris();
	void addTris(const TriMesh::CMeshPtr& pSrcMesh);
	const TriMesh::CMeshPtr& getModelMesh() const;
	TriMesh::CMeshPtr getBlockTriMesh(bool outerOnly) const;
	std::shared_ptr<std::vector<float>> makeFaceEdges(bool outerOnly) const;

	Index3DId addVertex(const Index3D& blockIdx, const Vector3d& pt, size_t currentId = -1);
	std::set<Edge> getVertexEdges(const Index3DId& vertexId) const;

	Vector3d getVertexPoint(const Index3DId& vertIdx) const;

	// pack removes the subBlock array if there's nothing interesting in it. It's a full search of the array and can be time consuming.
	void pack();
	bool isUnloaded() const;
	bool unload(std::string& filename);
	bool load();

	template<class LAMBDA>
	void vertexFunc(const Index3DId& vertId, LAMBDA func) const;

	template<class LAMBDA>
	void vertexFunc(const Index3DId& vertId, LAMBDA func);

	template<class LAMBDA>
	void faceFunc(const Index3DId& faceId, LAMBDA func) const;

	template<class LAMBDA>
	void faceFunc(const Index3DId& faceId, LAMBDA func);

	template<class LAMBDA>
	void facesFunc(LAMBDA func) const;

	template<class LAMBDA>
	void facesFunc(LAMBDA func);

private:
	friend class Volume;
	friend class TestBlock;

	enum class AxisIndex {
		X, Y, Z
	};

	struct CrossBlockPoint {
		inline operator const Vector3d& () const
		{
			return _pt;
		}

		Vector3d _pt;
		Index3D _ownerBlockIdx;
	};

	std::mutex& getEdgeMutex() const;
	std::mutex& getFaceMutex() const;
	std::mutex& getVertexMutex() const;

	void dividePolyhedra();
	void dividePolyhedraAtSharpVerts();
	void dividePolyhedraByCurvature();

	const Vector3d* getCornerPts() const; // Change to returning fractions so we can assign boundary values.
	std::vector<CrossBlockPoint> getSubBlockCornerPts(const Vector3d* blockPts, size_t blockDim, const Index3D& subBlockIdx) const;
	void getBlockEdgeSegs(const Vector3d* blockPts, std::vector<LineSegment>& segs) const;

	size_t rayCastFace(const Vector3d* pts, size_t samples, int axis, FaceRayHits& rayTriHits) const;
	void rayCastHexBlock(const Vector3d* pts, size_t blockDim, FaceRayHits _rayTriHits[3]);
	CrossBlockPoint triLinInterp(const Vector3d* blockPts, size_t blockDim, const Index3D& pt) const;
	void createSubBlocksForHexSubBlock(const Vector3d* blockPts, const Index3D& subBlockIdx);
	void addSubBlockIndicesForMeshVerts(std::set<Index3D>& subBlockIndices);
	void addSubBlockIndicesForRayHits(std::set<Index3D>& subBlockIndices);
	void splitAtSharpVertices(const Vector3d* blockPts, const Index3D& subBlockIdx);
	void splitAtLegIntersections(const Vector3d* blockPts, const Index3D& subBlockIdx);

	void addHitsForRay(size_t axis, size_t i, size_t j, size_t ii, size_t jj, std::set<Index3D>& subBlockIndices);
	static void addIndexToMap(const Index3D& subBlockIdx, std::set<Index3D>& subBlockIndices);
	size_t addHexCell(const Vector3d* blockPts, size_t blockDim, const Index3D& subBlockIdx);
	Index3DId addFace(const std::vector<CrossBlockPoint>& pts);
	Index3DId addFace(int axis, const Index3D& subBlockIdx, const std::vector<CrossBlockPoint>& pts);

	void divideSubBlock(const Index3D& subBlockIdx, size_t divs);
	void calBlockOriginSpan(Vector3d& origin, Vector3d& span) const;

	std::string _filename;

	Volume* _pVol;
	Index3D _blockIdx;
	size_t _blockDim; // This the dimension of the block = the number of celss across the block

	TriMesh::CMeshPtr _pModelTriMesh;
	Vector3d _corners[8];
	FaceRayHits _rayTriHits[3];

	ObjectPoolWMutex<Vertex> _vertices;
	ObjectPoolWMutex<Polygon> _polygons;
	ObjectPool<Polyhedron> _polyhedra;
	ObjectPool<SubBlock> _subBlocks;
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

inline size_t Block::numSubBlocks() const
{
	return _subBlocks.size();
}

inline size_t Block::calLinearSubBlockIndex(const Index3D& subBlockIdx) const
{
	if (subBlockIdx.isInBounds(_blockDim))
		return subBlockIdx[0] + _blockDim * (subBlockIdx[1] + _blockDim * subBlockIdx[2]);
	return -1;
}

inline SubBlock& Block::getSubBlock(const Index3D& subBlockIdx)
{
	size_t idx = calLinearSubBlockIndex(subBlockIdx);
	if (idx < _subBlocks.size()) {
		if (_subBlocks.exists(idx)) {
			auto& result = _subBlocks[idx];
			return result;
		}
	}
	throw std::runtime_error("Block::getSubBlock out of range");
}

inline const SubBlock& Block::getSubBlock(const Index3D& subBlockIdx) const
{
	size_t idx = calLinearSubBlockIndex(subBlockIdx);
	if (idx < _subBlocks.size()) {
		if (_subBlocks.exists(idx)) {
			auto& result = _subBlocks[idx];
			return result;
		}
	}
	throw std::runtime_error("Block::getSubBlock out of range");
}

inline void Block::freeSubBlock(const Index3D& subBlockIdx)
{
	size_t idx = calLinearSubBlockIndex(subBlockIdx);
	if (_subBlocks.exists(idx)) {
		_subBlocks.free(idx);
	}
}

inline bool Block::isUnloaded() const
{
	return !_filename.empty();
}

inline const TriMesh::CMeshPtr& Block::getModelMesh() const
{
	return _pModelTriMesh;
}

inline std::mutex& Block::getVertexMutex() const
{
	return _vertices.getMutex();
}

inline std::mutex& Block::getFaceMutex() const
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
void Block::facesFunc(LAMBDA func) const
{
	std::lock_guard g(_polygons);
	func(_polygons);
}

template<class LAMBDA>
void Block::facesFunc(LAMBDA func)
{
	std::lock_guard g(_polygons);
	func(_polygons);
}

}
