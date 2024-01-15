#pragma once

#include <stdexcept>
#include <triMesh.h>
#include <objectPoolWMutex.h>
#include <UniversalIndex3D.h>
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
	static void setMinBlockDim(size_t dim);
	static size_t getMinBlockDim();

	Block(Volume* pVol, const Index3D& blockIdx, std::vector<Vector3d>& pts);
	Block(const Block& src) = delete;
	~Block(); // NOT virtual - do not inherit

	size_t blockDim() const;
	Volume* getVolume();
	const Volume* getVolume() const;

	void createSubBlocks();
	size_t calLinearSubBlockIndex(size_t ix, size_t iy, size_t iz) const;
	size_t calLinearSubBlockIndex(const Index3D& subBlockIdx) const;
	Index3D calSubBlockIndexFromLinear(size_t linearIdx) const;
	void addSubBlockFaces();
	void createBlockFaces();

	size_t numSubBlocks() const;
	bool subBlockExists(size_t ix, size_t iy, size_t iz) const;
	bool subBlockExists(const Index3D& idx) const;
	SubBlock& getSubBlock(size_t ix, size_t iy, size_t iz);
	SubBlock& getSubBlock(const Index3D& idx);
	const SubBlock& getSubBlock(size_t ix, size_t iy, size_t iz) const;
	const SubBlock& getSubBlock(const Index3D& idx) const;
	void freeSubBlock(size_t ix, size_t iy, size_t iz);
	void freeSubBlock(const Index3D& idx);

	size_t numFaces(bool includeInner) const;
	size_t numPolyhedra() const;

	inline const Index3D& getBlockIdx()
	{
		return _blockIdx;
	}

	void processTris(const TriMesh::CMeshPtr& pSrcMesh, const std::vector<size_t>& triIndices);
	void processTris();
	void addTris(const TriMesh::CMeshPtr& pSrcMesh, const std::vector<size_t>& triIndices);
	const TriMesh::CMeshPtr& getModelMesh() const;
	size_t getGLModelEdgeLoops(std::vector<std::vector<float>>& edgeLoops) const;
	TriMesh::CMeshPtr getBlockTriMesh(bool outerOnly) const;

	// pack removes the subBlock array if there's nothing interesting in it. It's a full search of the array and can be time consuming.
	void pack();
	bool isUnloaded() const;
	bool unload(std::string& filename);
	bool load();

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

	const Vector3d* getCornerPts() const; // Change to returning fractions so we can assign boundary values.
	std::vector<CrossBlockPoint> getSubBlockCornerPts(const Vector3d* blockPts, size_t blockDim, const Index3D& subBlockIdx) const;
	std::vector<LineSegment> getSubBlockEdges(const Vector3d* blockPts, const Index3D& subBlockIdx) const;

	Block* getOwner(const Index3D& blockIdx);
	const Block* getOwner(const Index3D& blockIdx) const;

	void findFeatures();
	void findSharpVertices(const CBoundingBox3Dd& bbox);
	void findSharpEdgeGroups(const CBoundingBox3Dd& bbox);

	size_t rayCastFace(const Vector3d* pts, size_t samples, int axis, FaceRayHits& rayTriHits) const;
	void rayCastHexBlock(const Vector3d* pts, size_t blockDim, FaceRayHits _rayTriHits[3]);
	CrossBlockPoint triLinInterp(const Vector3d* blockPts, size_t blockDim, const Index3D& pt) const;
	static Vector3d invTriLinIterp(const Vector3d* blockPts, const Vector3d& pt);
	void createSubBlocksForHexSubBlock(const Vector3d* blockPts, const Index3D& subBlockIdx);
	void addSubBlockIndicesForMeshVerts(std::set<Index3D>& subBlockIndices);
	void addSubBlockIndicesForRayHits(std::set<Index3D>& subBlockIndices);
	void splitAtSharpVertices(const Vector3d* blockPts, const Index3D& subBlockIdx);
	void splitAtLegIntersections(const Vector3d* blockPts, const Index3D& subBlockIdx);

	void addHitsForRay(size_t axis, size_t i, size_t j, size_t ii, size_t jj, std::set<Index3D>& subBlockIndices);
	static void addIndexToMap(const Index3D& subBlockIdx, std::set<Index3D>& subBlockIndices);
	size_t addHexCell(const Vector3d* blockPts, size_t blockDim, const Index3D& subBlockIdx);
	UniversalIndex3D addFace(const std::vector<CrossBlockPoint>& pts);
	UniversalIndex3D addFace(int axis, const Index3D& subBlockIdx, const std::vector<CrossBlockPoint>& pts);
	void divideSubBlock(const Index3D& subBlockIdx, size_t divs);
	void calBlockOriginSpan(Vector3d& origin, Vector3d& span) const;

	UniversalIndex3D addVertex(const CrossBlockPoint& pt, size_t currentId = -1);

	std::string _filename;

	Volume* _pVol;
	static size_t s_minBlockDim;
	Index3D _blockIdx;
	size_t _blockDim; // This the dimension of the block = the number of celss across the block

	TriMesh::CMeshPtr _pModelTriMesh;
	Vector3d _corners[8];
	FaceRayHits _rayTriHits[3];

	std::vector<size_t> _sharpVertIndices, _sharpEdgeIndices;

	ObjectPoolWMutex<Vertex> _vertices;
	ObjectPoolWMutex<Polygon> _polygons;
	ObjectPoolWMutex<Edge> _edges;
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

inline size_t Block::calLinearSubBlockIndex(size_t ix, size_t iy, size_t iz) const
{
	if (ix < _blockDim && iy < _blockDim && iz < _blockDim)
		return ix + _blockDim * (iy + _blockDim * iz);
	return -1;
}

inline size_t Block::calLinearSubBlockIndex(const Index3D& celIdx) const
{
	return calLinearSubBlockIndex(celIdx[0], celIdx[1], celIdx[2]);
}

inline SubBlock& Block::getSubBlock(size_t ix, size_t iy, size_t iz)
{
	return getSubBlock(Index3D(ix, iy, iz));
}

inline SubBlock& Block::getSubBlock(const Index3D& idx3)
{
	size_t idx = calLinearSubBlockIndex(idx3);
	if (idx < _subBlocks.size()) {
		if (_subBlocks.exists(idx)) {
			auto& result = _subBlocks[idx];
			return result;
		}
	}
	throw std::runtime_error("Block::getSubBlock out of range");
}

inline const SubBlock& Block::getSubBlock(size_t ix, size_t iy, size_t iz) const
{
	size_t idx = calLinearSubBlockIndex(ix, iy, iz);
	if (idx < _subBlocks.size()) {
		if (_subBlocks.exists(idx)) {
			auto& result = _subBlocks[idx];
			return result;
		}
	}
	throw std::runtime_error("Block::getSubBlock out of range");
}

inline const SubBlock& Block::getSubBlock(const Index3D& idx) const
{
	return getSubBlock(idx[0], idx[1], idx[2]);
}

inline void Block::freeSubBlock(size_t ix, size_t iy, size_t iz)
{
	size_t idx = calLinearSubBlockIndex(ix, iy, iz);
	if (_subBlocks.exists(idx)) {
		_subBlocks.free(idx);
	}
}

inline void Block::freeSubBlock(const Index3D& idx)
{
	freeSubBlock(idx[0], idx[1], idx[2]);
}

inline bool Block::isUnloaded() const
{
	return !_filename.empty();
}

inline const TriMesh::CMeshPtr& Block::getModelMesh() const
{
	return _pModelTriMesh;
}


}
