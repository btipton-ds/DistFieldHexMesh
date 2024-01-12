#pragma once

#include <stdexcept>
#include <triMesh.h>
#include <objectPool.h>
#include <Index3D.h>
#include <cell.h>
#include <vertex.h>
#include <mutex>
#include <tm_vector3.h>

namespace DFHM {

class Volume;
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
	enum Side {
		None = 0,
		Left = 1,
		Front = 2,
		Bottom = 4,
		Right = 8,
		Back = 16,
		Top = 32,
	};

	static void setMinBlockDim(size_t dim);
	static size_t getMinBlockDim();

	Block(std::vector<Vector3d>& pts);
	Block(const Block& src);
	~Block(); // NOT virtual - do not inherit

	size_t blockDim() const;
	size_t getHash() const;
	bool operator < (const Block& rhs) const;

	void connectAdjacent(Volume& vol, const Index3D& idx);
	void clearAdjacent();
	void processBlock(size_t blockRayIdx, std::vector<bool>& cellsToCreate);
	bool scanCreateCellsWhereNeeded(std::vector<bool>& blocksToCreate, const Index3D& axisOrder);
	void createCells();
	size_t calLinearCellIndex(size_t ix, size_t iy, size_t iz) const;
	size_t calLinearCellIndex(const Index3D& cellIdx) const;
	Index3D calCellIndexFromLinear(size_t linearIdx) const;
	void addCellFaces();
	void createBlockFaces();

	size_t numCells() const;
	bool cellExists(size_t ix, size_t iy, size_t iz) const;
	bool cellExists(const Index3D& idx) const;
	Cell& getCell(size_t ix, size_t iy, size_t iz);
	Cell& getCell(const Index3D& idx);
	const Cell& getCell(size_t ix, size_t iy, size_t iz) const;
	const Cell& getCell(const Index3D& idx) const;
	void freeCell(size_t ix, size_t iy, size_t iz);
	void freeCell(const Index3D& idx);

	std::vector<uint32_t> getCellDivs() const;

	size_t numFaces(bool includeInner) const;
	size_t numPolyhedra() const;

	void fillEmpty();
	void processTris(const TriMesh::CMeshPtr& pSrcMesh, const std::vector<size_t>& triIndices);
	void processTris();
	void addTris(const TriMesh::CMeshPtr& pSrcMesh, const std::vector<size_t>& triIndices);
	const TriMesh::CMeshPtr& getModelMesh() const;
	TriMesh::CMeshPtr getBlockTriMesh(bool outerOnly) const;

	// pack removes the cell array if there's nothing interesting in it. It's a full search of the array and can be time consuming.
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

	struct RayTriHit {
		size_t _cellIdx;
		size_t _triIdx;
		Vector3d _relPt;
	};

	struct CrossBlockPoint {
		inline operator const Vector3d& () const
		{
			return _pt;
		}

		Vector3d _pt;
		uint8_t _lockMask = 0;
	};

	std::vector<Vector3d> getCornerPts() const; // Change to returning fractions so we can assign boundary values.
	std::vector<CrossBlockPoint> getCellCornerPts(const Index3D& cellIdx) const;
	std::vector<LineSegment> getCellEdges(const Index3D& cellIdx) const;

	uint8_t lockMaskOfCellIndex(size_t cellIdx) const;

	Block* getOwner(const Vector3i& blockIdx);
	const Block* getOwner(const Vector3i& blockIdx) const;

	// This gets the owner block from the lockMask of a vertex inclusively within this block
	Block* getOwner(uint8_t lockMask);
	const Block* getOwner(uint8_t lockMask) const;

	void rayCastFace(const std::vector<Vector3d>& pts, size_t samples, int axis, std::vector<RayTriHit>& rayTriHits) const;
	void setNumDivs();
	void subDivideCellIfNeeded(const LineSegment& seg, const std::vector<RayHit>& hits, const Index3D& cellIdx);
	CrossBlockPoint triLinInterp(const std::vector<Vector3d>& blockPts, const Index3D& pt) const;

	size_t addHexCell(const std::vector<CrossBlockPoint>& pts);
	size_t addQuadFace(const std::vector<CrossBlockPoint>& pts);
	void calBlockOriginSpan(Vector3d& origin, Vector3d& span) const;

	std::string _filename;

	static size_t s_minBlockDim;
	Index3D _blockIdx;
	size_t _blockDim;

	TriMesh::CMeshPtr _pModelTriMesh;
	Vertex::FixedPt _corners[8];
	std::vector<uint32_t> _cellDivs;
	std::vector<RayTriHit> _rayTriHits;

	ObjectPoolWMutex<Vertex> _vertices;
	ObjectPool<Polygon> _polygons;
	ObjectPool<Polyhedron> _polyhedra;
	ObjectPool<Cell> _cells;

	std::shared_ptr<Block> _pAdj[8]; // A 2x2x2 array of adjacent block pointers for the positive octant which may be null. This is at [1,1,1] and set to null
};

inline size_t Block::blockDim() const
{
	return _blockDim;
}

inline size_t Block::numCells() const
{
	return _cells.size();
}

inline size_t Block::calLinearCellIndex(size_t ix, size_t iy, size_t iz) const
{
	if (ix < _blockDim && iy < _blockDim && iz < _blockDim)
		return ix + _blockDim * (iy + _blockDim * iz);
	return -1;
}

inline size_t Block::calLinearCellIndex(const Index3D& celIdx) const
{
	return calLinearCellIndex(celIdx[0], celIdx[1], celIdx[2]);
}

inline Cell& Block::getCell(size_t ix, size_t iy, size_t iz)
{
	return getCell(Index3D(ix, iy, iz));
}

inline Cell& Block::getCell(const Index3D& idx3)
{
	size_t idx = calLinearCellIndex(idx3);
	if (idx < _cells.size()) {
		if (_cells.exists(idx)) {
			auto& result = _cells[idx];
			return result;
		}
	}
	throw std::runtime_error("Block::getCell out of range");
}

inline const Cell& Block::getCell(size_t ix, size_t iy, size_t iz) const
{
	size_t idx = calLinearCellIndex(ix, iy, iz);
	if (idx < _cells.size()) {
		if (_cells.exists(idx)) {
			auto& result = _cells[idx];
			return result;
		}
	}
	throw std::runtime_error("Block::getCell out of range");
}

inline const Cell& Block::getCell(const Index3D& idx) const
{
	return getCell(idx[0], idx[1], idx[2]);
}

inline void Block::freeCell(size_t ix, size_t iy, size_t iz)
{
	size_t idx = calLinearCellIndex(ix, iy, iz);
	if (_cells.exists(idx)) {
		_cells.free(idx);
	}
}

inline void Block::freeCell(const Index3D& idx)
{
	freeCell(idx[0], idx[1], idx[2]);
}

inline bool Block::isUnloaded() const
{
	return !_filename.empty();
}

inline std::vector<uint32_t> Block::getCellDivs() const
{
	return _cellDivs;
}

inline const TriMesh::CMeshPtr& Block::getModelMesh() const
{
	return _pModelTriMesh;
}

}
