#pragma once

#include <stdexcept>
#include <triMesh.h>
#include <objectPool.h>
#include <indices.h>
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

	std::vector<Vector3d> getCornerPts() const; // Change to returning fractions so we can assign boundary values.
	std::vector<Vector3d> getCellCornerPts(const Index3D& cellIdx) const;
	std::vector<LineSegment> getCellEdges(const Index3D& cellIdx) const;

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

	void rayCastFace(const std::vector<Vector3d>& pts, size_t samples, int axis, std::vector<RayTriHit>& rayTriHits) const;
	void setNumDivs();
	void subDivideCellIfNeeded(const LineSegment& seg, const std::vector<RayHit>& hits, const Index3D& cellIdx);
	Vector3d triLinInterp(const std::vector<Vector3d>& pts, const Index3D& pt) const;

	size_t addHexCell(const std::vector<Vector3d>& pts);
	size_t addQuadFace(const std::vector<Vector3d>& pts);
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

	std::shared_ptr<Block> _adjBack, _adjTop, _adjRight;
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
