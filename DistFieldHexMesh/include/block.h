#pragma once

#include <triMesh.h>
#include <dataPool.h>
#include <cell.h>
#include <vertex.h>
#include <tm_vector3.h>
#include <stdexcept>

namespace DFHM {

class Volume;

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

	void processBlock(size_t blockRayIdx, std::vector<bool>& cellsToCreate);
	bool scanCreateCellsWhereNeeded(std::vector<bool>& blocksToCreate, const Vector3i& axisOrder);
	void addCell(size_t ix, size_t iy, size_t iz);
	void addCell(const Vector3i& cellIdx);
	void createCells(const std::vector<bool>& cellsToCreate);
	static size_t calcCellIndex(size_t ix, size_t iy, size_t iz);
	static size_t calcCellIndex(const Vector3i& cellIdx);
	void addBlockFaces(size_t blockId, bool makeCells);
	void createBlockFaces();

	std::vector<Vector3d> getCornerPts() const;
	std::vector<Vector3d> getCellCornerPts(const Vector3i& cellIdx) const;
	std::vector<LineSegment> getCellEdges(const Vector3i& cellIdx) const;

	size_t numCells() const;
	bool cellExists(size_t ix, size_t iy, size_t iz) const;
	bool cellExists(const Vector3i& idx) const;
	Cell& getCell(size_t ix, size_t iy, size_t iz);
	Cell& getCell(const Vector3i& idx);
	const Cell& getCell(size_t ix, size_t iy, size_t iz) const;
	const Cell& getCell(const Vector3i& idx) const;
	void freeCell(size_t ix, size_t iy, size_t iz);
	void freeCell(const Vector3i& idx);

	std::vector<uint32_t> getCellDivs() const;

	void fillEmpty();
	void processTris(const TriMesh::CMeshPtr& pSrcMesh, const std::vector<size_t>& triIndices);

	// pack removes the cell array if there's nothing interesting in it. It's a full search of the array and can be time consuming.
	void pack();
	bool isUnloaded() const;
	bool unload(std::string& filename);
	bool load();

protected:
	enum class AxisIndex {
		X, Y, Z
	};

private:
	friend class Volume;
	friend class TestBlock;

	struct RayTriIntersect {
		double _w = -1;
		size_t _triIdx = -1, _blockIdx = -1, _cellIdx = -1;
	};

	struct CellLegIntersect {
		Vector3i _cellIdx;
		size_t _edgeNumber;
		std::vector<RayHit> hits;
	};

	using RayTriIntersectVec = std::vector<RayTriIntersect>;
	using RayBlockIntersectVec = std::vector<RayTriIntersectVec>;

	void addTris(const TriMesh::CMeshPtr& pSrcMesh, const std::vector<size_t>& triIndices);
	void setNumDivs();
	void subDivideCellIfNeeded(const LineSegment& seg, const std::vector<RayHit>& hits, const Vector3i& cellIdx);
	Vector3d triLinInterp(const std::vector<Vector3d>& pts, const Vector3i& pt) const;

	void addRectPrismFaces(size_t blockId, const std::vector<Vector3d>& pts);
	void addQuadFace(size_t blockId, const std::vector<Vector3d>& pts);
	void calBlockOriginSpan(Vector3d& origin, Vector3d& span) const;

	std::string _filename;

	static size_t s_minBlockDim;
	size_t _blockDim;

	TriMesh::CMeshPtr _pModelTriMesh;
	Vertex::FixedPt _corners[8];
	std::vector<uint32_t> _cellDivs;
	std::vector<CellLegIntersect> _cellLegIntersections;

	ObjectPool<Vertex> _vertexPool;
	ObjectPool<Polygon> _polygonPool;
	ObjectPool<Polyhedron> _polyhedronPool;
	ObjectPool<Cell> _cellPool;
};

inline size_t Block::blockDim() const
{
	return _blockDim;
}

inline size_t Block::numCells() const
{
	return _cellPool.size();
}

inline size_t Block::calcCellIndex(size_t ix, size_t iy, size_t iz)
{
	if (ix < s_minBlockDim && iy < s_minBlockDim && iz < s_minBlockDim)
		return ix + s_minBlockDim * (iy + s_minBlockDim * iz);
	return -1;
}

inline size_t Block::calcCellIndex(const Vector3i& celIdx)
{
	return calcCellIndex(celIdx[0], celIdx[1], celIdx[2]);
}

inline Cell& Block::getCell(size_t ix, size_t iy, size_t iz)
{
	return getCell(Vector3i(ix, iy, iz));
}

inline Cell& Block::getCell(const Vector3i& idx3)
{
	size_t idx = calcCellIndex(idx3);
	if (idx < _cellPool.size()) {
		if (_cellPool.exists(idx)) {
			auto& result = _cellPool[idx];
			return result;
		}
	}
	throw std::runtime_error("Block::getCell out of range");
}

inline const Cell& Block::getCell(size_t ix, size_t iy, size_t iz) const
{
	size_t idx = calcCellIndex(ix, iy, iz);
	if (idx < _cellPool.size()) {
		if (_cellPool.exists(idx)) {
			auto& result = _cellPool[idx];
			return result;
		}
	}
	throw std::runtime_error("Block::getCell out of range");
}

inline const Cell& Block::getCell(const Vector3i& idx) const
{
	return getCell(idx[0], idx[1], idx[2]);
}

inline void Block::freeCell(size_t ix, size_t iy, size_t iz)
{
	size_t idx = calcCellIndex(ix, iy, iz);
	if (_cellPool.exists(idx)) {
		_cellPool.free(idx);
	}
}

inline void Block::freeCell(const Vector3i& idx)
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

}
