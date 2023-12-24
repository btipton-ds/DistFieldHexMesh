#pragma once

#include <triMesh.h>
#include <dataPool.h>
#include <cell.h>

namespace DFHM {

class Volume;

class Block : public DataPool {
public:
	static void setBlockDim(size_t dim);
	static size_t getBlockDim();

	Block();
	Block(const Block& src);
	~Block(); // NOT virtual - do not inherit

	void processBlock(const TriMesh::CMeshPtr& pTriMesh, size_t blockRayIdx, const Vector3d& blockOrigin, const Vector3d& blockSpan, std::vector<bool>& cellsToCreate);
	bool scanCreateCellsWhereNeeded(const TriMesh::CMeshPtr& pTriMesh, const Vector3d& origin, const Vector3d& blockSpan, std::vector<bool>& blocksToCreate, const Vector3i& axisOrder);
	void addCell(size_t ix, size_t iy, size_t iz, size_t threadNum = 0);
	void addCell(const Vector3i& cellIdx, size_t threadNum = 0);
	void createCells(const std::vector<bool>& cellsToCreate, size_t threadNum = 0);
	static size_t calcCellIndex(size_t ix, size_t iy, size_t iz);
	static size_t calcCellIndex(const Vector3i& cellIdx);
	void addBlockTris(const ObjectPoolId& blockId, const Vector3d& blockOrigin, const Vector3d& blockSpan, TriMesh::CMeshPtr& pMesh, bool useCells) const;

	size_t numCells() const;
	bool cellExists(size_t ix, size_t iy, size_t iz) const;
	bool cellExists(const Vector3i& idx) const;
	Cell& getCell(size_t ix, size_t iy, size_t iz);
	Cell& getCell(const Vector3i& idx);
	const Cell& getCell(size_t ix, size_t iy, size_t iz) const;
	const Cell& getCell(const Vector3i& idx) const;
	void freeCell(size_t ix, size_t iy, size_t iz);
	void freeCell(const Vector3i& idx);

	void fillEmpty();

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

	using RayTriIntersectVec = std::vector<RayTriIntersect>;
	using RayBlockIntersectVec = std::vector<RayTriIntersectVec>;

	void addRectPrismFaces(const ObjectPoolId& blockId, const std::vector<Vector3d>& pts) const;
	void addQuadFace(const ObjectPoolId& blockId, const std::vector<Vector3d>& pts) const;

	std::string _filename;

	static size_t s_blockDim;
	std::vector<ObjectPoolId> _cells;
};

inline size_t Block::numCells() const
{
	return _cells.size();
}

inline size_t Block::calcCellIndex(size_t ix, size_t iy, size_t iz)
{
	if (ix < s_blockDim && iy < s_blockDim && iz < s_blockDim)
		return ix + s_blockDim * (iy + s_blockDim * iz);
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
	if (idx < _cells.size()) {
		auto& result = _cellPool[_cells[idx]];
		return result;
	}
	throw std::exception("Block::getCell out of range");
}

inline const Cell& Block::getCell(size_t ix, size_t iy, size_t iz) const
{
	size_t idx = calcCellIndex(ix, iy, iz);
	if (idx < _cells.size()) {
		auto& result = _cellPool[_cells[idx]];
		return result;
	}
	throw std::exception("Block::getCell out of range");
}

inline const Cell& Block::getCell(const Vector3i& idx) const
{
	return getCell(idx[0], idx[1], idx[2]);
}

inline void Block::freeCell(size_t ix, size_t iy, size_t iz)
{
	size_t idx = calcCellIndex(ix, iy, iz);
	if (idx < _cells.size() && _cells[idx] != -1) {
		_cellPool.free(_cells[idx]);
		_cells[idx] = -1;
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

}
