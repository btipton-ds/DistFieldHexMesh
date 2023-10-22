#pragma once

#include <triMesh.h>
#include <objectPool.h>
#include <types.h>

namespace DFHM {

class Block : public DataPool {
public:
	static void setBlockDim(size_t dim);
	static size_t getBlockDim();

	Block();
	Block(const Block& src);
	~Block(); // NOT virtual - do not inherit

	bool scanCreateCellsWhereNeeded(const TriMesh::CMeshPtr& pTriMesh, const Vector3d& origin, const Vector3d& blockSpan, std::vector<bool>& blocksToCreate, const Vector3i& axisOrder);
	void createCells(const Vector3d& blockOrigin, const Vector3d& blockSpan, const Vector3i& blockDim, const Vector3i& blockIdx, const std::vector<bool>& cellsToCreate, const RayHitRec& hits);
	void createIntersectionCells(Volume& vol, const Vector3d& blockOrigin, const Vector3d& blockSpan);
	static size_t calcLinearCellIndex(size_t ix, size_t iy, size_t iz);
	static size_t calcLinearCellIndex(const Vector3i& celIdx);
	static Vector3i calcCartesianCellIndex(size_t cIdx);
	static Vector3d calcCellOrigin(const Vector3i& cellIndex, const Vector3d& blockOrigin, const Vector3d& blockSpan);
	void addBlockTris(const Vector3d& blockOrigin, const Vector3d& blockSpan, TriMesh::CMeshPtr& pMesh, bool useCells);

	size_t numCells() const;

	size_t createCell(Cell*& pCell);
	Cell* getCell(size_t ix, size_t iy, size_t iz, bool create = false);
	Cell* getCell(const Vector3i& idx, bool create = false);
	const Cell* getCell(size_t ix, size_t iy, size_t iz) const;
	const Cell* getCell(const Vector3i& idx) const;
	void freeCell(size_t ix, size_t iy, size_t iz);
	void freeCell(const Vector3i& idx);

	void fillEmpty();

	// pack removes the cell array if there's nothing interesting in it. It's a full search of the array and can be time consuming.
	void pack();
	bool isUnloaded() const;
	bool unload(std::string& filename);
	bool load();

private:
	friend class TestBlock;
	void addCellHitsX(const Vector3d& blockOrigin, const Vector3d& cellSpan, const Vector3i& blockDim, const Vector3i& blockIdx, const RayBlockIntersectVec& xHits);
	void addCellHitsY(const Vector3d& blockOrigin, const Vector3d& cellSpan, const Vector3i& blockDim, const Vector3i& blockIdx, const RayBlockIntersectVec& yHits);
	void addCellHitsZ(const Vector3d& blockOrigin, const Vector3d& cellSpan, const Vector3i& blockDim, const Vector3i& blockIdx, const RayBlockIntersectVec& zHits);

	std::string _filename;

	static size_t s_blockDim;
	std::vector<Cell> _cellData;
	std::vector<size_t> _cells, _freeCells;
};

inline size_t Block::numCells() const
{
	return _cells.size();
}

inline size_t Block::calcLinearCellIndex(size_t ix, size_t iy, size_t iz)
{
	if (ix < s_blockDim && iy < s_blockDim && iz < s_blockDim)
		return ix + s_blockDim * (iy + s_blockDim * iz);
	return -1;
}

inline size_t Block::calcLinearCellIndex(const Vector3i& celIdx)
{
	return calcLinearCellIndex(celIdx[0], celIdx[1], celIdx[2]);
}

inline Vector3i Block::calcCartesianCellIndex(size_t cIdx)
{
	Vector3i cellIdx;
	size_t temp;

	cellIdx[0] = cIdx % s_blockDim;
	temp = cIdx / s_blockDim;

	cellIdx[1] = temp % s_blockDim;
	temp = temp / s_blockDim;

	cellIdx[2] = temp % s_blockDim;

	// Back check the math. TODO remove later
	assert(calcLinearCellIndex(cellIdx) == cIdx);

	return cellIdx;

}

inline Vector3d Block::calcCellOrigin(const Vector3i& cellIndex, const Vector3d& blockOrigin, const Vector3d& blockSpan)
{
	Vector3d result;
	for (int i = 0; i < 3; i++)
		result[i] = blockOrigin[i] + cellIndex[i] * blockSpan[i];
	return result;
}

inline Cell* Block::getCell(const Vector3i& idx, bool create)
{
	return getCell(idx[0], idx[1], idx[2], create);
}

inline const Cell* Block::getCell(size_t ix, size_t iy, size_t iz) const
{
	size_t idx = calcLinearCellIndex(ix, iy, iz);
	if (idx < _cells.size())
		return  &_cellData[_cells[idx]];

	return nullptr;
}

inline const Cell* Block::getCell(const Vector3i& idx) const
{
	return getCell(idx[0], idx[1], idx[2]);
}

inline void Block::freeCell(size_t ix, size_t iy, size_t iz)
{
	size_t idx = calcLinearCellIndex(ix, iy, iz);
	if (idx < _cells.size() && _cells[idx] != -1) {
		_freeCells.push_back(_cells[idx]);
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
