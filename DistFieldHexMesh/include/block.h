#pragma once

#include <triMesh.h>
#include <objectPool.h>

namespace DFHM {

class Block : public DataPool {
public:
	static void setBlockDim(size_t dim);
	static size_t getBlockDim();

	Block();
	Block(const Block& src);

	bool scanCreateCellsWhereNeeded(const TriMesh::CMeshPtr& pTriMesh, const Vector3d& origin, const Vector3d& blockSpan, std::vector<bool>& blocksToCreate, const Vector3i& axisOrder);
	void createCells(const std::vector<bool>& cellsToCreate);
	size_t calcCellIndex(size_t ix, size_t iy, size_t iz) const;
	size_t calcCellIndex(const Vector3i& celIdx) const;
	void addBlockTris(const Vector3d& blockOrigin, const Vector3d& blockSpan, TriMesh::CMeshPtr& pMesh, bool useCells);

	Cell* getCell(size_t ix, size_t iy, size_t iz);
	Cell* getCell(const Vector3i& idx);
	const Cell* getCell(size_t ix, size_t iy, size_t iz) const;
	const Cell* getCell(const Vector3i& idx) const;

	bool isUnloaded() const;
	bool unload(std::string& filename);
	bool load();

private:
	std::string _filename;

	static size_t s_blockDim;
	std::vector<size_t> _cells;
};

inline size_t Block::calcCellIndex(size_t ix, size_t iy, size_t iz) const
{
	if (ix < s_blockDim && iy < s_blockDim && iz < s_blockDim)
		return ix + s_blockDim * (iy + s_blockDim * iz);
	return -1;
}

inline size_t Block::calcCellIndex(const Vector3i& celIdx) const
{
	return calcCellIndex(celIdx[0], celIdx[1], celIdx[2]);
}

inline Cell* Block::getCell(size_t ix, size_t iy, size_t iz)
{
	size_t idx = calcCellIndex(ix, iy, iz);
	if (idx < _cells.size())
		return _cellPool.getObj(_cells[idx]);

	return nullptr;
}

inline Cell* Block::getCell(const Vector3i& idx)
{
	return getCell(idx[0], idx[1], idx[2]);
}

inline const Cell* Block::getCell(size_t ix, size_t iy, size_t iz) const
{
	size_t idx = calcCellIndex(ix, iy, iz);
	if (idx < _cells.size())
		return _cellPool.getObj(_cells[idx]);

	return nullptr;
}

inline const Cell* Block::getCell(const Vector3i& idx) const
{
	return getCell(idx[0], idx[1], idx[2]);
}

inline bool Block::isUnloaded() const
{
	return !_filename.empty();
}

}
