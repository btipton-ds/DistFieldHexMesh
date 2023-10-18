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

	void processBlock(const TriMesh::CMeshPtr& pTriMesh, size_t blockRayIdx, const Vector3d& blockOrigin, const Vector3d& blockSpan, std::vector<bool>& cellsToCreate);
	bool scanCreateCellsWhereNeeded(const TriMesh::CMeshPtr& pTriMesh, const Vector3d& origin, const Vector3d& blockSpan, std::vector<bool>& blocksToCreate, const Vector3i& axisOrder);
	void createCells(const std::vector<bool>& cellsToCreate);
	static size_t calcCellIndex(size_t ix, size_t iy, size_t iz);
	static size_t calcCellIndex(const Vector3i& celIdx);
	void addBlockTris(const Vector3d& blockOrigin, const Vector3d& blockSpan, TriMesh::CMeshPtr& pMesh, bool useCells);

	Cell* getCell(size_t ix, size_t iy, size_t iz);
	Cell* getCell(const Vector3i& idx);
	const Cell* getCell(size_t ix, size_t iy, size_t iz) const;
	const Cell* getCell(const Vector3i& idx) const;

	bool isUnloaded() const;
	bool unload(std::string& filename);
	bool load();

protected:
	enum class AxisIndex {
		X, Y, Z
	};
	static Vector3i getAxisOrder(AxisIndex axisIdx);

private:
	friend class Volume;


	struct RayTriIntersect {
		double _w = -1;
		size_t _triIdx = -1, _blockIdx = -1, _cellIdx = -1;
	};

	using RayTriIntersectVec = std::vector<RayTriIntersect>;
	using RayBlockIntersectVec = std::vector<RayTriIntersectVec>;

	std::string _filename;

	static size_t s_blockDim;
	std::vector<size_t> _cells;
};

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
