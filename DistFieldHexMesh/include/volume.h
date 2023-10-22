#pragma once

/*
* DistFieldHexMesh
* 
* cell.h
* 
*/

#include <stdint.h>
#include <vector>
#include <map>
#include <mutex>
#include "indices.h"
#include <tm_spatialSearch.h>
#include <triMesh.h>
#include <ObjectPool.h>
#include <polygon.h>
#include <polyhedron.h>
#include <types.h>
#include <cell.h>
#include <block.h>

namespace TriMesh {
	class CMesh;
	using CMeshPtr = std::shared_ptr<CMesh>;
}

namespace DFHM {

using CMesh = TriMesh::CMesh;


class Volume : public DataPool {
public:
	Volume(const Index3& size = Index3(0, 0, 0));
	Volume(const Volume& src);

	void setBlockDims(const Index3& size);
	const Index3& getBlockDims() const;
	const Index3& getDims() const;

	// Get the block using a block index
	const Block* getBlock(size_t ix, size_t iy, size_t iz) const;
	const Block* getBlock(const Vector3i& blockIdx) const;
	Block* getBlock(size_t ix, size_t iy, size_t iz, bool create = false);
	Block* getBlock(const Vector3i& blockIdx, bool create = false);

	// Get the cell using a cell index
	Cell* getCell(size_t ix, size_t iy, size_t iz);
	Cell* getCell(const Vector3i& cellIdx);
	const Cell* getCell(size_t ix, size_t iy, size_t iz) const;
	const Cell* getCell(const Vector3i& cellIdx) const;

	size_t getVertIdx(const Vector3d& pt) const;
	size_t getVertIdx(const Vertex& vert) const;
	size_t addVert(const Vertex& vert);

	// Currently flow direction is along positive x axis.
	size_t calLinearBlockIndex(size_t ix, size_t iy, size_t iz) const;
	size_t calLinearBlockIndex(const Vector3i& blockIdx) const;
	Vector3i calCartesianBlockIndex(size_t idx) const;
	void buildCFDHexes(const TriMesh::CMeshPtr& pTriMesh, double minCellSize, const Vector3d& emptyVolRatio = Vector3d(10, 3, 3));
	TriMesh::CMeshPtr makeBlockTris(bool cells = true);
	TriMesh::CMeshPtr makeIntersectionTris(bool cells = true);
	void dumpSections(const std::string& dirName) const;

private:
	void createBlockRays(AxisIndex axisIdx, const TriMesh::CMeshPtr& pTriMesh, std::vector<bool>& blocksToCreate);
	void createBlockCellRays(AxisIndex axisIdx, const TriMesh::CMeshPtr& pTriMesh, const std::vector<bool>& blocksToCreate, RayHitRec& intersections, std::vector<std::vector<bool>>& cellsToCreate);
	void processRayHit(const RayHit& triHit, AxisIndex rayAxis, const Vector3d& blockSpan, const Vector3d& cellSpan, size_t& blockIdx, size_t& cellIdx);
	void processRayHit(const RayHit& triHit, AxisIndex rayAxis, size_t i, size_t j, const Vector3d& blockSpan, const Vector3d& cellSpan, RayTriIntersectVec& blockHits, size_t& blockIdx, size_t& cellIdx);

	mutable std::mutex _mutex;
	Vector3d _originMeters, _spanMeters;
	Index3 _blockDim;

	std::vector<size_t> _blocks;
	std::shared_ptr<CSpatialSearchSTd> _pVertFinder;
};

using VolumePtr = std::shared_ptr<Volume>;

inline const Block* Volume::getBlock(size_t ix, size_t iy, size_t iz) const
{
	std::lock_guard<std::mutex> lock(_mutex);

	size_t idx = calLinearBlockIndex(ix, iy, iz);
	if (idx < _blocks.size())
		return _blockPool.getObj(_blocks[idx]);
	return nullptr;
}

inline Block* Volume::getBlock(size_t ix, size_t iy, size_t iz, bool create)
{
	std::lock_guard<std::mutex> lock(_mutex);

	size_t idx = calLinearBlockIndex(ix, iy, iz);
	if (idx < _blocks.size()) {
		if (_blocks[idx] != -1)
			return _blockPool.getObj(_blocks[idx]);
		else if (create) {
			Block* pBlock = nullptr;
			_blocks[idx] = _blockPool.getObj(-1, pBlock, true);
			return pBlock;
		} 
	}
	return nullptr;
}

inline const Block* Volume::getBlock(const Vector3i& blockIdx) const
{
	return getBlock(blockIdx[0], blockIdx[1], blockIdx[2]);
}

inline Block* Volume::getBlock(const Vector3i& blockIdx, bool create)
{
	return getBlock(blockIdx[0], blockIdx[1], blockIdx[2], create);
}

inline size_t Volume::calLinearBlockIndex(size_t ix, size_t iy, size_t iz) const
{
	if (ix < _blockDim[0] && iy < _blockDim[1] && iz < _blockDim[2])
	return ix + _blockDim[0] * (iy + _blockDim[1] * iz);

	return -1;
}

inline size_t Volume::calLinearBlockIndex(const Vector3i& blockIdx) const
{
	return calLinearBlockIndex(blockIdx[0], blockIdx[1], blockIdx[2]);
}

inline Vector3i Volume::calCartesianBlockIndex(size_t idx) const
{
	Vector3i blockIdx;
	size_t temp;

	blockIdx[0] = idx % _blockDim[0];
	temp = idx / _blockDim[0];

	blockIdx[1] = temp % _blockDim[1];
	temp = temp / _blockDim[1];

	blockIdx[2] = temp % _blockDim[2];

	// Back check the math. TODO remove later
	assert(calLinearBlockIndex(blockIdx) == idx);

	return blockIdx;
}

inline Cell* Volume::getCell(const Vector3i& cellIdx)
{
	return getCell(cellIdx[0], cellIdx[1], cellIdx[2]);
}

inline const Cell* Volume::getCell(const Vector3i& cellIdx) const
{
	return getCell(cellIdx[0], cellIdx[1], cellIdx[2]);
}

} // end namespace DFHM