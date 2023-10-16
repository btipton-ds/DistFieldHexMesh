#pragma once

/*
* DistFieldHexMesh
* 
* cell.h
* 
*/

#include <stdint.h>
#include <vector>
#include <mutex>
#include "indices.h"
#include <triMesh.h>
#include <ObjectPool.h>
#include <polygon.h>
#include <polyhedron.h>

namespace TriMesh {
	class CMesh;
	using CMeshPtr = std::shared_ptr<CMesh>;
}

namespace DFHM {

using CMesh = TriMesh::CMesh;

class Cell : public DataPool {
public:
	enum VolumeType {
		VT_UNKNOWN,
		VT_VOID,
		VT_SOLID,
		VT_FLUID,
	};
	VolumeType volType = VT_UNKNOWN;
	std::vector<size_t> _pPolygons; // indices of polygons in this cell
	std::vector<size_t> _pPolyhedra;// indices of polyedra in this cell
};

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

private:

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

class Volume : public DataPool {
public:
	Volume(const Index3& size = Index3(0, 0, 0));
	Volume(const Volume& src);

	void setBlockDims(const Index3& size);
	const Index3& getBlockDims() const;
	const Index3& getDims() const;

	const Block* getBlock(size_t ix, size_t iy, size_t iz) const;
	const Block* getBlock(const Vector3i& blockIdx) const;
	Block* getBlock(size_t ix, size_t iy, size_t iz);
	Block* getBlock(const Vector3i& blockIdx);

	// Currently flow direction is along positive x axis.
	size_t calLinearBlockIndex(size_t ix, size_t iy, size_t iz) const;
	size_t calLinearBlockIndex(const Vector3i& blockIdx) const;
	TriMesh::CMeshPtr buildCFDHexes(const TriMesh::CMeshPtr& pTriMesh, double minCellSize, const Vector3& emptyVolRatio = Vector3(10, 3, 3));
//	bool doesBlockIntersectMesh(const TriMesh::CMeshPtr& pTriMesh, const Vector3i& blockIdx) const;
	TriMesh::CMeshPtr makeTris(bool cells = true);

private:
	void scanVolumePlaneCreateBlocksWhereNeeded(const TriMesh::CMeshPtr& pTriMesh, std::vector<bool>& blocksToCreate, const Vector3i& axisOrder);

	Eigen::Vector3d _originMeters, _spanMeters;
	Index3 _blockDim;

	std::vector<size_t> _faces;
	std::vector<size_t> _polyHedra;
	std::vector<size_t> _blocks;
};

using VolumePtr = std::shared_ptr<Volume>;

inline const Block* Volume::getBlock(size_t ix, size_t iy, size_t iz) const
{
	size_t idx = calLinearBlockIndex(ix, iy, iz);
	if (idx < _blocks.size())
		return _blockPool.getObj(_blocks[idx]);
	return nullptr;
}

inline Block* Volume::getBlock(size_t ix, size_t iy, size_t iz)
{
	size_t idx = calLinearBlockIndex(ix, iy, iz);
	if (idx < _blocks.size())
		return _blockPool.getObj(_blocks[idx]);
	return nullptr;
}

inline const Block* Volume::getBlock(const Vector3i& blockIdx) const
{
	return getBlock(blockIdx[0], blockIdx[1], blockIdx[2]);
}

inline Block* Volume::getBlock(const Vector3i& blockIdx)
{
	return getBlock(blockIdx[0], blockIdx[1], blockIdx[2]);
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

} // end namespace DFHM