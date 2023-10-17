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
#include <triMesh.h>
#include <ObjectPool.h>
#include <polygon.h>
#include <polyhedron.h>
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

	const Block* getBlock(size_t ix, size_t iy, size_t iz) const;
	const Block* getBlock(const Vector3i& blockIdx) const;
	Block* getBlock(size_t ix, size_t iy, size_t iz);
	Block* getBlock(const Vector3i& blockIdx);

	// Currently flow direction is along positive x axis.
	size_t calLinearBlockIndex(size_t ix, size_t iy, size_t iz) const;
	size_t calLinearBlockIndex(const Vector3i& blockIdx) const;
	TriMesh::CMeshPtr buildCFDHexes(const TriMesh::CMeshPtr& pTriMesh, double minCellSize, const Vector3d& emptyVolRatio = Vector3d(10, 3, 3));
//	bool doesBlockIntersectMesh(const TriMesh::CMeshPtr& pTriMesh, const Vector3i& blockIdx) const;
	TriMesh::CMeshPtr makeTris(bool cells = true);

private:
	enum class AxisIndex {
		X, Y, Z
	};
	struct RayTriIntersect {
		double _w = -1;
		size_t _triIdx = -1, _blockIdx = -1, _cellIdx = -1;
	};

	using RayTriIntersectVec = std::vector<RayTriIntersect>;
	using RayBlockIntersectVec = std::vector<RayTriIntersectVec>;

	static Vector3i getAxisOrder(Volume::AxisIndex axisIdx);
	void rayCastAxis(const TriMesh::CMeshPtr& pTriMesh, AxisIndex axis, std::vector<bool>& blocksToCreate);

	Vector3d _originMeters, _spanMeters;
	Index3 _blockDim;

	std::vector<size_t> _faces;
	std::vector<size_t> _polyHedra;
	std::vector<size_t> _blocks;

	std::map<size_t, std::map<size_t, RayBlockIntersectVec>> _xHits, _yHits, _zHits;
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