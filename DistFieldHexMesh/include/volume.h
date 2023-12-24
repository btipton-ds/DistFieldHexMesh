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

	// Get the block using a block index
	bool blockExists(size_t ix, size_t iy, size_t iz) const;
	bool blockExists(const Vector3i& blockIdx) const;
	Block& addBlock(size_t ix, size_t iy, size_t iz, size_t threadNum);
	Block& addBlock(const Vector3i& blockIdx, size_t threadNum);
	const Block& getBlock(size_t ix, size_t iy, size_t iz) const;
	const Block& getBlock(const Vector3i& blockIdx) const;
	Block& getBlock(size_t ix, size_t iy, size_t iz);
	Block& getBlock(const Vector3i& blockIdx);

	// Get the cell using a cell index
	bool cellExists(size_t ix, size_t iy, size_t iz) const;
	bool cellExists(const Vector3i& blockIdx) const;
	Cell& getCell(size_t ix, size_t iy, size_t iz);
	Cell& getCell(const Vector3i& cellIdx);
	const Cell& getCell(size_t ix, size_t iy, size_t iz) const;
	const Cell& getCell(const Vector3i& cellIdx) const;

	// Currently flow direction is along positive x axis.
	size_t calLinearBlockIndex(size_t ix, size_t iy, size_t iz) const;
	size_t calLinearBlockIndex(const Vector3i& blockIdx) const;
	TriMesh::CMeshPtr buildCFDHexes(const TriMesh::CMeshPtr& pTriMesh, double minCellSize, const Vector3d& emptyVolRatio = Vector3d(10, 3, 3));
//	bool doesBlockIntersectMesh(const TriMesh::CMeshPtr& pTriMesh, const Vector3i& blockIdx) const;
	TriMesh::CMeshPtr makeTris(bool cells = true);

	void writePolyMesh(const std::string& dirName) const;

	void dumpSections(const std::string& dirName) const;

private:
	using AxisIndex = Block::AxisIndex;
	using RayTriIntersect = Block::RayTriIntersect;

	using RayTriIntersectVec = Block::RayTriIntersectVec;
	using RayBlockIntersectVec = Block::RayBlockIntersectVec;

	void createBlockRays(AxisIndex axisIdx, const TriMesh::CMeshPtr& pTriMesh, std::vector<bool>& blocksToCreate);
	void createBlockCellRays(AxisIndex axisIdx, const TriMesh::CMeshPtr& pTriMesh, const std::vector<bool>& blocksToCreate, std::vector<std::vector<bool>>& cellsToCreate);
	void processRayHit(const RayHit& triHit, int rayAxis, const Vector3d& blockSpan, const Vector3d& cellSpan, size_t& blockIdx, size_t& cellIdx);

	void writePolyMeshPoints(const std::string& dirName) const;
	void writeFOAMHeader(std::ofstream& out, const std::string& foamClass, const std::string& object) const;

	mutable std::mutex _mutex;
	Vector3d _originMeters, _spanMeters;
	Index3 _blockDim;

	std::vector<ObjectPoolId> _faces;
	std::vector<ObjectPoolId> _polyHedra;
	std::vector<ObjectPoolId> _blocks;

};

using VolumePtr = std::shared_ptr<Volume>;



inline const Block& Volume::getBlock(const Vector3i& blockIdx) const
{
	return getBlock(blockIdx[0], blockIdx[1], blockIdx[2]);
}

inline Block& Volume::getBlock(const Vector3i& blockIdx)
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

inline Cell& Volume::getCell(const Vector3i& cellIdx)
{
	return getCell(cellIdx[0], cellIdx[1], cellIdx[2]);
}

inline const Cell& Volume::getCell(const Vector3i& cellIdx) const
{
	return getCell(cellIdx[0], cellIdx[1], cellIdx[2]);
}

} // end namespace DFHM