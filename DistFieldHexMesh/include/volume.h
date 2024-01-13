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
#include <Index3D.h>
#include <triMesh.h>
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


class Volume {
public:
	Volume(const Index3D& size = Index3D(0, 0, 0));
	Volume(const Volume& src);
	~Volume();

	void setOrigin(const Vector3d& origin);
	void setSpan(const Vector3d& span);
	void setBlockDims(const Index3D& size);
	const Index3D& getBlockDims() const;
	const Index3D& getDims() const;

	std::vector<TriMesh::CMeshPtr> addAllBlocks();

	// Get the block using a block index
	bool blockExists(size_t ix, size_t iy, size_t iz) const;
	bool blockExists(const Index3D& blockIdx) const;

	bool blockInBounds(const Index3D& blockIdx) const;

	Block& addBlock(size_t ix, size_t iy, size_t iz);
	Block& addBlock(const Index3D& blockIdx);

	std::shared_ptr<Block> getBlockPtr(const Index3D& blockIdx);

	const Block& getBlock(size_t ix, size_t iy, size_t iz) const;
	const Block& getBlock(const Index3D& blockIdx) const;
	Block& getBlock(size_t ix, size_t iy, size_t iz);
	Block& getBlock(const Index3D& blockIdx);

	// Get the cell using a cell index
	bool cellExists(size_t ix, size_t iy, size_t iz) const;
	bool cellExists(const Index3D& blockIdx) const;
	Cell& getCell(size_t ix, size_t iy, size_t iz);
	Cell& getCell(const Index3D& cellIdx);
	const Cell& getCell(size_t ix, size_t iy, size_t iz) const;
	const Cell& getCell(const Index3D& cellIdx) const;

	// Currently flow direction is along positive x axis.
	size_t calLinearBlockIndex(size_t ix, size_t iy, size_t iz) const;
	size_t calLinearBlockIndex(const Index3D& blockIdx) const;
	Index3D calBlockIndexFromLinearIndex(size_t linearIdx) const;

	std::vector<TriMesh::CMeshPtr> buildCFDHexes(const TriMesh::CMeshPtr& pTriMesh, double minCellSize, bool outerFacesOnly);
	std::vector<TriMesh::CMeshPtr> makeTris(bool outerOnly, bool multiCore);

	size_t numFaces(bool includeInner) const;
	size_t numPolyhedra() const;

	void writePolyMesh(const std::string& dirName) const;

	void dumpSections(const std::string& dirName) const;

private:
	using AxisIndex = Block::AxisIndex;

	void createBlockRays(AxisIndex axisIdx, std::vector<bool>& blocksToCreate);
	void createBlockCellRays(AxisIndex axisIdx, const std::vector<bool>& blocksToCreate, std::vector<std::vector<bool>>& cellsToCreate);
	void processRayHit(const RayHit& triHit, int rayAxis, const Vector3d& blockSpan, const Vector3d& cellSpan, size_t& blockIdx, size_t& cellIdx);

	void writePolyMeshPoints(const std::string& dirName) const;
	void writeFOAMHeader(std::ofstream& out, const std::string& foamClass, const std::string& object) const;

	TriMesh::CMeshPtr _pModelTriMesh;
	Vector3d _originMeters, _spanMeters;
	Index3D _blockDim;

	std::vector<std::shared_ptr<Block>> _blocks;

};

using VolumePtr = std::shared_ptr<Volume>;


inline void Volume::setOrigin(const Vector3d& origin)
{
	_originMeters = origin;
}

inline void Volume::setSpan(const Vector3d& span)
{
	_spanMeters = span;
}

inline std::shared_ptr<Block> Volume::getBlockPtr(const Index3D& blockIdx)
{
	return _blocks[calLinearBlockIndex(blockIdx)];
}

inline const Block& Volume::getBlock(const Index3D& blockIdx) const
{
	return getBlock(blockIdx[0], blockIdx[1], blockIdx[2]);
}

inline Block& Volume::getBlock(const Index3D& blockIdx)
{
	return getBlock(blockIdx[0], blockIdx[1], blockIdx[2]);
}

inline size_t Volume::calLinearBlockIndex(size_t ix, size_t iy, size_t iz) const
{
	if (ix < _blockDim[0] && iy < _blockDim[1] && iz < _blockDim[2])
		return ix + _blockDim[0] * (iy + _blockDim[1] * iz);

	return -1;
}

inline size_t Volume::calLinearBlockIndex(const Index3D& blockIdx) const
{
	return calLinearBlockIndex(blockIdx[0], blockIdx[1], blockIdx[2]);
}

inline Index3D Volume::calBlockIndexFromLinearIndex(size_t linearIdx) const
{
	Index3D result;
	size_t temp = linearIdx;

//		ix + _blockDim[0] * iy + _blockDim[0] * _blockDim[1] * iz;


	size_t denom = _blockDim[0] * _blockDim[1];
	result[2] = (Index3DBaseType) (temp / denom);
	temp = temp % denom;

	denom = _blockDim[0];

	result[1] = (Index3DBaseType)(temp / denom);
	temp = temp % denom;
	result[0] = (Index3DBaseType)temp;

	if (calLinearBlockIndex(result) != linearIdx) {
		throw std::runtime_error("calBlockIndexFromLinearIndex failed");
	}

	return result;
}

inline Cell& Volume::getCell(const Index3D& cellIdx)
{
	return getCell(cellIdx[0], cellIdx[1], cellIdx[2]);
}

inline const Cell& Volume::getCell(const Index3D& cellIdx) const
{
	return getCell(cellIdx[0], cellIdx[1], cellIdx[2]);
}

} // end namespace DFHM
