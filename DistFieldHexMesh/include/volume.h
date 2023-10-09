#pragma once

/*
* DistFieldHexMesh
* 
* cell.h
* 
*/

#include <stdint.h>
#include <vector>
#include "indices.h"

namespace TriMesh {
	class CMesh;
	using CMeshPtr = std::shared_ptr<CMesh>;
}

namespace DFHM {

using CMesh = TriMesh::CMesh;

class Volume;
class Block;

using VolumePtr = std::shared_ptr<Volume>;

class Cell {
public:
	enum VolumeType {
		VT_VOID,
		VT_SOLID,
		VT_FLUID,
	};
	VolumeType volType;
	std::vector<size_t> polyhedra;
};

class HalfJack {
public:
	HalfJack();

private:
	// This best thought of a cartesian coordinate system, however it supports non-orthoganal axes
	uint32_t _x, _y, _z; // fraction of span where an intersection occurs x = x0 + vx * (cellNum + _x / UINT32_MAX)
};

class HalfJackPool {
	size_t allocate();
	void free(size_t idx);
	const HalfJack* get(size_t idx) const;
	HalfJack* get(size_t idx);

private:
	std::vector<HalfJack> _cells;
	std::vector<size_t> _avalCells;
};

class Block {
public:
	static void setBlockDim(size_t dim);
	static size_t getBlockDim();

	Block();
	Block(const Block& src);

private:
	static size_t s_blockDim;
	std::vector< std::shared_ptr<Cell>> _cells;
};

class Polygon {
public:
	std::vector<size_t> vertexIndices;
};

class PolyHedron {
public:
	std::vector<size_t> faceIndices;
};

class Volume {
public:
	Volume(const Index3& size = Index3(0, 0, 0));
	Volume(const Volume& src);

	void setBlockDims(const Index3& size);
	const Index3& getBlockDims() const;
	const Index3& getDims() const;

	std::shared_ptr<Block> getBlock(size_t ix, size_t iy, size_t iz);
	std::shared_ptr<const Block> getBlock(size_t ix, size_t iy, size_t iz) const;

	// Currently flow direction is along positive x axis.
	void buildCFDHexes(const TriMesh::CMeshPtr& pTriMesh, double minCellSize, const Vector3& emptyVolRatio = Vector3(10, 3, 3));

private:
	size_t calLinearBlockIndex(size_t ix, size_t iy, size_t iz) const;

	Eigen::Vector3d _originMeters, _spanMeters;
	Index3 _blockDim;
	HalfJackPool _cellPool;
	std::vector<Polygon> _faces;
	std::vector<PolyHedron> _polyHedra;

	std::vector<std::shared_ptr<Block>> _blocks;
};

using VolumePtr = std::shared_ptr<Volume>;

inline std::shared_ptr<Block> Volume::getBlock(size_t ix, size_t iy, size_t iz)
{
	size_t idx = calLinearBlockIndex(ix, iy, iz);
	if (idx < _blocks.size())
		return _blocks[idx];
	return nullptr;
}

inline std::shared_ptr<const Block> Volume::getBlock(size_t ix, size_t iy, size_t iz) const
{
	size_t idx = calLinearBlockIndex(ix, iy, iz);
	if (idx < _blocks.size())
		return _blocks[idx];
	return nullptr;
}

inline size_t Volume::calLinearBlockIndex(size_t ix, size_t iy, size_t iz) const
{
	size_t result = ix + _blockDim[1] * (iy + _blockDim[2] * iz);

	return result;
}

} // end namespace DFHM