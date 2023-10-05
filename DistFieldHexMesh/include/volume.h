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
}

namespace DFHM {

using CMesh = TriMesh::CMesh;

class Volume;
class Block;

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
	std::vector<size_t> _cells;
};

class Volume {
	Volume(const Index3& size = Index3(0, 0, 0));
	Volume(const Volume& src);

	void setSize(const Index3& size);
	const Index3& getSize() const;

	// Currently flow direction is along positive x axis.
	void buildCFDMesh(const CMesh& triMesh, double minCellSize, const Vector3& emptyVolRatio = Vector3(10, 3, 3));

private:
	Eigen::Vector3d _originMeters, _spanMeters;
	Index3 _size;
	HalfJackPool _cellPool;

	std::vector<Block> _blocks;
};

} // end namespace DFHM