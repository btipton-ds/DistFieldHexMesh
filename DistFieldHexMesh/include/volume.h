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

class Cell;
class Block;
class Polygon;
class Polyhedron;
class Volume;

using VolumePtr = std::shared_ptr<Volume>;

template<class T>
class ObjectPool {
public:
	inline void free(size_t index)
	{
		_available.push_back(index);
	}

	inline size_t getObj(size_t index, T* pObj, bool allocateIfNeeded)
	{
		size_t result = -1;
		if (index != -1 && index < _pool.size()) {
			result = index;
		} else {
			if (_available.empty()) {
				result = _pool.size();
				_pool.push_back(T());
			} else {
				result = _available.back();
				_available.pop_back();
				{
					_pool[result] = T();
				}
			}
		}
		pObj = &_pool[result];
		return result;
	}

	inline const T* getObj(size_t index) const
	{
		if (index != -1 && index < _pool.size()) {
			return &_pool[index];
		}
		return nullptr;
	}

private:
	std::vector<size_t> _available;
	std::vector<T> _pool;
};

class DataPool
{
protected:
	static ObjectPool<Polygon> _polygonPool;
	static ObjectPool<Polyhedron> _polyhedronPool;
	static ObjectPool<Cell> _cellPool;
	static ObjectPool<Block> _blockPool;
};

class Cell : public DataPool {
public:
	enum VolumeType {
		VT_VOID,
		VT_SOLID,
		VT_FLUID,
	};
	VolumeType volType;
	std::vector<size_t> polygons; // indices of polygons in this cell
	std::vector<size_t> polyhedra;// indices of polyedra in this cell
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

class Block : public DataPool {
public:
	static void setBlockDim(size_t dim);
	static size_t getBlockDim();

	Block();
	Block(const Block& src);

	void scanCreateCellsWhereNeeded(const TriMesh::CMeshPtr& pTriMesh, const Vector3d& origin, const Vector3d& blockSpan, const Vector3i& axisOrder);
	size_t calcCellIndex(size_t ix, size_t iy, size_t iz) const;
	size_t calcCellIndex(const Vector3i& celIdx) const;
	void addBlockTris(const Vector3d& blockOrigin, const Vector3d& blockSpan, TriMesh::CMeshPtr& pMesh, bool useCells);

private:
	static size_t s_blockDim;
	std::vector<size_t> _cells;
};

inline size_t Block::calcCellIndex(size_t ix, size_t iy, size_t iz) const
{
	size_t result = ix + s_blockDim * (iy + s_blockDim * iz);
	return result;
}

inline size_t Block::calcCellIndex(const Vector3i& celIdx) const
{
	return calcCellIndex(celIdx[0], celIdx[1], celIdx[2]);
}

class Polygon : public DataPool {
public:
	std::vector<size_t> vertexIndices;
};

class Polyhedron : public DataPool {
public:
	std::vector<size_t> faceIndices;
};

class Volume : public DataPool {
public:
	Volume(const Index3& size = Index3(0, 0, 0));
	Volume(const Volume& src);

	void setBlockDims(const Index3& size);
	const Index3& getBlockDims() const;
	const Index3& getDims() const;

	std::shared_ptr<Block> getBlock(size_t ix, size_t iy, size_t iz);
	std::shared_ptr<const Block> getBlock(size_t ix, size_t iy, size_t iz) const;

	// Currently flow direction is along positive x axis.
	size_t calLinearBlockIndex(size_t ix, size_t iy, size_t iz) const;
	size_t calLinearBlockIndex(const Vector3i& blockIdx) const;
	TriMesh::CMeshPtr buildCFDHexes(const TriMesh::CMeshPtr& pTriMesh, double minCellSize, const Vector3& emptyVolRatio = Vector3(10, 3, 3));
//	bool doesBlockIntersectMesh(const TriMesh::CMeshPtr& pTriMesh, const Vector3i& blockIdx) const;
	TriMesh::CMeshPtr makeTris(bool cells = true);

private:
	void scanVolumePlaneCreateBlocksWhereNeeded(const TriMesh::CMeshPtr& pTriMesh, const Vector3d& origin, const Vector3i& axisOrder);
	void scanVolumeCreateCellsWhereNeeded(const TriMesh::CMeshPtr& pTriMesh);

	Eigen::Vector3d _originMeters, _spanMeters;
	Index3 _blockDim;
	HalfJackPool _cellPool;
	std::vector<size_t> _faces;
	std::vector<size_t> _polyHedra;

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

inline size_t Volume::calLinearBlockIndex(const Vector3i& blockIdx) const
{
	return calLinearBlockIndex(blockIdx[0], blockIdx[1], blockIdx[2]);
}

} // end namespace DFHM