

#include <triMesh.h>

#include "volume.h"

using namespace std;
using namespace DFHM;

Volume::Volume(const Index3& size)
{
	setSize(size);
}

Volume::Volume(const Volume& src)
	: _originMeters(src._originMeters), 
	_spanMeters(_spanMeters),
	_size(src._size),
	_cellPool(src._cellPool),
	_blocks(src._blocks)
{
}

void Volume::setSize(const Index3& size)
{
	_size = size;
	_blocks.resize(_size[0] * _size[1] * _size[2]);
}

const Index3& Volume::getSize() const
{
	return _size;
}

void Volume::buildCFDMesh(const CMesh& triMesh, double minCellSize, const Vector3& emptyVolRatio)
{
	CMesh::BoundingBox bb = triMesh.getBBox();
	_originMeters = (bb.getMin() + bb.getMax()) * 0.5;
	Vector3 halfRange = bb.range() / 2;

	double gap = triMesh.findMinGap();

	{
		CMesh::BoundingBox bbTmp;
		Vector3 v(halfRange[0] * emptyVolRatio[0], halfRange[1] * emptyVolRatio[1], halfRange[2] * emptyVolRatio[2]);
		for (int i = 0; i < 2; i++) {
			for (int j = 0; j < 2; j++) {
				for (int k = 0; k < 2; k++) {
					Vector3 p = _originMeters + Vector3(
						(i == 0 ? -1 : 1) * v[0],
						(j == 0 ? -1 : 1) * v[1],
						(k == 0 ? -1 : 1) * v[2]);
					bbTmp.merge(p);;
				}
			}
		}
		bb = bbTmp;
	}

	_spanMeters = bb.range();
	Index3 size(
		(size_t)(_spanMeters[0] / minCellSize / Block::getBlockDim() + 0.5),
		(size_t)(_spanMeters[1] / minCellSize / Block::getBlockDim() + 0.5),
		(size_t)(_spanMeters[2] / minCellSize / Block::getBlockDim() + 0.5)
	);

	setSize(size);

}

/******************** Block **************************/

size_t Block::s_blockDim = 8;

void Block::setBlockDim(size_t dim)
{
	s_blockDim = dim;
}

size_t Block::getBlockDim()
{
	return s_blockDim;
}

Block::Block()
{}

Block::Block(const Block& src)
	: _cells(src._cells)
{

}
