

#include <triMesh.h>

#include <volume.h>

using namespace std;
using namespace DFHM;
using namespace TriMesh;

Volume::Volume(const Index3& blockSize)
{
	setBlockDims(blockSize);
}

Volume::Volume(const Volume& src)
	: _originMeters(src._originMeters), 
	_spanMeters(_spanMeters),
	_blockDim(src._blockDim),
	_cellPool(src._cellPool),
	_blocks(src._blocks)
{
}

void Volume::setBlockDims(const Index3& blockSize)
{
	_blockDim = blockSize;
	_blocks.resize(_blockDim[0] * _blockDim[1] * _blockDim[2]);
}

const Index3& Volume::getBlockDims() const
{
	return _blockDim;
}

void Volume::buildCFDHexes(const CMeshPtr& pTriMesh, double minCellSize, const Vector3& emptyVolRatio)
{
	CMesh::BoundingBox bb = pTriMesh->getBBox();
	_originMeters = bb.getMin();
	Vector3 halfRange = bb.range() / 2;

	_spanMeters = bb.range();
	Index3 blockSize(
		(size_t)(_spanMeters[0] / minCellSize / Block::getBlockDim() + 0.5),
		(size_t)(_spanMeters[1] / minCellSize / Block::getBlockDim() + 0.5),
		(size_t)(_spanMeters[2] / minCellSize / Block::getBlockDim() + 0.5)
	);

	setBlockDims(blockSize);

	Vector3d xAxis(1, 0, 0);
	Vector3d yAxis(0, 1, 0);
	Vector3d zAxis(0, 0, 1);

	// scan xy plane
	Vector3d rayOrigin(0, 0, _originMeters[2]);
	for (size_t bx = 0; bx <= _blockDim[0]; bx++) {
		double tx = bx / (double)_blockDim[0];
		rayOrigin[0] = _originMeters[0] + tx * _spanMeters[0];

		for (size_t by = 0; by <= _blockDim[1]; by++) {
			double ty = by / (double)_blockDim[1];
			rayOrigin[1] = _originMeters[1] + ty * _spanMeters[1];

			Ray ray(rayOrigin, zAxis);
			vector<RayHit> hits;
			if (pTriMesh->biDirRayCast(ray, hits)) {
				if (hits.size() % 2 == 0) {
					for (size_t i = 0; i < hits.size(); i++) {
						double t = hits[i].dist / _spanMeters[2];
						size_t idx = (size_t)(blockSize[2] * t);
						cout << "d: " << hits[i].dist << " t: " << t << " idx[" << i << "]: " << idx << "\n";
					}
				} else {

				}
			}
		}

	}
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
