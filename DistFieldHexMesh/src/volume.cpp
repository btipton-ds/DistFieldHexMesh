

#include <triMesh.h>

#include <volume.h>
#include <MultiCoreUtil.h>

using namespace std;
using namespace DFHM;
using namespace TriMesh;

ObjectPool<Polygon> DataPool::_polygonPool;
ObjectPool<Polyhedron> DataPool::_polyhedronPool;
ObjectPool<Cell> DataPool::_cellPool;
ObjectPool<Block> DataPool::_blockPool;

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

#define RUN_MULTI_THREAD true

void Volume::scanVolumePlaneCreateBlocksWhereNeeded(const CMeshPtr& pTriMesh, const Vector3d& origin, const Vector3i& axisOrder)
{
	MultiCore::runLambda([this, pTriMesh, axisOrder, origin](size_t threadNum, size_t numThreads) {
		Vector3d axis;

		Vector3d rayOrigin(origin);
		switch (axisOrder[2]) {
		case 0:
			rayOrigin[0] = origin[0];
			axis = Vector3d(1, 0, 0);
			break;
		case 1:
			rayOrigin[1] = origin[1];
			axis = Vector3d(0, 1, 0);
			break;
		case 2:
			rayOrigin[2] = origin[2];
			axis = Vector3d(0, 0, 1);
			break;
		}

		Vector3 blockRange;
		const auto& volumeRange = pTriMesh->getBBox().range();
		for (int i = 0; i < 3; i++) {
			blockRange[i] = volumeRange[i] / _blockDim[i];
		}
		size_t axisIdx0 = axisOrder[0];
		size_t axisIdx1 = axisOrder[1];
		size_t axisIdx2 = axisOrder[2];

		for (size_t i = threadNum; i <= _blockDim[axisIdx0]; i += numThreads) {
			double ti = i / (double)_blockDim[axisIdx0];
			rayOrigin[axisIdx0] = _originMeters[axisIdx0] + ti * _spanMeters[axisIdx0];

			for (size_t j = 0; j <= _blockDim[axisIdx1]; j++) {
				double tj = j / (double)_blockDim[axisIdx1];
				rayOrigin[axisIdx1] = _originMeters[axisIdx1] + tj * _spanMeters[axisIdx1];

				Ray ray(rayOrigin, axis);
				// We need to support solids and surfaces
				// Solids will always have crossing in pairs, except at tangential crossings
				// Surface crossings are singular
				// This sorts them out
				vector<RayHit> hits;
				if (pTriMesh->biDirRayCast(ray, hits)) {
					for (const auto hit : hits) {

						double tk = hit.dist / _spanMeters[axisIdx2];
						size_t k0 = (size_t)(_blockDim[axisIdx2] * tk);
						size_t k1 = (size_t)(_blockDim[axisIdx2] * tk + 0.5);
						for (size_t k = k0; k <= k1; k++) {
							size_t ix, iy, iz;
							switch (axisOrder[0]) {
								case 0: ix = i; break;
								case 1: iy = i; break;
								case 2: iz = i; break;
							}

							switch (axisOrder[1]) {
								case 0: ix = j; break;
								case 1: iy = j; break;
								case 2: iz = j; break;
							}

							switch (axisOrder[2]) {
								case 0: ix = k; break;
								case 1: iy = k; break;
								case 2: iz = k; break;
							}

							Vector3i blockIdx(ix, iy, iz);
							size_t bIdx = calLinearBlockIndex(blockIdx);
							if (!_blocks[bIdx]) {
								auto pBlock = make_shared<Block>();
								_blocks[bIdx] = pBlock;
								Vector3 cellOrigin = origin;
								cellOrigin[0] += volumeRange[0] * (ix / (double)_blockDim[0]);
								cellOrigin[1] += volumeRange[1] * (iy / (double)_blockDim[1]);
								cellOrigin[2] += volumeRange[2] * (iz / (double)_blockDim[2]);
							}
						}
					}

				}
			}

		}
	}, RUN_MULTI_THREAD);
}

void Volume::scanVolumeCreateCellsWhereNeeded(const TriMesh::CMeshPtr& pTriMesh)
{
	auto volOrigin = pTriMesh->getBBox().getMin();

	const auto& range = pTriMesh->getBBox().range();

	Vector3 blockSpan;
	for (int i = 0; i < 3; i++)
		blockSpan[i] = range[i] / _blockDim[i];

	MultiCore::runLambda([this, pTriMesh, volOrigin, blockSpan](size_t threadNum, size_t numThreads) {
		for (size_t i = threadNum; i < _blockDim[0]; i += numThreads) {
			for (size_t j = 0; j < _blockDim[1]; j++) {
				for (size_t k = 0; k < _blockDim[2]; k++) {
					size_t blockIdx = calLinearBlockIndex(i, j, k);
					auto pBlock = _blocks[blockIdx];
					if (pBlock) {
						Vector3 origin = volOrigin;

						origin[0] += i * blockSpan[0];
						origin[1] += j * blockSpan[1];
						origin[2] += k * blockSpan[2];

						pBlock->scanCreateCellsWhereNeeded(pTriMesh, origin, blockSpan, Vector3i(0, 1, 2));
						pBlock->scanCreateCellsWhereNeeded(pTriMesh, origin, blockSpan, Vector3i(2, 0, 1));
						pBlock->scanCreateCellsWhereNeeded(pTriMesh, origin, blockSpan, Vector3i(1, 2, 0));
					}
				}
			}
		}
	}, RUN_MULTI_THREAD);
}


void Block::scanCreateCellsWhereNeeded(const TriMesh::CMeshPtr& pTriMesh, const Vector3d& origin, const Vector3d& blockSpan, const Vector3i& axisOrder)
{
	static mutex lock;
	const double TOL = 1.0e-6;
	static size_t numCells = 0;
	Vector3d axis;
	Vector3d cellSpan = blockSpan * (1.0 / s_blockDim);

	Vector3d rayOrigin(origin);
	switch (axisOrder[2]) {
	case 0:
		rayOrigin[0] = origin[0];
		axis = Vector3d(1, 0, 0);
		break;
	case 1:
		rayOrigin[1] = origin[1];
		axis = Vector3d(0, 1, 0);
		break;
	case 2:
		rayOrigin[2] = origin[2];
		axis = Vector3d(0, 0, 1);
		break;
	}

	size_t axisIdx0 = axisOrder[0];
	size_t axisIdx1 = axisOrder[1];
	size_t axisIdx2 = axisOrder[2];

	size_t initialNumCells = numCells;
	for (size_t i = 0; i < s_blockDim; i ++) {
		double ti = i / (double)s_blockDim;
		rayOrigin[axisIdx0] = origin[axisIdx0] + ti * blockSpan[axisIdx0];

		for (size_t j = 0; j < s_blockDim; j++) {
			double tj = j / (double)s_blockDim;
			rayOrigin[axisIdx1] = origin[axisIdx1] + tj * blockSpan[axisIdx1];

			Ray ray(rayOrigin, axis);
			// We need to support solids and surfaces
			// Solids will always have crossing in pairs, except at tangential crossings
			// Surface crossings are singular
			// This sorts them out
			vector<RayHit> hits;
			if (pTriMesh->biDirRayCast(ray, hits)) {
				for (const auto hit : hits) {

					double tk = hit.dist / cellSpan[axisIdx2];
					size_t k0 = (size_t)(s_blockDim * tk);
					if (0 <= k0 && k0 < s_blockDim) {
						size_t k1 = (size_t)(s_blockDim * tk + 0.5);
						if (k1 >= s_blockDim)
							k1 = s_blockDim - 1;
						lock.lock();
						cout << "k0: " << k0 << ", k1" << k1 << "\n";
						lock.unlock();
						for (size_t k = k0; k <= k1; k++) {
							size_t ix, iy, iz;
							switch (axisOrder[0]) {
							case 0: ix = i; break;
							case 1: iy = i; break;
							case 2: iz = i; break;
							}

							switch (axisOrder[1]) {
							case 0: ix = j; break;
							case 1: iy = j; break;
							case 2: iz = j; break;
							}

							switch (axisOrder[2]) {
							case 0: ix = k; break;
							case 1: iy = k; break;
							case 2: iz = k; break;
							}

							size_t cellIdx = calcCellIndex(ix, iy, iz);
							if (!_cells[cellIdx]) {
								auto pBlock = make_shared<Cell>();
								lock.lock();
								numCells++;
								if (numCells % 10000 == 0) {
									cout << numCells << "\n";
								}
								lock.unlock();

								Cell* pCell = nullptr;
								_cells[cellIdx] = _cellPool.getObj(-1, pCell, true);
								Vector3 cellOrigin = origin;
								cellOrigin[0] += blockSpan[0] * (ix / (double)s_blockDim);
								cellOrigin[1] += blockSpan[0] * (iy / (double)s_blockDim);
								cellOrigin[2] += blockSpan[0] * (iz / (double)s_blockDim);
							}
						}
					}
				}

			}
		}

	}
	if (initialNumCells != numCells) {
		lock.lock();
		cout << numCells << "\n";
		lock.unlock();
	}
}

CMeshPtr Volume::buildCFDHexes(const CMeshPtr& pTriMesh, double minCellSize, const Vector3& emptyVolRatio)
{
	CMesh::BoundingBox bb = pTriMesh->getBBox();
	_originMeters = bb.getMin();
	_spanMeters = bb.range();

	Index3 blockSize(
		(size_t)(_spanMeters[0] / minCellSize / Block::getBlockDim() + 0.5),
		(size_t)(_spanMeters[1] / minCellSize / Block::getBlockDim() + 0.5),
		(size_t)(_spanMeters[2] / minCellSize / Block::getBlockDim() + 0.5)
	);

	setBlockDims(blockSize);

	// Scan the model in from all three orthoganal planes and create all intersecting blocks
	scanVolumePlaneCreateBlocksWhereNeeded(pTriMesh, _originMeters, Vector3i(0, 1, 2));
	scanVolumePlaneCreateBlocksWhereNeeded(pTriMesh, _originMeters, Vector3i(2, 0, 1));
	scanVolumePlaneCreateBlocksWhereNeeded(pTriMesh, _originMeters, Vector3i(1, 2, 0));

//	scanVolumeCreateCellsWhereNeeded(pTriMesh);

	CMeshPtr result = makeTris(false);

	return result;
}

CMeshPtr Volume::makeTris(bool cells)
{
	CMeshPtr result = make_shared<CMesh>();

	Vector3d blockSpan, blockOrigin;

	for (int i = 0; i < 3; i++)
		blockSpan[i] = _spanMeters[i] / _blockDim[i];

	Vector3i blockIdx;
	for (blockIdx[0] = 0; blockIdx[0] < _blockDim[0]; blockIdx[0]++) {
		blockOrigin[0] = _originMeters[0] + blockIdx[0] * blockSpan[0];
		for (blockIdx[1] = 0; blockIdx[1] < _blockDim[1]; blockIdx[1]++) {
			blockOrigin[1] = _originMeters[1] + blockIdx[1] * blockSpan[1];
			for (blockIdx[2] = 0; blockIdx[2] < _blockDim[2]; blockIdx[2]++) {
				blockOrigin[2] = _originMeters[2] + blockIdx[2] * blockSpan[2];
				size_t bIdx = calLinearBlockIndex(blockIdx);
				auto pBlock = _blocks[bIdx];
				if (pBlock) {
					pBlock->addBlockTris(blockOrigin, blockSpan, result, cells);
				}
			}
		}
	}

	return result;
}

void Block::addBlockTris(const Vector3d& blockOrigin, const Vector3d& blockSpan, TriMesh::CMeshPtr& pMesh, bool useCells)
{

	if (useCells && !_cells.empty()) {

	} else {
		vector<Vector3d> pts = {
			Vector3d(blockOrigin[0], blockOrigin[1], blockOrigin[2]),
			Vector3d(blockOrigin[0], blockOrigin[1] + blockSpan[1], blockOrigin[2]),
			Vector3d(blockOrigin[0] + blockSpan[0], blockOrigin[1] + blockSpan[1], blockOrigin[2]),
			Vector3d(blockOrigin[0] + blockSpan[0], blockOrigin[1], blockOrigin[2]),

			Vector3d(blockOrigin[0], blockOrigin[1], blockOrigin[2] + blockSpan[2]),
			Vector3d(blockOrigin[0] + blockSpan[0], blockOrigin[1], blockOrigin[2] + blockSpan[2]),
			Vector3d(blockOrigin[0] + blockSpan[0], blockOrigin[1] + blockSpan[1], blockOrigin[2] + blockSpan[2]),
			Vector3d(blockOrigin[0], blockOrigin[1] + blockSpan[1], blockOrigin[2] + blockSpan[2]),
		};
		pMesh->addRectPrism(pts);
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
{
	_cells.resize(s_blockDim * s_blockDim * s_blockDim);
}

Block::Block(const Block& src)
	: _cells(src._cells)
{

}
