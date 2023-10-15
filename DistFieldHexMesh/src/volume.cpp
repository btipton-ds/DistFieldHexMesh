

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
	_blocks.resize(_blockDim[0] * _blockDim[1] * _blockDim[2], -1);
}

const Index3& Volume::getBlockDims() const
{
	return _blockDim;
}

#define RUN_MULTI_THREAD true

void Volume::scanVolumePlaneCreateBlocksWhereNeeded(const CMeshPtr& pTriMesh, std::vector<bool>& blocksToCreate, const Vector3i& axisOrder)
{
	if (blocksToCreate.empty())
		blocksToCreate.resize(_blocks.size());
	MultiCore::runLambda([this, pTriMesh, &blocksToCreate, axisOrder](size_t threadNum, size_t numThreads) {
		int overFlow = 1;
		Vector3d axis;

		switch (axisOrder[2]) {
		case 0:
			axis = Vector3d(1, 0, 0);
			break;
		case 1:
			axis = Vector3d(0, 1, 0);
			break;
		case 2:
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

		Eigen::Vector3d offset(0, 0, 0);

		for (size_t i = threadNum; i < _blockDim[axisIdx0]; i += numThreads) {
			offset[axisIdx0] = i * blockRange[axisIdx0];

			for (size_t j = 0; j < _blockDim[axisIdx1]; j++) {
				offset[axisIdx1] = j * blockRange[axisIdx1];

				Eigen::Vector3d rayOrigin = _originMeters + offset;

				Ray ray(rayOrigin, axis);
				// We need to support solids and surfaces
				// Solids will always have crossing in pairs, except at tangential crossings
				// Surface crossings are singular
				// This sorts them out
				vector<RayHit> hits;
				if (pTriMesh->biDirRayCast(ray, hits)) {
					for (const auto& hit : hits) {
//						cout << "x: " << (offset[axisIdx0] / volumeRange[axisIdx0]) << ", y: " << (offset[axisIdx1] / volumeRange[axisIdx1]) << ", z: " << (hit.dist / volumeRange[axisIdx2]) << "\n";

						double tk = hit.dist / _spanMeters[axisIdx2];
						size_t k = (size_t)(_blockDim[axisIdx2] * tk);
						if (k < 0)
							k = 0;
						if (k >= _blockDim[axisIdx2])
							k = _blockDim[axisIdx2] - 1;
						{
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

							size_t iix = ix, iiy = iy, iiz = iz;
							for (int ii = -overFlow; ii <= overFlow; ii++) {
								switch (axisOrder[0]) {
									case 0: iix = ix + ii; break;
									case 1: iiy = iy + ii; break;
									case 2: iiz = iz + ii; break;
								}

								for (int jj = -overFlow; jj <= overFlow; jj++) {
									switch (axisOrder[1]) {
										case 0: iix = ix + jj; break;
										case 1: iiy = iy + jj; break;
										case 2: iiz = iz + jj; break;
									}

									for (int kk = -overFlow; kk <= overFlow; kk++) {
										switch (axisOrder[2]) {
											case 0: iix = ix + kk; break;
											case 1: iiy = iy + kk; break;
											case 2: iiz = iz + kk; break;
										}

										size_t bIdx = calLinearBlockIndex(iix, iiy, iiz);
										// No mutex required, because order of setting true is not a race condition.
										if (bIdx != -1)
											blocksToCreate[bIdx] = true;
									}
								}
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
					Block* pBlock = getBlock(i, j, k);
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
	vector<bool> blocksToCreate;

	// Scan the model in from all three orthoganal planes and create all intersecting blocks
	scanVolumePlaneCreateBlocksWhereNeeded(pTriMesh, blocksToCreate, Vector3i(0, 1, 2));
	scanVolumePlaneCreateBlocksWhereNeeded(pTriMesh, blocksToCreate, Vector3i(2, 0, 1));
	scanVolumePlaneCreateBlocksWhereNeeded(pTriMesh, blocksToCreate, Vector3i(1, 2, 0));

//	scanVolumeCreateCellsWhereNeeded(pTriMesh);

#if 0
	for (size_t iz = 0; iz < _blockDim[2]; iz++) {
		cout << "Slice " << iz << "\n";

		for (size_t iy = 0; iy < _blockDim[1]; iy++) {
			for (size_t ix = 0; ix < _blockDim[0]; ix++) {
				auto idx = calLinearBlockIndex(ix, iy, iz);
				if (blocksToCreate[idx])
					cout << "X";
				else
					cout << " ";
			}
			cout << "\n";
		}

		cout << "\n\n";
	}
#endif

	for (size_t i = 0; i < blocksToCreate.size(); i++) {
		if (blocksToCreate[i]) {
			Block* pBlock;
			_blocks[i] = _blockPool.getObj(i, pBlock, true);
		}
	}

	CMeshPtr result = makeTris(false);

	return result;
}

CMeshPtr Volume::makeTris(bool cells)
{
	CBoundingBox3Dd bbox;
	bbox.merge(_originMeters);
	bbox.merge(_originMeters + _spanMeters);
	auto diagDist = bbox.range().norm();
	bbox.grow(diagDist * 0.05);

	vector<CMeshPtr> results;
	for (size_t i = 0; i < MultiCore::getNumCores(); i++) {
		results.push_back(make_shared<CMesh>());
		results.back()->reset(bbox);
	}

	Vector3d blockSpan;
	for (int i = 0; i < 3; i++)
		blockSpan[i] = _spanMeters[i] / _blockDim[i];

//	for (blockIdx[0] = 0; blockIdx[0] < _blockDim[0]; blockIdx[0]++) {
	MultiCore::runLambda([this, &blockSpan, &results, cells](size_t threadNum, size_t numThreads) {
		Vector3d blockOrigin;
		auto result = results[threadNum];

		Vector3i blockIdx;
		for (blockIdx[0] = threadNum; blockIdx[0] < _blockDim[0]; blockIdx[0] += numThreads) {
			blockOrigin[0] = _originMeters[0] + blockIdx[0] * blockSpan[0];
			for (blockIdx[1] = 0; blockIdx[1] < _blockDim[1]; blockIdx[1]++) {
				blockOrigin[1] = _originMeters[1] + blockIdx[1] * blockSpan[1];
				for (blockIdx[2] = 0; blockIdx[2] < _blockDim[2]; blockIdx[2]++) {
					blockOrigin[2] = _originMeters[2] + blockIdx[2] * blockSpan[2];
					Block* pBlock = getBlock(blockIdx);
					if (pBlock) {
						pBlock->addBlockTris(blockOrigin, blockSpan, result, cells);
					}
				}
			}
		}
	}, RUN_MULTI_THREAD);

	CMeshPtr result = make_shared<CMesh>();
	result->merge(results);
	cout << "Num tris: " << result->numTris();
	cout << "Num cells: " << result->numTris() / 12;
	return result;
}

void Block::addBlockTris(const Vector3d& blockOrigin, const Vector3d& blockSpan, TriMesh::CMeshPtr& pMesh, bool useCells)
{

	if (useCells && !_cells.empty()) {

	} else {
		vector<Vector3d> pts = {
			Vector3d(blockOrigin[0], blockOrigin[1], blockOrigin[2]),
			Vector3d(blockOrigin[0] + blockSpan[0], blockOrigin[1], blockOrigin[2]),
			Vector3d(blockOrigin[0] + blockSpan[0], blockOrigin[1] + blockSpan[1], blockOrigin[2]),
			Vector3d(blockOrigin[0], blockOrigin[1] + blockSpan[1], blockOrigin[2]),

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
