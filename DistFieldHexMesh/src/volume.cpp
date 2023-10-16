

#include <triMesh.h>

#include <cell.h>
#include <block.h>
#include <volume.h>
#include <MultiCoreUtil.h>

using namespace std;
using namespace DFHM;
using namespace TriMesh;

#define RUN_MULTI_THREAD false

namespace
{
inline void assignAxes(const Vector3i& indexIn, const Vector3i& axisOrder, Vector3i& indexOut)
{
	indexOut[axisOrder[0]] = indexIn[0];
	indexOut[axisOrder[1]] = indexIn[1];
	indexOut[axisOrder[2]] = indexIn[2];
}
}

Volume::Volume(const Index3& blockSize)
{
	setBlockDims(blockSize);
}

Volume::Volume(const Volume& src)
	: DataPool(src)
	, _originMeters(src._originMeters) 
	, _spanMeters(src._spanMeters)
	, _blockDim(src._blockDim)
	, _blocks(src._blocks)
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

void Volume::scanVolumePlaneCreateBlocksWhereNeeded(const CMeshPtr& pTriMesh, std::vector<bool>& blocksToCreate, const Vector3i& axisOrder)
{
	if (blocksToCreate.empty())
		blocksToCreate.resize(_blocks.size());
	MultiCore::runLambda([this, pTriMesh, &blocksToCreate, axisOrder](size_t threadNum, size_t numThreads) {
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
				if (pTriMesh->rayCast(ray, hits, false)) {
					for (const auto& hit : hits) {

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
								case 0: {
									ix = k; 

									for (int iiy = -1; iiy <= 1; iiy++) {
										if (iy + iiy >= _blockDim[1])
											continue;
										for (int iiz = -1; iiz <= 1; iiz++) {
											if (iz + iiz >= _blockDim[2])
												continue;

											size_t bIdx = calLinearBlockIndex(ix, iy + iiy, iz + iiz);
											// No mutex required, because order of setting true is not a race condition.
											if (bIdx != -1)
												blocksToCreate[bIdx] = true;
										}
									}

									break;
								}
								case 1: {
									iy = k;

									for (int iix = -1; iix <= 1; iix++) {
										if (ix + iix >= _blockDim[0])
											continue;
										for (int iiz = -1; iiz <= 1; iiz++) {
											if (iz + iiz >= _blockDim[2])
												continue;

											size_t bIdx = calLinearBlockIndex(ix + iix, iy, iz + iiz);
											// No mutex required, because order of setting true is not a race condition.
											if (bIdx != -1)
												blocksToCreate[bIdx] = true;
										}
									}

									break;
								}
								case 2: {
									iz = k;

									for (int iix = -1; iix <= 1; iix++) {
										if (ix + iix >= _blockDim[0])
											continue;
										for (int iiy = -1; iiy <= 1; iiy++) {
											if (iy + iiy >= _blockDim[1])
												continue;

											size_t bIdx = calLinearBlockIndex(ix + iix, iy + iiy, iz);
											// No mutex required, because order of setting true is not a race condition.
											if (bIdx != -1)
												blocksToCreate[bIdx] = true;
										}
									}
									break;
								}
							}						
						}
					}
				}
			}
		}
	}, RUN_MULTI_THREAD);

}

void Volume::scanVolumePlaneCreateCellsWhereNeeded(const TriMesh::CMeshPtr& pTriMesh, const Vector3i& axisOrder)
{
	MultiCore::runLambda([this, pTriMesh, axisOrder](size_t threadNum, size_t numThreads) {
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

				scanBlockColumn(pTriMesh, i, j, blockRange, axisOrder);
			}
		}
	}, RUN_MULTI_THREAD);
}

void Volume::scanBlockColumn(const TriMesh::CMeshPtr& pTriMesh, size_t i, size_t j, const Vector3d& blockRange, const Vector3i& axisOrder)
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

	bool needScan = false;

	switch (axisOrder[2]) {
		case 0: {
			for (size_t ix = 0; ix < _blockDim[axisOrder[0]]; ix++) {
				size_t bIdx = calLinearBlockIndex(ix, iy, iz);
				if (_blocks[bIdx]) {
					needScan = true;
					break;
				}
			}
			
			break;
		}
		case 1: {
			for (size_t iy = 0; iy < _blockDim[axisOrder[1]]; iy++) {
				size_t bIdx = calLinearBlockIndex(ix, iy, iz);
				if (_blocks[bIdx]) {
					needScan = true;
					break;
				}
			}
			break;
		}
		case 2: {
			for (size_t iz = 0; iz < _blockDim[axisOrder[2]]; iz++) {
				size_t bIdx = calLinearBlockIndex(ix, iy, iz);
				if (_blocks[bIdx]) {
					needScan = true;
					break;
				}
			}

			break;
		}
	}


	if (!needScan)
		return;

	for (int ii = 0; ii < _blockDim[axisOrder[0]]; ii++) {
		for (int jj = 0; jj < _blockDim[axisOrder[1]]; jj++) {
			size_t iix, iiy, iiz;
			switch (axisOrder[0]) {
				case 0: iix = ii; break;
				case 1: iiy = ii; break;
				case 2: iiz = ii; break;
			}

			switch (axisOrder[1]) {
				case 0: iix = jj; break;
				case 1: iiy = jj; break;
				case 2: iiz = jj; break;
			}
		}
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

	MultiCore::runLambda([this, pTriMesh, &blocksToCreate](size_t threadNum, size_t numThreads) {
		Vector3 blockSpan;
		for (int i = 0; i < 3; i++) {
			blockSpan[i] = _spanMeters[i] / (double)_blockDim[i];
		}
		for (size_t i = threadNum; i < blocksToCreate.size(); i += numThreads) {
			if (blocksToCreate[i]) {
				_blocks[i] = _blockPool.create();
#if 0
				Block* pBlock;
				size_t ix = i % _blockDim[0];
				size_t tempI = i / _blockDim[0];
				size_t iy = tempI % _blockDim[1];
				tempI = tempI / _blockDim[1];
				size_t iz = tempI % _blockDim[2];

				Vector3 blockOrigin(_originMeters);
				blockOrigin += Vector3(ix * blockSpan[0], iy * blockSpan[1], iz * blockSpan[2]);

				vector<bool> cellsToCreate;
				size_t bd = Block::getBlockDim();
				cellsToCreate.resize(bd * bd * bd);

				bool changed = pBlock->scanCreateCellsWhereNeeded(pTriMesh, blockOrigin, blockSpan, cellsToCreate, Vector3i(0, 1, 2));
				changed = pBlock->scanCreateCellsWhereNeeded(pTriMesh, blockOrigin, blockSpan, cellsToCreate, Vector3i(2, 0, 1)) || changed;
				changed = pBlock->scanCreateCellsWhereNeeded(pTriMesh, blockOrigin, blockSpan, cellsToCreate, Vector3i(1, 2, 0)) || changed;

				if (!changed) {
					_blockPool.free(_blocks[i]);
					_blocks[i] = -1;
				} else {
					pBlock->createCells(cellsToCreate);
				}
#endif
			}
		}
	}, RUN_MULTI_THREAD);

	CMeshPtr result = makeTris(true);

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

