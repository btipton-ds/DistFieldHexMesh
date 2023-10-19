

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

inline void assignAxes(const Vector3i& axisOrder, Vector3i& indexOut)
{
	assignAxes(Vector3i(0, 1, 2), axisOrder, indexOut);
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

Cell* Volume::getCell(size_t ix, size_t iy, size_t iz)
{
	size_t blkDim = Block::getBlockDim();
	Vector3i idx(ix, iy, iz);
	Vector3i blockIdx, cellIdx;
	for (int i = 0; i < 3; i++) {
		blockIdx[i] = idx[i] / blkDim;
		cellIdx[i] = idx[i] % blkDim;
	}
	Block* pBlock = getBlock(blockIdx);
	if (pBlock) {
		Cell* pCell = pBlock->getCell(cellIdx);
		return pCell;
	}
	return nullptr;
}

const Cell* Volume::getCell(size_t ix, size_t iy, size_t iz) const
{
	size_t blkDim = Block::getBlockDim();
	Vector3i idx(ix, iy, iz);
	Vector3i blockIdx, cellIdx;
	for (int i = 0; i < 3; i++) {
		blockIdx[i] = idx[i] / blkDim;
		cellIdx[i] = idx[i] % blkDim;
	}
	const Block* pBlock = getBlock(blockIdx);
	if (pBlock) {
		const Cell* pCell = pBlock->getCell(cellIdx);
		return pCell;
	}
	return nullptr;
}

void Volume::createBlockRays(const TriMesh::CMeshPtr& pTriMesh, AxisIndex axisIdx, vector<bool>& blocksToCreate)
{
	const Vector3i axisOrder = Block::getAxisOrder(axisIdx);

	const Vector3d rayDir = axisIdx == AxisIndex::Z ? Vector3d(0, 0, 1) : (axisIdx == AxisIndex::Y ? Vector3d(0, 1, 0) : Vector3d(1, 0, 0));

	const size_t numI = _blockDim[axisOrder[0]] + 1;
	const size_t numJ = _blockDim[axisOrder[1]] + 1;

	MultiCore::runLambda([this, pTriMesh, axisOrder, rayDir, numI, numJ, &blocksToCreate](size_t threadNum, size_t numThreads) {
		const double TOL = 1.0e-6;
		const size_t bd = Block::getBlockDim();
		const size_t axis0 = axisOrder[0];
		const size_t axis1 = axisOrder[1];
		const size_t axis2 = axisOrder[2];

		Vector3d origin(_originMeters);

		Vector3d blockSpan, cellSpan;
		for (int i = 0; i < 3; i++) {
			blockSpan [i] = _spanMeters[i] / _blockDim[i];
			cellSpan[i] = blockSpan[i] / bd;
		}

		for (size_t i = 0; i < numI; i++) {
			origin[axis0] = _originMeters[axis0] + i * blockSpan[axis0];

			for (size_t j = 0; j < numJ; j++) {
				const size_t rayIdx = i + numI * j;
				if (rayIdx % numThreads != threadNum)
					continue;

				origin[axis1] = _originMeters[axis1] + j * blockSpan[axis1];

				Ray ray(origin, rayDir);
				vector<RayHit> hits;
				if (pTriMesh->rayCast(ray, hits)) {
					for (const auto& triHit : hits) {
						double dist0 = triHit.dist;
						if (dist0 < 0)
							dist0 = 0;
						else if (dist0 >= _spanMeters[axis2])
							dist0 = _spanMeters[axis2];

						double w0 = dist0 / _spanMeters[axis2];
							
						size_t blockIdx = (size_t)(w0 * _blockDim[axis2]);
						if (blockIdx >= _blockDim[axis2])
							blockIdx = _blockDim[axis2] - 1;
						double dist1 = dist0 - (blockIdx * blockSpan[axis2]);

						double w1 = dist1 / blockSpan[axis2];
						size_t cellIdx = (size_t)(w1 * Block::getBlockDim());
						if (cellIdx >= Block::getBlockDim())
							cellIdx = Block::getBlockDim() - 1;

						double dist2 = dist1 - (cellIdx * cellSpan[axis2]);

						size_t k = blockIdx;
						for (int di = -1; di < 2; di++) {
							size_t ii = i + di;
							if (ii >= _blockDim[axis0])
								continue;

							for (int dj = -1; dj < 2; dj++) {
								size_t jj = j + dj;
								if (jj >= _blockDim[axis1])
									continue;

								Vector3i idxIn(ii, jj, k), blockIndex;
								assignAxes(idxIn, axisOrder, blockIndex);
								size_t bIdx = calLinearBlockIndex(blockIndex);
								if (bIdx < blocksToCreate.size()) {
									blocksToCreate[bIdx] = true;
								} else
									cout << "block index out  of bounds\n";
							}
						}
					}
				}
			}
		}
	}, RUN_MULTI_THREAD);

}

void Volume::createBlockCellRays(const TriMesh::CMeshPtr& pTriMesh, AxisIndex axisIdx, const BlockColRec& blockColRec, std::vector<std::vector<bool>>& cellsToCreate)
{
	const Vector3i axisOrder = Block::getAxisOrder(axisIdx);
	const auto& blocksToTest = (axisIdx == AxisIndex::X) ? blockColRec._yzBlocks : (axisIdx == AxisIndex::Y) ? blockColRec._zxBlocks : blockColRec._xyBlocks;
	const Vector3d rayDir = axisIdx == AxisIndex::Z ? Vector3d(0, 0, 1) : (axisIdx == AxisIndex::Y ? Vector3d(0, 1, 0) : Vector3d(1, 0, 0));

	MultiCore::runLambda([this, pTriMesh, axisOrder, rayDir, &blocksToTest, &cellsToCreate](size_t threadNum, size_t numThreads) {
		const double TOL = 1.0e-6;
		const size_t bd = Block::getBlockDim();
		const size_t numBlockCells = bd * bd * bd;
		const size_t axis0 = axisOrder[0];
		const size_t axis1 = axisOrder[1];
		const size_t axis2 = axisOrder[2];

		const size_t numSteps = Block::getBlockDim() + 1;
		Vector3d blockOrigin(_originMeters);

		Vector3d blockSpan, cellSpan;
		for (int i = 0; i < 3; i++) {
			blockSpan[i] = _spanMeters[i] / _blockDim[i];
			cellSpan[i] = blockSpan[i] / bd;
		}

		for (size_t iBlk = 0; iBlk < _blockDim[axisOrder[0]]; iBlk++) {
			blockOrigin[axis0] = _originMeters[axis0] + iBlk * blockSpan[axis0];

			for (size_t jBlk = 0; jBlk < _blockDim[axisOrder[1]]; jBlk++) {
				const size_t blkIdx = iBlk + _blockDim[axisOrder[0]] * jBlk;
				if (blkIdx % numThreads != threadNum || !blocksToTest[blkIdx])
					continue;

				blockOrigin[axis1] = _originMeters[axis1] + jBlk * blockSpan[axis1];

				Vector3d cellOrigin;
				cellOrigin[axis2] = _originMeters[axis2];
				for (size_t iCell = 0; iCell < numSteps; iCell++) {
					cellOrigin[axis0] = blockOrigin[axis0] + iCell * cellSpan[axis0];

					for (size_t jCell = 0; jCell < numSteps; jCell++) {
						cellOrigin[axis1] = blockOrigin[axis1] + jCell * cellSpan[axis1];
						Ray ray(cellOrigin, rayDir);
						vector<RayHit> hits;
						if (pTriMesh->rayCast(ray, hits)) {
							for (const auto& triHit : hits) {
								double dist0 = triHit.dist;
								if (dist0 < 0)
									dist0 = 0;
								else if (dist0 >= _spanMeters[axis2])
									dist0 = _spanMeters[axis2];

								double w0 = dist0 / _spanMeters[axis2];

								size_t rayBlockIdx = (size_t)(w0 * _blockDim[axis2]);
								if (rayBlockIdx >= _blockDim[axis2])
									rayBlockIdx = _blockDim[axis2] - 1;
								double dist1 = dist0 - (rayBlockIdx * blockSpan[axis2]);

								double w1 = dist1 / blockSpan[axis2];
								size_t rayCellIdx = (size_t)(w1 * bd);
								if (rayCellIdx >= bd)
									rayCellIdx = bd - 1;

								double dist2 = dist1 - (rayCellIdx * cellSpan[axis2]);

								Vector3i blkIdxIn(iBlk, jBlk, rayBlockIdx), blockIndex;
								assignAxes(blkIdxIn, axisOrder, blockIndex);
								size_t bIdx = calLinearBlockIndex(blockIndex);

								vector<bool>& blockCellsToCreate = cellsToCreate[bIdx];
								if (blockCellsToCreate.empty())
									blockCellsToCreate.resize(numBlockCells);

								for (int di = -1; di < 2; di++) {
									size_t iiCell = iCell + di;
									if (iiCell >= bd)
										continue;

									for (int dj = -1; dj < 2; dj++) {
										size_t jjCell = jCell + dj;
										if (jjCell >= bd)
											continue;

										Vector3i cellIdxIn(iiCell, jjCell, rayCellIdx), cellIndex;
										assignAxes(cellIdxIn, axisOrder, cellIndex);
										size_t cIdx = Block::calcCellIndex(cellIndex);
										if (cIdx < blockCellsToCreate.size()) {
											blockCellsToCreate[cIdx] = true;
										}
										else
											cout << "block index out  of bounds\n";
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
void Volume::createCellRays(const TriMesh::CMeshPtr& pTriMesh, const Vector3i& axisOrder, const Vector3d& blockOrigin, const Vector3d& rayDir, const Vector3d& blockSpan,
	vector<bool>& cellsToCreate, vector<shared_ptr<RayBlockIntersectVec>>& cellHits)
{
	const size_t bd = Block::getBlockDim();
	const double TOL = 1.0e-6;
	const size_t axis0 = axisOrder[0];
	const size_t axis1 = axisOrder[1];
	const size_t axis2 = axisOrder[2];

	cellsToCreate.resize(bd * bd * bd);
	cellHits.resize((bd + 1) * (bd + 1));

	Vector3d origin(blockOrigin), cellSpan;
	for (int i = 0; i < 3; i++)
		cellSpan[i] = blockSpan[i] / bd;

	for (size_t i = 0; i <= bd; i++) {
		origin[axis0] = blockOrigin[axis0] + i * cellSpan[axis0];

		for (size_t j = 0; j < bd; j++) {
			const size_t rayIdx = i + bd * j;
			origin[axis1] = blockOrigin[axis1] + j * cellSpan[axis1];

			Ray ray(origin, rayDir);
			vector<RayHit> hits;
			if (pTriMesh->rayCast(ray, hits)) {
				if (!cellHits[rayIdx]) {
					cellHits[rayIdx] = make_shared<RayBlockIntersectVec>();
				}
				auto& blockHits = *cellHits[rayIdx];

				if (blockHits.empty())
					blockHits.resize(1);
				RayTriIntersectVec& rayHits = blockHits.front();

				for (const auto& triHit : hits) {
					double dist0 = triHit.dist;
					if (dist0 < 0)
						dist0 = 0;
					else if (dist0 >= _spanMeters[axis2])
						dist0 = _spanMeters[axis2];

					RayTriIntersect rti;
					rti._triIdx = triHit.triIdx;

					double w0 = dist0 / _spanMeters[axis2];

					rti._blockIdx = (size_t)(w0 * _blockDim[axis2]);
					if (rti._blockIdx >= _blockDim[axis2])
						rti._blockIdx = _blockDim[axis2] - 1;
					double dist1 = dist0 - (rti._blockIdx * blockSpan[axis2]);

					double w1 = dist1 / blockSpan[axis2];
					rti._cellIdx = (size_t)(w1 * bd);
					if (rti._cellIdx >= bd)
						rti._cellIdx = bd - 1;

					double dist2 = dist1 - (rti._cellIdx * cellSpan[axis2]);

					rti._w = dist2 / cellSpan[axis2];
					rayHits.push_back(rti);

					size_t k = rti._cellIdx;
					for (int di = -1; di < 2; di++) {
						size_t ii = i + di;
						if (ii >= bd)
							continue;

						for (int dj = -1; dj < 2; dj++) {
							size_t jj = j + dj;
							if (jj >= bd)
								continue;

							Vector3i idxIn(ii, jj, k), cellIndex;
							assignAxes(idxIn, axisOrder, cellIndex);
							size_t bIdx = Block::calcCellIndex(cellIndex);
							if (bIdx < cellsToCreate.size()) {
								cellsToCreate[bIdx] = true;
							}
							else
								cout << "Cell index out  of bounds\n";
						}
					}
				}
			}
		}
	}
}

CMeshPtr Volume::buildCFDHexes(const CMeshPtr& pTriMesh, double minCellSize, const Vector3d& emptyVolRatio)
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
	blocksToCreate.resize(_blockDim[0] * _blockDim[1] * _blockDim[2]);

	createBlockRays(pTriMesh, AxisIndex::X, blocksToCreate);
	createBlockRays(pTriMesh, AxisIndex::Y, blocksToCreate);
	createBlockRays(pTriMesh, AxisIndex::Z, blocksToCreate);

	BlockColRec blockColRec(_blockDim);

	MultiCore::runLambda([this, pTriMesh, &blocksToCreate, &blockColRec](size_t threadNum, size_t numThreads) {
		Vector3d blockSpan, blockOrigin;
		for (int i = 0; i < 3; i++) {
			blockSpan[i] = _spanMeters[i] / (double)_blockDim[i];
		}
		for (size_t i = threadNum; i < _blockDim[0]; i += numThreads) {
			blockOrigin[0] = _originMeters[0] + i * blockSpan[0];
			for (size_t j = 0; j < _blockDim[1]; j++) {
				blockOrigin[1] = _originMeters[1] + j * blockSpan[1];
				for (size_t k = 0; k < _blockDim[2]; k++) {
					blockOrigin[2] = _originMeters[2] + k * blockSpan[2];

					size_t bIdx = calLinearBlockIndex(i, j, k);
					if (blocksToCreate[bIdx]) {
						blockColRec.set(i, j, k);
					}
				}
			}
		}
	}, RUN_MULTI_THREAD); 

	vector<vector<bool>> cellsToCreate;
	cellsToCreate.resize(_blockDim[0] * _blockDim[1] * _blockDim[2]);
	createBlockCellRays(pTriMesh, AxisIndex::X, blockColRec, cellsToCreate);
	createBlockCellRays(pTriMesh, AxisIndex::Y, blockColRec, cellsToCreate);
	createBlockCellRays(pTriMesh, AxisIndex::Z, blockColRec, cellsToCreate);

	MultiCore::runLambda([this, pTriMesh, &cellsToCreate](size_t threadNum, size_t numThreads) {
		Vector3d blockSpan, blockOrigin;
		for (int i = 0; i < 3; i++) {
			blockSpan[i] = _spanMeters[i] / (double)_blockDim[i];
		}
		for (size_t i = threadNum; i < _blockDim[0]; i += numThreads) {
			blockOrigin[0] = _originMeters[0] + i * blockSpan[0];
			for (size_t j = 0; j < _blockDim[1]; j++) {
				blockOrigin[1] = _originMeters[1] + j * blockSpan[1];
				for (size_t k = 0; k < _blockDim[2]; k++) {
					blockOrigin[2] = _originMeters[2] + k * blockSpan[2];

					size_t bIdx = calLinearBlockIndex(i, j, k);
					if (!cellsToCreate[bIdx].empty()) {
						Block* pBlock = nullptr;
						_blocks[bIdx] = _blockPool.getObj(bIdx, pBlock, true);
						pBlock->createCells(cellsToCreate[bIdx]);
					}
				}
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


Volume::BlockColRec::BlockColRec(const Vector3i& blockDim)
	: _blockDim(blockDim)
{
	_xyBlocks.resize(_blockDim[0] * _blockDim[1]);
	_yzBlocks.resize(_blockDim[1] * _blockDim[2]);
	_zxBlocks.resize(_blockDim[2] * _blockDim[0]);

}

inline void Volume::BlockColRec::set(size_t i, size_t j, size_t k)
{
	_xyBlocks[i + _blockDim[0] * j] = true;
	_yzBlocks[j + _blockDim[1] * k] = true;
	_zxBlocks[k + _blockDim[2] * i] = true;
}



