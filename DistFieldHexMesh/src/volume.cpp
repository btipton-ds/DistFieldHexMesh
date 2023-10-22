
#include <sstream>
#include <fstream>

#include <triMesh.h>

#include <cell.h>
#include <block.h>
#include <volume.h>
#include <vertex.h>
#include <MultiCoreUtil.h>

using namespace std;
using namespace DFHM;
using namespace TriMesh;

#define RUN_MULTI_THREAD true

namespace
{

inline Vector3i assignAxes(const Vector3i& indexIn, const Vector3i& axisOrder)
{
	Vector3i result;
	result[axisOrder[0]] = indexIn[0];
	result[axisOrder[1]] = indexIn[1];
	result[axisOrder[2]] = indexIn[2];
	return result;
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

void Volume::processRayHit(const RayHit& triHit, AxisIndex rayAxis, const Vector3d& blockSpan, const Vector3d& cellSpan, size_t& blockIdx, size_t& cellIdx)
{
	RayTriIntersectVec blockHits;
	processRayHit(triHit, rayAxis, -1, -1, blockSpan, cellSpan, blockHits, blockIdx, cellIdx);
}

void Volume::processRayHit(const RayHit& triHit, AxisIndex rayAxis, size_t i, size_t j, const Vector3d& blockSpan, const Vector3d& cellSpan, RayTriIntersectVec& blockHits, size_t& blockIdx, size_t& cellIdx)
{

	int rayAxisIdx = (int)rayAxis;

	double dist0 = triHit.dist;
	if (dist0 < 0)
		dist0 = 0;
	else if (dist0 >= _spanMeters[rayAxisIdx])
		dist0 = _spanMeters[rayAxisIdx];

	double w0 = dist0 / _spanMeters[rayAxisIdx];

	blockIdx = (size_t)(w0 * _blockDim[rayAxisIdx]);
	if (blockIdx >= _blockDim[rayAxisIdx])
		blockIdx = _blockDim[rayAxisIdx] - 1;
	double dist1 = dist0 - (blockIdx * blockSpan[rayAxisIdx]);

	double w1 = dist1 / blockSpan[rayAxisIdx];
	cellIdx = (size_t)(w1 * Block::getBlockDim());
	if (cellIdx >= Block::getBlockDim())
		cellIdx = Block::getBlockDim() - 1;

	double dist2 = dist1 - (cellIdx * cellSpan[rayAxisIdx]);
	double w2 = dist2 / cellSpan[rayAxisIdx];

	RayTriIntersect rti;
	rti._blockIdx = blockIdx;
	rti._triIdx = triHit.triIdx;
	rti._hitPt = triHit.hitPt;
	switch (rayAxis) {
		case AxisIndex::X :
			rti._cellIdx = Vector3i(cellIdx, i, j);
			break;
		case AxisIndex::Y:
			rti._cellIdx = Vector3i(i, cellIdx, j);
			break;
		case AxisIndex::Z:
			rti._cellIdx = Vector3i(i, j, cellIdx);
			break;
	}

	blockHits.push_back(rti);
}

void Volume::createBlockRays(AxisIndex axisIdx, const TriMesh::CMeshPtr& pTriMesh, vector<bool>& blocksToCreate)
{
	const size_t bd = Block::getBlockDim();
	Vector3d blockSpan, cellSpan;
	for (int i = 0; i < 3; i++) {
		blockSpan[i] = _spanMeters[i] / _blockDim[i];
		cellSpan[i] = blockSpan[i] / bd;
	}

	switch (axisIdx) {
		case AxisIndex::X: {
			MultiCore::runLambda([this, pTriMesh, blockSpan, cellSpan, &blocksToCreate](size_t threadNum, size_t numThreads) {
				const Vector3d rayDir(1, 0, 0);
				const size_t numY = _blockDim[1] + 1;
				const size_t numZ = _blockDim[2] + 1;

				Vector3d origin(_originMeters);

				for (size_t iy = 0; iy < numY; iy++) {
					origin[1] = _originMeters[1] + iy * blockSpan[1];

					for (size_t iz = 0; iz < numZ; iz++) {
						const size_t rayIdx = iy + numY * iz;
						if (rayIdx % numThreads != threadNum)
							continue;

						origin[2] = _originMeters[2] + iz * blockSpan[2];

						Ray ray(origin, rayDir);
						vector<RayHit> hits;
						if (pTriMesh->rayCast(ray, hits)) {
							for (const auto& triHit : hits) {
								size_t blockIdx, cellIdx;
								processRayHit(triHit, AxisIndex::X, blockSpan, cellSpan, blockIdx, cellIdx);
								size_t ix = blockIdx;
								for (int dy = -1; dy <= 0; dy++) {
									for (int dz = -1; dz <= 0; dz++) {
										size_t bIdx = calLinearBlockIndex(Vector3i(ix, iy + dy, iz + dz));
										if (bIdx < blocksToCreate.size())
											blocksToCreate[bIdx] = true;
									}
								}
							}
						}
					}
				}
			}, RUN_MULTI_THREAD);
			break;
		}
		case AxisIndex::Y: {
			MultiCore::runLambda([this, pTriMesh, blockSpan, cellSpan, &blocksToCreate](size_t threadNum, size_t numThreads) {
				Vector3d rayDir(0, 1, 0);
				const size_t numX = _blockDim[0] + 1;
				const size_t numZ = _blockDim[2] + 1;

				Vector3d origin(_originMeters);

				for (size_t ix = 0; ix < numX; ix++) {
					origin[0] = _originMeters[0] + ix * blockSpan[0];

					for (size_t iz = 0; iz < numZ; iz++) {
						const size_t rayIdx = ix + numX * iz;
						if (rayIdx % numThreads != threadNum)
							continue;

						origin[2] = _originMeters[2] + iz * blockSpan[2];

						Ray ray(origin, rayDir);
						vector<RayHit> hits;
						if (pTriMesh->rayCast(ray, hits)) {
							for (const auto& triHit : hits) {
								size_t blockIdx, cellIdx;
								processRayHit(triHit, AxisIndex::Y, blockSpan, cellSpan, blockIdx, cellIdx);
								size_t iy = blockIdx;
								for (int dx = -1; dx <= 0; dx++) {
									for (int dz = -1; dz <= 0; dz++) {
										size_t bIdx = calLinearBlockIndex(Vector3i(ix + dx, iy, iz + dz));
										if (bIdx < blocksToCreate.size())
											blocksToCreate[bIdx] = true;
									}
								}
							}
						}
					}
				}
			}, RUN_MULTI_THREAD);
			break;
		}
		case AxisIndex::Z: {
			MultiCore::runLambda([this, pTriMesh, blockSpan, cellSpan, &blocksToCreate](size_t threadNum, size_t numThreads) {
				Vector3d rayDir(0, 0, 1);
				const size_t numX = _blockDim[0] + 1;
				const size_t numY = _blockDim[1] + 1;

				Vector3d origin(_originMeters);

				for (size_t ix = 0; ix < numX; ix++) {
					origin[0] = _originMeters[0] + ix * blockSpan[0];

					for (size_t iy = 0; iy < numY; iy++) {
						const size_t rayIdx = ix + numX * iy;
						if (rayIdx % numThreads != threadNum)
							continue;

						origin[1] = _originMeters[1] + iy * blockSpan[1];

						Ray ray(origin, rayDir);
						vector<RayHit> hits;
						if (pTriMesh->rayCast(ray, hits)) {
							for (const auto& triHit : hits) {
								size_t blockIdx, cellIdx;
								processRayHit(triHit, AxisIndex::Z, blockSpan, cellSpan, blockIdx, cellIdx);
								size_t iz = blockIdx;
								for (int dx = -1; dx <= 0; dx++) {
									for (int dy = -1; dy <= 0; dy++) {
										size_t bIdx = calLinearBlockIndex(Vector3i(ix + dx, iy + dy, iz));
										if (bIdx < blocksToCreate.size())
											blocksToCreate[bIdx] = true;
									}
								}
							}
						}
					}
				}
			}, RUN_MULTI_THREAD);
			break;
		}
	}
}

void Volume::createBlockCellRays(AxisIndex axisIdx, const TriMesh::CMeshPtr& pTriMesh, const vector<bool>& blocksToCreate, RayHitRec& inter, vector<vector<bool>>& cellsToCreate)
{
	const size_t bd = Block::getBlockDim();
	Vector3d blockSpan, cellSpan;
	for (int i = 0; i < 3; i++) {
		blockSpan[i] = _spanMeters[i] / _blockDim[i];
		cellSpan[i] = blockSpan[i] / bd;
	}

	switch (axisIdx) {
		case AxisIndex::X: {
			auto& interX = inter._xIntersects;
			interX.resize(_blockDim[1] * _blockDim[2]);
			MultiCore::runLambda([this, pTriMesh, blockSpan, cellSpan, &blocksToCreate, &interX, &cellsToCreate](size_t threadNum, size_t numThreads) {
				const Vector3d rayDir(1, 0, 0);
				const size_t bd = Block::getBlockDim();
				const size_t numBlockCells = bd * bd * bd;

				const size_t numSteps = bd + 1;
				Vector3d blockOrigin(_originMeters);

				for (size_t iyBlk = 0; iyBlk < _blockDim[1]; iyBlk++) {
					blockOrigin[1] = _originMeters[1] + iyBlk * blockSpan[1];

					for (size_t izBlk = 0; izBlk < _blockDim[2]; izBlk++) {
						size_t blockIdx = iyBlk + izBlk * _blockDim[1];
						if (blockIdx % numThreads != threadNum)
							continue;

						bool skipBlock = true;
						for (size_t ixBlk = 0; ixBlk < _blockDim[0]; ixBlk++) {
							size_t bIdx = calLinearBlockIndex(ixBlk, iyBlk, izBlk);
							if (blocksToCreate[bIdx]) {
								skipBlock = false;
								break;
							}
						}
						if (skipBlock)
							continue;

						blockOrigin[2] = _originMeters[2] + izBlk * blockSpan[2];

						Vector3d rayOrigin(blockOrigin);
						for (size_t iyCell = 0; iyCell < numSteps; iyCell++) {
							rayOrigin[1] = blockOrigin[1] + iyCell * cellSpan[1];

							for (size_t izCell = 0; izCell < numSteps; izCell++) {
								rayOrigin[2] = blockOrigin[2] + izCell * cellSpan[2];

								Ray ray(rayOrigin, rayDir);
								vector<RayHit> hits;
								if (pTriMesh->rayCast(ray, hits)) {
									auto pBlockInter = make_shared<RayTriIntersectVec>();
									interX[blockIdx] = pBlockInter;
									for (const auto& triHit : hits) {
										size_t blkIdx, cellIdx;
										processRayHit(triHit, AxisIndex::X, iyCell, izCell, blockSpan, cellSpan, *pBlockInter, blkIdx, cellIdx);

										size_t bIdx = calLinearBlockIndex(blkIdx, iyBlk, izBlk);
										if (bIdx < cellsToCreate.size()) {
											vector<bool>& blockCellsToCreate = cellsToCreate[bIdx];
											if (blockCellsToCreate.empty())
												blockCellsToCreate.resize(numBlockCells);

											for (int dy = -1; dy <= 0; dy++) {
												for (int dz = -1; dz <= 0; dz++) {
													size_t cIdx = Block::calcLinearCellIndex(Vector3i(cellIdx, iyCell + dy, izCell + dz));
													if (cIdx < blockCellsToCreate.size()) {
														blockCellsToCreate[cIdx] = true;
													}
												}
											}
										}
									}
								}
							}
						}
					}
				}
			}, RUN_MULTI_THREAD);
			break;
		}
		case AxisIndex::Y: {
			auto& interY = inter._yIntersects;
			interY.resize(_blockDim[0] * _blockDim[2]);
			MultiCore::runLambda([this, pTriMesh, blockSpan, cellSpan, &blocksToCreate, &interY, &cellsToCreate](size_t threadNum, size_t numThreads) {
				const Vector3d rayDir(0, 1, 0);
				const size_t bd = Block::getBlockDim();
				const size_t numBlockCells = bd * bd * bd;

				const size_t numSteps = bd + 1;
				Vector3d blockOrigin(_originMeters);

				for (size_t ixBlk = 0; ixBlk < _blockDim[0]; ixBlk++) {
					blockOrigin[0] = _originMeters[0] + ixBlk * blockSpan[0];

					for (size_t izBlk = 0; izBlk < _blockDim[2]; izBlk++) {
						size_t blockIdx = ixBlk + izBlk * _blockDim[0];
						if (blockIdx % numThreads != threadNum)
							continue;

						bool skipBlock = true;
						for (size_t iyBlk = 0; iyBlk < _blockDim[1]; iyBlk++) {
							size_t bIdx = calLinearBlockIndex(ixBlk, iyBlk, izBlk);
							if (blocksToCreate[bIdx]) {
								skipBlock = false;
								break;
							}
						}
						if (skipBlock)
							continue;

						blockOrigin[2] = _originMeters[2] + izBlk * blockSpan[2];

						Vector3d rayOrigin(blockOrigin);
						for (size_t ixCell = 0; ixCell < numSteps; ixCell++) {
							rayOrigin[0] = blockOrigin[0] + ixCell * cellSpan[0];

							for (size_t izCell = 0; izCell < numSteps; izCell++) {
								rayOrigin[2] = blockOrigin[2] + izCell * cellSpan[2];

								Ray ray(rayOrigin, rayDir);
								vector<RayHit> hits;
								if (pTriMesh->rayCast(ray, hits)) {
									auto pBlockInter = make_shared<RayTriIntersectVec>();
									interY[blockIdx] = pBlockInter;
									for (const auto& triHit : hits) {
										size_t blkIdx, cellIdx;
										processRayHit(triHit, AxisIndex::Y, ixCell, izCell, blockSpan, cellSpan, *pBlockInter, blkIdx, cellIdx);

										size_t bIdx = calLinearBlockIndex(ixBlk, blkIdx, izBlk);
										if (bIdx < cellsToCreate.size()) {
											vector<bool>& blockCellsToCreate = cellsToCreate[bIdx];
											if (blockCellsToCreate.empty())
												blockCellsToCreate.resize(numBlockCells);

											for (int dx = -1; dx <= 0; dx++) {
												for (int dz = -1; dz <= 0; dz++) {
													size_t cIdx = Block::calcLinearCellIndex(Vector3i(ixCell + dx, cellIdx, izCell + dz));
													if (cIdx < blockCellsToCreate.size()) {
														blockCellsToCreate[cIdx] = true;
													}
												}
											}
										}
									}
								}
							}
						}
					}
				}
			}, RUN_MULTI_THREAD);
			break;
		}
		case AxisIndex::Z: {
			auto& interZ = inter._zIntersects;
			interZ.resize(_blockDim[0] * _blockDim[1]);
			MultiCore::runLambda([this, pTriMesh, blockSpan, cellSpan, &blocksToCreate, &interZ, &cellsToCreate](size_t threadNum, size_t numThreads) {
				const Vector3d rayDir(0, 0, 1);
				const size_t bd = Block::getBlockDim();
				const size_t numBlockCells = bd * bd * bd;

				const size_t numSteps = bd + 1;
				Vector3d blockOrigin(_originMeters);

				for (size_t ixBlk = 0; ixBlk < _blockDim[0]; ixBlk++) {
					blockOrigin[0] = _originMeters[0] + ixBlk * blockSpan[0];

					for (size_t iyBlk = 0; iyBlk < _blockDim[1]; iyBlk++) {
						size_t blockIdx = ixBlk + iyBlk * _blockDim[0];
						if (blockIdx % numThreads != threadNum)
							continue;

						bool skipBlock = true;
						for (size_t izBlk = 0; izBlk < _blockDim[2]; izBlk++) {
							size_t bIdx = calLinearBlockIndex(ixBlk, iyBlk, izBlk);
							if (blocksToCreate[bIdx]) {
								skipBlock = false;
								break;
							}
						}
						if (skipBlock)
							continue;

						blockOrigin[1] = _originMeters[1] + iyBlk * blockSpan[1];

						Vector3d rayOrigin(blockOrigin);
						for (size_t ixCell = 0; ixCell < numSteps; ixCell++) {
							rayOrigin[0] = blockOrigin[0] + ixCell * cellSpan[0];

							for (size_t iyCell = 0; iyCell < numSteps; iyCell++) {
								rayOrigin[1] = blockOrigin[1] + iyCell * cellSpan[1];

								Ray ray(rayOrigin, rayDir);
								vector<RayHit> hits;
								if (pTriMesh->rayCast(ray, hits)) {
									auto pBlockInter = make_shared<RayTriIntersectVec>();
									interZ[blockIdx] = pBlockInter;
									for (const auto& triHit : hits) {
										size_t blkIdx, cellIdx;
										processRayHit(triHit, AxisIndex::Z, ixCell, iyCell, blockSpan, cellSpan, *pBlockInter, blkIdx, cellIdx);

										size_t bIdx = calLinearBlockIndex(ixBlk, iyBlk, blkIdx);
										if (bIdx < cellsToCreate.size()) {
											vector<bool>& blockCellsToCreate = cellsToCreate[bIdx];
											if (blockCellsToCreate.empty())
												blockCellsToCreate.resize(numBlockCells);

											for (int dx = -1; dx <= 0; dx++) {
												for (int dy = -1; dy <= 0; dy++) {
													size_t cIdx = Block::calcLinearCellIndex(Vector3i(ixCell + dx, iyCell + dy, cellIdx));
													if (cIdx < blockCellsToCreate.size()) {
														blockCellsToCreate[cIdx] = true;
													}
												}
											}
										}
									}
								}
							}
						}
					}
				}
			}, RUN_MULTI_THREAD);
			break;
		}
	}
}

void Volume::buildCFDHexes(const CMeshPtr& pTriMesh, double minCellSize, const Vector3d& emptyVolRatio)
{
	CMesh::BoundingBox bb = pTriMesh->getBBox();
	_originMeters = bb.getMin();
	_spanMeters = bb.range();

	Index3 blockDims(
		(size_t)(_spanMeters[0] / minCellSize / Block::getBlockDim() + 0.5),
		(size_t)(_spanMeters[1] / minCellSize / Block::getBlockDim() + 0.5),
		(size_t)(_spanMeters[2] / minCellSize / Block::getBlockDim() + 0.5)
	);

	setBlockDims(blockDims);

	vector<bool> blocksToCreate;
	blocksToCreate.resize(_blockDim[0] * _blockDim[1] * _blockDim[2]);

	cout << "createBlockRays X\n";
	createBlockRays(AxisIndex::X, pTriMesh, blocksToCreate);
	cout << "createBlockRays Y\n";
	createBlockRays(AxisIndex::Y, pTriMesh, blocksToCreate);
	cout << "createBlockRays Z\n";
	createBlockRays(AxisIndex::Z, pTriMesh, blocksToCreate);

	vector<vector<bool>> cellsToCreate;
	cellsToCreate.resize(_blockDim[0] * _blockDim[1] * _blockDim[2]);

	RayHitRec rayHits;
	cout << "createBlockCellRays X\n";
	createBlockCellRays(AxisIndex::X, pTriMesh, blocksToCreate, rayHits, cellsToCreate);
	cout << "createBlockCellRays Y\n";
	createBlockCellRays(AxisIndex::Y, pTriMesh, blocksToCreate, rayHits, cellsToCreate);
	cout << "createBlockCellRays Z\n";
	createBlockCellRays(AxisIndex::Z, pTriMesh, blocksToCreate, rayHits, cellsToCreate);

	cout << "Creating cells\n";

	vector<size_t> blocksToProcess;
	for (size_t bIdx = 0; bIdx < cellsToCreate.size(); bIdx++) {
		if (!cellsToCreate[bIdx].empty())
			blocksToProcess.push_back(bIdx);

	}
	cout << "Num blocks " << (_blockDim[0] * _blockDim[1] * _blockDim[2]) << "\n";
	cout << "Num blocks to process " << blocksToProcess.size() << "\n";

	MultiCore::runLambda([this, pTriMesh, &blocksToProcess, &rayHits, &cellsToCreate](size_t threadNum, size_t numThreads) {
		cout << "Thread num " << threadNum << " starting.\n";
		Vector3d blockSpan;
		for (int i = 0; i < 3; i++)
			blockSpan[i] = _spanMeters[i] / _blockDim[i];

		for (size_t bpIdx = threadNum; bpIdx < blocksToProcess.size(); bpIdx += numThreads) {
			size_t bIdx = blocksToProcess[bpIdx];
			if (threadNum == 0)
				cout << "Progress: " << (100 * bpIdx / (blocksToProcess.size() - 1.0)) << "%\n";
			if (!cellsToCreate[bIdx].empty()) {
				Vector3i blockIndex = calCartesianBlockIndex(bIdx);
				Block* pBlock = getBlock(blockIndex, true);
				if (pBlock) {
					Vector3d blockOrigin;
					for (int i = 0; i < 3; i++)
						blockOrigin[i] = _originMeters[i] + blockIndex[i] * blockSpan[i];
					pBlock->createCells(blockOrigin, blockSpan, _blockDim, blockIndex, cellsToCreate[bIdx], rayHits);
				}
			}
		}
		cout << "Thread num " << threadNum << " complete.\n";
	}, RUN_MULTI_THREAD);
#if 0
	cout << "Intersecting cells\n";
	MultiCore::runLambda([this, &blocksToProcess, pTriMesh](size_t threadNum, size_t numThreads) {
		Vector3d blockSpan;
		for (int i = 0; i < 3; i++)
			blockSpan[i] = _spanMeters[i] / _blockDim[i];

		for (size_t bpIdx = threadNum; bpIdx < blocksToProcess.size(); bpIdx += numThreads) {
			size_t bIdx = blocksToProcess[bpIdx];
			if (threadNum == 0)
				cout << "Progress: " << (100 * bpIdx / (blocksToProcess.size() - 1.0)) << "%\n";
			Vector3i blockIndex = calCartesianBlockIndex(bIdx);

			Block* pBlock = getBlock(blockIndex);
			if (pBlock) {
				Vector3d blockOrigin;
				for (int i = 0; i < 3; i++)
					blockOrigin[i] = _originMeters[i] + blockIndex[i] * blockSpan[i];
				pBlock->createIntersectionCells(*this, blockOrigin, blockSpan);
			}
		}
	}, RUN_MULTI_THREAD);
#endif
	cout << "buildCFDHexes finished\n";
}

CMeshPtr Volume::makeBlockTris(bool cells)
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

		for (size_t ix = threadNum; ix < _blockDim[0]; ix += numThreads) {
			blockOrigin[0] = _originMeters[0] + ix * blockSpan[0];
			for (size_t iy = 0; iy < _blockDim[1]; iy++) {
				blockOrigin[1] = _originMeters[1] + iy * blockSpan[1];
				for (size_t iz = 0; iz < _blockDim[2]; iz++) {
					blockOrigin[2] = _originMeters[2] + iz * blockSpan[2];
					Block* pBlock = getBlock(ix, iy, iz);
					if (pBlock) {
						pBlock->addBlockTris(blockOrigin, blockSpan, result, cells);
					}
				}
			}
		}
	}, RUN_MULTI_THREAD);

	CMeshPtr result = results.back();
	results.pop_back();
	result->merge(results, true);
	cout << "Num tris: " << result->numTris();
	cout << "Num cells: " << result->numTris() / 12;
	return result;
}

void Volume::dumpSections(const string& dirName) const
{
	const size_t bd = Block::getBlockDim();
	size_t numX = _blockDim[0] * bd;
	size_t numY = _blockDim[1] * bd;
	size_t numZ = _blockDim[2] * bd;

	for (size_t iz = 0; iz < numZ; iz++) {
		stringstream ss;
		ss << dirName << "/dump" << to_string(iz) << ".txt";
		ofstream o(ss.str());
		for (size_t iy = 0; iy < numY; iy++) {
			if (iy % bd == (bd - 1)) {
				for (size_t ix = 0; ix < numX; ix++)
					o << "_";
				o << "\n";
			}

			for (size_t ix = 0; ix < numX; ix++) {
				if (ix % bd == (bd - 1))
					o << "|";
				if (getCell(ix, iy, iz))
					o << "X";
				else
					o << " ";
			}
			o << "\n";
		}
	}
}
size_t Volume::getVertIdx(const Vertex& vert) const
{
	return getVertIdx(vert.getPoint());
}

size_t Volume::getVertIdx(const Vector3d& pt) const
{
	CBoundingBox3Dd bbox(pt);
	vector<size_t> indices;
	if (_pVertFinder && _pVertFinder->find(bbox, indices)) {
		if (indices.size() == 1)
			return indices.front();
		else
			throw "should never get multiple hits";
	}

	return -1;
}

size_t Volume::addVert(const Vertex& vert)
{
	CBoundingBox3Dd bbox(vert.getPoint());
	vector<size_t> indices;
	if (!_pVertFinder) {
		CBoundingBox3Dd volBox;
		volBox.merge(_originMeters);
		volBox.merge(_originMeters + _spanMeters);
		volBox.grow(_spanMeters.norm() * 0.1);
		_pVertFinder = make_shared<CSpatialSearchSTd>(volBox);
	}
	Vertex* pVert;
	return _vertexPool.create(pVert, -1, vert);
}

