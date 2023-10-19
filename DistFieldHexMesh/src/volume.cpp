
#include <sstream>
#include <fstream>

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

								Vector3i blockIndex = assignAxes(Vector3i(ii, jj, k), axisOrder);
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

bool Volume::skipBlock(const std::vector<bool>& blocksToCreate, size_t iBlk, size_t jBlk, const Vector3i& axisOrder, size_t threadNum, size_t numThreads) const
{
	if ((iBlk + _blockDim[axisOrder[0]] * jBlk) % numThreads == threadNum) {
		// See if any block in the cast direction is occupied. If not skip this cast.
		for (size_t kBlk = 0; kBlk < _blockDim[axisOrder[2]]; kBlk++) {
			Vector3i blockIdx = assignAxes(Vector3i(iBlk, jBlk, kBlk), axisOrder);
			size_t blkIdx = calLinearBlockIndex(blockIdx);
			if (blocksToCreate[blkIdx]) {
				return false;
			}
		}
	}
	return true;
}

void Volume::createBlockCellRays(const TriMesh::CMeshPtr& pTriMesh, AxisIndex axisIdx, const vector<bool>& blocksToCreate, vector<vector<bool>>& cellsToCreate)
{
	mutex cellsToCreateLock;
	const Vector3i axisOrder = Block::getAxisOrder(axisIdx);
	const Vector3d rayDir = axisIdx == AxisIndex::Z ? Vector3d(0, 0, 1) : (axisIdx == AxisIndex::Y ? Vector3d(0, 1, 0) : Vector3d(1, 0, 0));

	MultiCore::runLambda([this, pTriMesh, axisOrder, rayDir, &cellsToCreateLock, &blocksToCreate, &cellsToCreate](size_t threadNum, size_t numThreads) {
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
				if (skipBlock(blocksToCreate, iBlk, jBlk, axisOrder, threadNum, numThreads)) continue;
				blockOrigin[axis1] = _originMeters[axis1] + jBlk * blockSpan[axis1];

				Vector3d rayOrigin;
				rayOrigin[axis2] = blockOrigin[axis2];
				for (size_t iCell = 0; iCell < numSteps; iCell++) {
					rayOrigin[axis0] = blockOrigin[axis0] + iCell * cellSpan[axis0];

					for (size_t jCell = 0; jCell < numSteps; jCell++) {
						rayOrigin[axis1] = blockOrigin[axis1] + jCell * cellSpan[axis1];

						Ray ray(rayOrigin, rayDir);
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

								Vector3i blockIndex = assignAxes(Vector3i(iBlk, jBlk, rayBlockIdx), axisOrder);
								size_t bIdx = calLinearBlockIndex(blockIndex);
								if (bIdx < cellsToCreate.size()) {
									vector<bool>& blockCellsToCreate = cellsToCreate[bIdx];

									for (int di = -1; di < 2; di++) {
										size_t iiCell = iCell + di;
										if (iiCell >= bd)
											continue;

										for (int dj = -1; dj < 2; dj++) {
											size_t jjCell = jCell + dj;
											if (jjCell >= bd)
												continue;

											if (blockCellsToCreate.empty())
												blockCellsToCreate.resize(numBlockCells);

											Vector3i cellIndex = assignAxes(Vector3i(iiCell, jjCell, rayCellIdx), axisOrder);
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
		}
	}, false && RUN_MULTI_THREAD);
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

	vector<vector<bool>> cellsToCreate;
	cellsToCreate.resize(_blockDim[0] * _blockDim[1] * _blockDim[2]);
	createBlockCellRays(pTriMesh, AxisIndex::X, blocksToCreate, cellsToCreate);
	createBlockCellRays(pTriMesh, AxisIndex::Y, blocksToCreate, cellsToCreate);
	createBlockCellRays(pTriMesh, AxisIndex::Z, blocksToCreate, cellsToCreate);

	std::cout << "Blockdim: " << _blockDim << "\n";
	MultiCore::runLambda([this, pTriMesh, &cellsToCreate](size_t threadNum, size_t numThreads) {
		for (size_t i = threadNum; i < _blockDim[0]; i += numThreads) {
			for (size_t j = 0; j < _blockDim[1]; j++) {
				for (size_t k = 0; k < _blockDim[2]; k++) {

					size_t bIdx = calLinearBlockIndex(i, j, k);
					if (!cellsToCreate[bIdx].empty()) {
						Block* pBlock;
						if (_blocks[bIdx] == -1) {
							_blocks[bIdx] = _blockPool.getObj(bIdx, pBlock, true);
						} else {
							pBlock = _blockPool.getObj(bIdx);
						}
						if (pBlock) {
							pBlock->createCells(cellsToCreate[bIdx]);
						}
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
