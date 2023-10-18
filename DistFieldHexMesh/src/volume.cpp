

#include <triMesh.h>

#include <cell.h>
#include <block.h>
#include <volume.h>
#include <MultiCoreUtil.h>

using namespace std;
using namespace DFHM;
using namespace TriMesh;

#define RUN_MULTI_THREAD true

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

inline Vector3i Volume::getAxisOrder(AxisIndex axisIdx)
{
	const Vector3i axisOrder = axisIdx == AxisIndex::Z ? Vector3i(0, 1, 2) : (axisIdx == AxisIndex::Y ? Vector3i(2, 0, 1) : Vector3i(1, 2, 0));
	return axisOrder;
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

void Volume::createBlockRays(const TriMesh::CMeshPtr& pTriMesh, AxisIndex axisIdx, std::vector<bool>& blocksToCreate)
{
	const Vector3i axisOrder = getAxisOrder(axisIdx);

	const Vector3d rayDir = axisIdx == AxisIndex::Z ? Vector3d(0, 0, 1) : (axisIdx == AxisIndex::Y ? Vector3d(0, 1, 0) : Vector3d(1, 0, 0));

	vector<shared_ptr<RayBlockIntersectVec>>& allHits = axisIdx == AxisIndex::Z ? _zHits : (axisIdx == AxisIndex::Y ? _yHits : _xHits);
	const size_t numI = _blockDim[axisOrder[0]] + 1;
	const size_t numJ = _blockDim[axisOrder[1]] + 1;

	allHits.resize(numI * numJ);
	MultiCore::runLambda([this, pTriMesh, axisOrder, rayDir, numI, numJ, &allHits](size_t threadNum, size_t numThreads) {
		const double TOL = 1.0e-6;
		const size_t axis0 = axisOrder[0];
		const size_t axis1 = axisOrder[1];
		const size_t axis2 = axisOrder[2];

		Vector3d origin(_originMeters);

		for (size_t i = 0; i < numI; i++) {
			double t = i / (numI - 1.0);
			origin[axis0] = _originMeters[axis0] + t * _spanMeters[axis0];

			for (size_t j = 0; j < numJ; j++) {
				const size_t rayIdx = i + numI * j;
				if (rayIdx % numThreads != threadNum)
					continue;

				const double blockSpan = _spanMeters[axis2] / _blockDim[axis2];
				const double cellSpan = blockSpan / Block::getBlockDim();

				double u = j / (numJ - 1.0);
				origin[axis1] = _originMeters[axis1] + u * _spanMeters[axis1];

				Ray ray(origin, rayDir);
				vector<RayHit> hits;
				if (pTriMesh->rayCast(ray, hits)) {
					if (!allHits[rayIdx])
						allHits[rayIdx] = make_shared<RayBlockIntersectVec>();
					auto& blockHits = *allHits[rayIdx];

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
						double dist1 = dist0 - (rti._blockIdx * blockSpan);

						double w1 = dist1 / blockSpan;
						rti._cellIdx = (size_t)(w1 * Block::getBlockDim());
						if (rti._cellIdx >= Block::getBlockDim())
							rti._cellIdx = Block::getBlockDim() - 1;

						double dist2 = dist1 - (rti._cellIdx * cellSpan);

						rti._w = dist2 / cellSpan;
						rayHits.push_back(rti);
					}
				}
			}
		}
	}, RUN_MULTI_THREAD);

	size_t numHits = 0;
	for (const auto& pHits : allHits) {
		if (pHits)
			numHits += pHits->size();
	}
	cout << "numHits: " << numHits << "\n";

	if (blocksToCreate.empty())
		blocksToCreate.resize(_blockDim[0] * _blockDim[1] * _blockDim[2], false);

	MultiCore::runLambda([this, pTriMesh, axisOrder, rayDir, &allHits, &blocksToCreate](size_t threadNum, size_t numThreads) {
		const size_t axis0 = axisOrder[0];
		const size_t axis1 = axisOrder[1];
		const size_t axis2 = axisOrder[2];

		for (size_t i = 0; i < _blockDim[axis0]; i++) {
			for (size_t j = 0; j < _blockDim[axis1]; j++) {
				const size_t rayIdx = i + (_blockDim[axis0] + 1) * j;
				if (rayIdx % numThreads != threadNum || !allHits[rayIdx])
					continue;

				const RayBlockIntersectVec& kBlockHits = *allHits[rayIdx];
				if (!kBlockHits.empty()) {
					const RayTriIntersectVec& kHits = kBlockHits.front();
					for (const auto& kHit : kHits) {
						size_t k = kHit._blockIdx;
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
								if (bIdx < blocksToCreate.size())
									blocksToCreate[bIdx] = true;
								else
									cout << "block index out  of bounds\n";
							}
						}
					}
				}
			}

		}
	}, RUN_MULTI_THREAD);
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
	createBlockRays(pTriMesh, AxisIndex::X, blocksToCreate);
	createBlockRays(pTriMesh, AxisIndex::Y, blocksToCreate);
	createBlockRays(pTriMesh, AxisIndex::Z, blocksToCreate);

	MultiCore::runLambda([this, pTriMesh, &blocksToCreate](size_t threadNum, size_t numThreads) {
		Vector3d blockSpan;
		for (int i = 0; i < 3; i++) {
			blockSpan[i] = _spanMeters[i] / (double)_blockDim[i];
		}
		for (size_t i = threadNum; i < blocksToCreate.size(); i += numThreads) {
			if (blocksToCreate[i]) {
				_blocks[i] = _blockPool.create();
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

